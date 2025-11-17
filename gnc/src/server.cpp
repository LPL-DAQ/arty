#include <cctype>
#include <zephyr/logging/log.h>
#include <zephyr/net/socket.h>
#include <zephyr/sys/errno_private.h>
#include <zephyr/net/net_pkt.h>
#include <sstream>
#include <string>
#include <array>

#include "throttle_valve.h"
#include "server.h"
#include "guards/SocketGuard.h"
#include "pts.h"
#include "sequencer.h"


LOG_MODULE_REGISTER(Server, CONFIG_LOG_DEFAULT_LEVEL);

#define MAX_OPEN_CLIENTS 4

/// Main server thread must acquire one of these before accepting a connection. It must then scan through the thread
/// array to find an open slot.
K_SEM_DEFINE(num_open_connections, MAX_OPEN_CLIENTS, MAX_OPEN_CLIENTS);

bool has_thread[MAX_OPEN_CLIENTS] = {false};
K_MUTEX_DEFINE(has_thread_lock);

static k_thread client_threads[MAX_OPEN_CLIENTS] = {nullptr};
#define CONNECTION_THREAD_STACK_SIZE (4 * 1024)
K_THREAD_STACK_ARRAY_DEFINE(client_stacks, MAX_OPEN_CLIENTS, CONNECTION_THREAD_STACK_SIZE);

/// Helper that sends a payload completely through an socket
int send_fully(int sock, const char *buf, int len) {
    int bytes_sent = 0;
    while (bytes_sent < len) {
        int ret = zsock_send(sock, buf + bytes_sent,
                             len - bytes_sent, 0);
        if (ret < 0) {
            LOG_ERR("Unexpected error while sending response: err %d", ret);
            return ret;
        }
        bytes_sent += ret;
    }
    return 0;
}

int send_string_fully(int sock, const std::string &payload) {
    return send_fully(sock, payload.c_str(), std::ssize(payload));
}

/// Handles a client connection. Should run in its own thread.
static void handle_client(void *p1_client_socket, void *, void *) {
    SocketGuard client_guard{reinterpret_cast<int>(p1_client_socket)};
    LOG_INF("Handling socket: %d", client_guard.socket);
    k_sleep(K_MSEC(500));

    while (true) {
        // Read one byte at a time till we get a #-terminated command
        constexpr int MAX_COMMAND_LEN = 512;
        char command_buf[MAX_COMMAND_LEN + 1];
        int next_command_byte = 0;
        while (true) {
            ssize_t bytes_read = zsock_recv(client_guard.socket, command_buf + next_command_byte, 1, 0);
            if (bytes_read < 0) {
                LOG_WRN("Failed to read bytes: errno %d", errno);
                return;
            }
            if (bytes_read) {
                // Ignore whitespace
                if (std::isspace(command_buf[next_command_byte])) {
                    continue;
                }
                if (command_buf[next_command_byte] == '#') {
                    command_buf[next_command_byte + 1] = '\0';
                    break;
                }
            }
            next_command_byte += bytes_read;
            if (next_command_byte == MAX_COMMAND_LEN) {
                LOG_WRN("Didn't find command terminator `#` after %d bytes", MAX_COMMAND_LEN);
                return;
            }
        }

        LOG_INF("Got command: %s", command_buf);
        std::string command(command_buf); // TODO: Heap allocation? perhaps stick with annoying cstring?

        if (command == "calibrate#") {
            throttle_valve_start_calibrate();
            send_string_fully(client_guard.socket, "Done calibrating\n");
        } else if (command == "resetopen#") {
            // Sets the current position as 90 deg WITHOUT moving the valve.
            throttle_valve_set_open();
            send_string_fully(client_guard.socket, "Done reset open\n");
        } else if (command == "resetclose#") {
            // Sets the current position as 0 deg WITHOUT moving the valve.
            throttle_valve_set_closed();
            send_string_fully(client_guard.socket, "Done reset close\n");
        } else if (command.starts_with("seq")) {
            // Example: seq500;75.5,52.0,70,90, where 500 -> 500ms between each breakpoint and
            // the commas-seperated values are the breakpoints in degrees.
            // NOTE: An initial breakpoint, representing the valve current starting position,
            // is implicitly added. Thus, the example shown will run for 2s as we actually start
            // at, say, 90 deg.
            // Also, please do not give invalid input :) :) :)

            std::vector<float> seq_breakpoints;

            // Mini token parser
            int gap = 0;
            seq_breakpoints.clear();
            seq_breakpoints.push_back(throttle_valve_get_pos());
            bool wrote_gap = false;
            int curr_token = 0;
            bool is_neg = false;
            for (int i = 3; i < std::ssize(command) - 1; ++i) {
                if (!(command[i] >= '0' && command[i] <= '9')) {
                    if (command[i] == '-') {
                        is_neg = true;
                    } else if (wrote_gap) {
                        if (is_neg) {
                            curr_token = -curr_token;
                        }
                        seq_breakpoints.push_back(curr_token);
                        is_neg = false;
                    } else {
                        gap = curr_token;
                        wrote_gap = true;
                    }
                    curr_token = 0;
                } else {
                    curr_token = 10 * curr_token + (command[i] - '0');
                }
            }
            if (is_neg) {
                curr_token = -curr_token;
            }
            seq_breakpoints.push_back(curr_token);
            is_neg = false;

            if (seq_breakpoints.size() <= 1) {
                send_string_fully(client_guard.socket, "Breakpoints too short\n");
                continue;
            }
            int time_ms = (std::ssize(seq_breakpoints) - 1) * gap;
            if (sequencer_prepare(gap, seq_breakpoints)) {
                send_string_fully(client_guard.socket, "Failed to prepare sequence");
                continue;
            }
            std::string msg = "Breakpoints prepared, length is: " + std::to_string(time_ms) + "ms\n";
            send_string_fully(client_guard.socket, msg.c_str());

        } else if (command == "getpos#") {
            double pos = throttle_valve_get_pos();
            std::string payload = "valve pos: " + std::to_string(pos) + " deg\n";
            int err = send_fully(client_guard.socket, payload.c_str(), std::ssize(payload));
            if (err) {
                LOG_ERR("Failed to fully send valve pos: err %d", err);
            }

        } else if (command == "getpts#") {
            pt_readings readings = pts_sample();
            std::string payload =
                    "pt203: " + std::to_string(readings.pt203) + ", pt202: " + std::to_string(readings.pt203) +
                    ", ptf401: " + std::to_string(readings.ptf401) + ", pt102: " + std::to_string(readings.pt102) +
                    "\n";
            int err = send_fully(client_guard.socket, payload.c_str(), std::ssize(payload));
            if (err) {
                LOG_ERR("Failed to fully send pt readings: err %d", err);
            }
        } else if (command == "START#") {
            send_string_fully(client_guard.socket, "ACK#");
            // Triggered in DAQ sequencer.
            LOG_INF("Triggering sequence from DAQ.");
            int err = sequencer_start_trace();
            if (err) {
                LOG_ERR("Failed to run sequence: err %d", err);
                continue;
            }
        } else if (command == "listen#") {
            send_string_fully(client_guard.socket,
                              "Don't send additional commands till the sequence is done, lest the output be mangled.\n");
            send_string_fully(client_guard.socket, "Listening for sequence...\n");
            sequencer_set_data_recipient(client_guard.socket);
        } else if (command == "dstart#") {
            // Triggered manually.
            sequencer_set_data_recipient(client_guard.socket);
            int err = sequencer_start_trace();
            if (err) {
                LOG_ERR("Failed to run sequence: err %d", err);
                send_string_fully(client_guard.socket, "Failed to run sequence\n");
                continue;
            }
            send_string_fully(client_guard.socket, "Done sequence.\n");
        } else if (command.starts_with("configpt")) {
            // Configure the pt bias as such:
            // configptbias,pt203,-5#
            // Or set the PT range (e.g., 1k PT) as such:
            // configptrang,pt203,2000#

            bool config_bias_not_range = false;
            std::string config_what = command.substr(8, 4);
            if (config_what == "bias") {
                config_bias_not_range = true;
            } else if (config_what == "rang") {
                config_bias_not_range = false;
            } else {
                LOG_ERR("Invalid config option for PT");
                continue;
            }

            std::string pt_name;
            float value = 0;
            bool bias_is_negative = false;
            bool in_label_segment = true;
            for (int i = 12; i < std::ssize(command) - 1; ++i) {
                if (command[i] != ',') {
                    if (in_label_segment) {
                        pt_name += command[i];
                    } else {
                        if (command[i] == '-') {
                            bias_is_negative = true;
                        } else {
                            value = value * 10.0f + static_cast<float>(command[i] - '0');
                        }
                    }
                } else {
                    in_label_segment = false;
                }
            }
            if (bias_is_negative) {
                value *= -1;
            }

            int pt_index = -1;
            if (pt_name == "pt202") {
                pt_index = 1;
            } else if (pt_name == "pt203") {
                pt_index = 2;
            } else if (pt_name == "ptf401") {
                pt_index = 3;
            } else {
                LOG_ERR("Invalid pt name: %s", pt_name.c_str());
                continue;
            }

            int err = 0;
            if (config_bias_not_range) {
                err = pts_set_bias(pt_index, value);
            } else {
                err = pts_set_range(pt_index, value);
            }
            if (err) {
                LOG_ERR("Failed to set PT bias: err %d", err);
                continue;
            }

            send_string_fully(client_guard.socket, "Set PT bias.");
        } else if (command == "getptconfigs#") {
            std::string payload;
            std::array<std::string, 4> index_to_pt{
                    "UNUSED",
                    "pt202",
                    "pt203",
                    "ptf401"
            };
            for (int i = 1; i < 4; ++i) {
                payload += index_to_pt[i] + ": bias=" + std::to_string(pt_configs[i].bias) + "psig, range=" +
                           std::to_string(pt_configs[i].range) + "psig\n";
            }
            send_string_fully(client_guard.socket, payload);
        } else {
            LOG_WRN("Unknown command.");
        }
    }
}

/// Attempts to join connection handler threads, allowing the thread slots to be reused to service new connection.
[[noreturn]] static void reap_dead_connections(void *, void *, void *) {
    bool freed_threads[MAX_OPEN_CLIENTS] = {false};
    while (true) {
        k_mutex_lock(&has_thread_lock, K_FOREVER);
        for (int i = 0; i < MAX_OPEN_CLIENTS; ++i) {
            if (has_thread[i]) {
                int ret = k_thread_join(&client_threads[i], K_NO_WAIT);
                if (ret == 0) {
                    has_thread[i] = false;
                    freed_threads[i] = true;
                    k_sem_give(&num_open_connections);
                } else if (ret == -EBUSY) {
                    // Thread still running
                } else {
                    LOG_ERR("Unexpected code from joining client thread: err %d", ret);
                }
            }
        }
        k_mutex_unlock(&has_thread_lock);

        // Log freed threads outside mutex
        for (int i = 0; i < MAX_OPEN_CLIENTS; ++i) {
            if (freed_threads[i]) {
                LOG_INF("Freed thread at slot %d", i);
                freed_threads[i] = false;
            }
        }

        k_sleep(K_MSEC(50));
    }
}

K_THREAD_DEFINE(server_reaper, 1024, reap_dead_connections, nullptr, nullptr, nullptr, 1, 0, 0);


/// Opens a TCP server, listens for incoming clients, and spawns new threads to serve these connections. This function
/// blocks indefinitely.
void serve_connections() {
    LOG_INF("Opening socket");
    k_sleep(K_MSEC(500));
    LOG_INF("Opening socket");
    k_sleep(K_MSEC(500));
    int server_socket = zsock_socket(AF_INET, SOCK_STREAM, IPPROTO_TCP);
    if (server_socket < 0) {
        LOG_ERR("Failed to create TCP socket: %d", errno);
        return;
    }
    LOG_INF("HEY!");
    k_sleep(K_MSEC(500));

    sockaddr_in bind_addr = {};
    bind_addr.sin_family = AF_INET;
    bind_addr.sin_addr.s_addr = htonl(INADDR_ANY);
    bind_addr.sin_port = htons(19690);

    LOG_INF("Binding socket to address");
    int err = zsock_bind(server_socket, reinterpret_cast<sockaddr *>(&bind_addr), sizeof(bind_addr));
    if (err) {
        LOG_ERR("Failed to bind to socket `%d`: %d", server_socket, err);
        return;
    }
    LOG_INF("Listening for open connections");
    err = zsock_listen(server_socket, 0);
    if (err) {
        LOG_ERR("Failed to listen on socket `%d`: %d", server_socket, err);
        return;
    }

    // Serve new connections indefinitely
    while (true) {
        // Wait for free thread slot
        err = k_sem_take(&num_open_connections, K_FOREVER);
        if (err) {
            LOG_INF("Failed to acquire semaphore: %d", err);
            return;
        }

        // Find open connection index
        int connection_index = 0;
        k_mutex_lock(&has_thread_lock, K_FOREVER);
        for (connection_index = 0; connection_index < MAX_OPEN_CLIENTS; ++connection_index) {
            if (!has_thread[connection_index]) {
                break;
            }
        }
        k_mutex_unlock(&has_thread_lock);
        if (connection_index == MAX_OPEN_CLIENTS) {
            LOG_ERR("Consistency error: Server acquired connection semaphore but no thread slots were open");
            return;
        }

        // Spawn thread to service client connection
        int client_socket = zsock_accept(server_socket, nullptr, nullptr);
        LOG_INF("Spawning thread in slot %d to serve socket %d", connection_index, client_socket);
        k_thread_create(&client_threads[connection_index],
                        reinterpret_cast<k_thread_stack_t *>(&client_stacks[connection_index]),
                        CONNECTION_THREAD_STACK_SIZE,
                        handle_client,
                        reinterpret_cast<void *>(client_socket), nullptr, nullptr, 5, 0, K_NO_WAIT
        );

        k_mutex_lock(&has_thread_lock, K_FOREVER);
        has_thread[connection_index] = true;
        k_mutex_unlock(&has_thread_lock);
    }
}
