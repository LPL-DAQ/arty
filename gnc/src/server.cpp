#include <cctype>
#include <zephyr/logging/log.h>
#include <zephyr/net/socket.h>
#include <zephyr/sys/errno_private.h>
#include <zephyr/net/net_pkt.h>
#include <sstream>
#include <string>
#include <array>
#include <climits>

#include "throttle_valve.h"
#include "server.h"
#include "guards/SocketGuard.h"
#include "pts.h"
#include "sequencer.h"


LOG_MODULE_REGISTER(Server, CONFIG_LOG_DEFAULT_LEVEL
);

#define MAX_OPEN_CLIENTS 3

/// Main server thread must acquire one of these before accepting a connection. It must then scan through the thread
/// array to find an open slot.
K_SEM_DEFINE(num_open_connections,
             MAX_OPEN_CLIENTS, MAX_OPEN_CLIENTS);

bool has_thread[MAX_OPEN_CLIENTS] = {false};
K_MUTEX_DEFINE(has_thread_lock);

static k_thread client_threads[MAX_OPEN_CLIENTS] = {nullptr};
#define CONNECTION_THREAD_STACK_SIZE (4 * 1024)
K_THREAD_STACK_ARRAY_DEFINE(client_stacks,
                            MAX_OPEN_CLIENTS, CONNECTION_THREAD_STACK_SIZE);

/// Helper that sends a payload completely through an socket
int send_fully(int sock, const char* buf, int len)
{
    int bytes_sent = 0;
    while (bytes_sent < len) {
        int ret = zsock_send(sock, buf + bytes_sent,
                             len - bytes_sent, 0);
        if (ret < 0) {
            LOG_ERR("Unexpected error while sending response to sock %d: err %d", sock, ret);
            return ret;
        }
        bytes_sent += ret;
    }
    return 0;
}

int send_string_fully(int sock, const std::string &payload)
{
    return send_fully(sock, payload.c_str(), std::ssize(payload));
}

/// Handles a client connection. Should run in its own thread.
static void handle_client(void* p1_client_socket, void*, void*)
{
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

            if (bytes_read == 0) {
                LOG_INF("Client at sock %d has closed their connection.", client_guard.socket);
                return;
            }
            else if (bytes_read < 0) {
                LOG_WRN("Failed to read bytes: errno %d", errno);
                return;
            }

            // Ignore whitespace
            if (std::isspace(command_buf[next_command_byte])) {
                continue;
            }
            if (command_buf[next_command_byte] == '#') {
                command_buf[next_command_byte + 1] = '\0';
                break;
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
//            throttle_valve_start_calibrate();
            send_string_fully(client_guard.socket, "Done calibrating\n");
        }
        else if (command == ("resetfuelopen#")) {
            FuelValve::reset_pos(90.0f);
            send_string_fully(client_guard.socket, "Done reset fuel open\n");
        }
        else if (command == "resetloxopen#") {
            LoxValve::reset_pos(90.0f);
            send_string_fully(client_guard.socket, "Done reset lox open\n");
        }
        else if (command == ("resetfuelclose#")) {
            FuelValve::reset_pos(0.0f);
            send_string_fully(client_guard.socket, "Done reset fuel close\n");
        }
        else if (command == "resetloxclose#") {
            LoxValve::reset_pos(0.0f);
            send_string_fully(client_guard.socket, "Done reset lox close\n");
        }
        else if (command.starts_with("seqsinemotr")) {
            // Example: seqsinemotr5000,50,40,1000,180#
            // In order, specify total time, offset, amplitude, period, and phase (in integer deg). The example shown
            // will oscillate between 10 and 90 deg with a period of 1000ms over 5000ms, starting from the lower valley
            // of a sine wave.

            std::array<float, 5> values;
            int next_values = 0;
            float curr_token = 0.0f;
            bool invalid = false;
            for (int i = 11; i < std::ssize(command); ++i) {
                if (!(command[i] >= '0' && command[i] <= '9')) {
                    if (next_values == 5) {
                        send_string_fully(client_guard.socket, "Must have at most 5 in sequence.");
                        invalid = true;
                        break;
                    }
                    values[next_values] = curr_token;
                    next_values++;
                    curr_token = 0;
                }
                else {
                    curr_token = 10 * curr_token + (command[i] - '0');
                }
            }
            if (invalid) {
                break;
            }
            if (next_values < 5) {
                send_string_fully(client_guard.socket, "Must have 5 parameters in sequence.\n");
                continue;
            }

            int err = sequencer_prepare_sine(values[0], values[1], values[2], values[3], values[4]);
            if (err) {
                send_string_fully(client_guard.socket, "Failed to prepare sine sequence.\n");
                continue;
            }
            send_string_fully(client_guard.socket, "Sine sequence prepared.\n");
        }
        else if (command == "halt#") {
            sequencer_halt();
        }
        else if (command.starts_with("seq")) {
            // Example: seqmotr500;75_75,52_52,70_60,90_30
            // Use seqmotr to specify motor open-loop setpoints, and use seqctrl to specify setpoints for a closed-loop
            // control sequence.

            std::vector<float> seq_fuel_breakpoints{
                0.0f}; // Initial dummy value, will get rewritten to current when sequence starts.
            std::vector<float> seq_lox_breakpoints{
                0.0f}; // Initial dummy value, will get rewritten to current when sequence starts.

            bool motor_only;
            std::string seq_kind = command.substr(3, 4);
            if (seq_kind == "motr") {
                motor_only = true;
            }
            else if (seq_kind == "ctrl") {
                motor_only = false;
            }
            else {
                send_string_fully(client_guard.socket, "Invalid sequence kind (must be seqmotr or seqctrl)\n");
                continue;
            }

            // Mini token parser
            int gap = 0;
            bool wrote_gap = false;
            float fuel_token = 0;
            float curr_token = 0;
            bool saw_decimal = false;
            int num_decimals = 0;
            for (int i = 7; i < std::ssize(command); ++i) {
                if (!((command[i] >= '0' && command[i] <= '9') || command[i] == '.')) {
                    if (command[i] == '_') {
                        fuel_token = curr_token;
                    }
                    else if (wrote_gap) {
                        seq_fuel_breakpoints.push_back(fuel_token);
                        seq_lox_breakpoints.push_back(curr_token);
                        fuel_token = 0;
                    }
                    else {
                        gap = curr_token;
                        wrote_gap = true;
                    }
                    curr_token = 0;
                    saw_decimal = false;
                    num_decimals = 0;
                }
                else {
                    if (command[i] == '.') {
                        saw_decimal = true;
                    }
                    else if (!saw_decimal) {
                        curr_token = 10.0f * curr_token + (command[i] - '0');
                    }
                    else {
                        float multiplier = 0.1f;
                        for (int j = 0; j < num_decimals; ++j) {
                            multiplier *= 0.1f;
                        }
                        num_decimals++;
                        curr_token += (command[i] - '0') * multiplier;
                    }
                }
            }

            if (seq_fuel_breakpoints.size() != seq_lox_breakpoints.size()) {
                send_string_fully(client_guard.socket, "Fuel breakpoints length not same as lox breakpoints\n");
                continue;
            }

            if (seq_lox_breakpoints.size() <= 1) {
                send_string_fully(client_guard.socket, "Breakpoints too short\n");
                continue;
            }
            int time_ms = (std::ssize(seq_lox_breakpoints) - 1) * gap;
            if (sequencer_prepare(gap, seq_fuel_breakpoints, seq_lox_breakpoints, motor_only)) {
                send_string_fully(client_guard.socket, "Failed to prepare sequence");
                continue;
            }
            std::string msg = "Breakpoints prepared, length is: " + std::to_string(time_ms) + "ms\n";
            send_string_fully(client_guard.socket, msg.c_str());
        }
        else if (command.starts_with("comboseq")) {
            // Example: comboseqmotr500;75_75,s50,52_52,70_60,s20,90_30
            // Use seqmotr to specify motor open-loop setpoints, and use seqctrl to specify setpoints for a closed-loop
            // control sequence.

            std::vector<float> seq_fuel_breakpoints{
                0.0f}; // Initial dummy value, will get rewritten to current when sequence starts.
            std::vector<float> seq_lox_breakpoints{
                0.0f}; // Initial dummy value, will get rewritten to current when sequence starts.

            // offset for sinusoidal segments (0.0f for linear segments)
            std::vector<float> seq_sine_offsets;

            bool motor_only;
            std::string seq_kind = command.substr(8, 4);
            if (seq_kind == "motr") {
                motor_only = true;
            }
            else if (seq_kind == "ctrl") {
                motor_only = false;
            }
            else {
                send_string_fully(client_guard.socket, "Invalid sequence kind (must be seqmotr or seqctrl)\n");
                continue;
            }

            int gap = 0;
            bool wrote_gap = false;
            float fuel_token = 0;
            float curr_token = 0;
            bool saw_decimal = false;
            int num_decimals = 0;

            int sine_offset = 0;
            bool saw_sine = false;

            for (int i = 12; i < std::ssize(command); ++i) {                // Start of a sinusoidal token: "sXX"
                if (command[i] == 's') {
                    saw_sine = true;
                    curr_token = 0.0f;
                    saw_decimal = false;
                    num_decimals = 0;
                    continue;
                }

                if (!((command[i] >= '0' && command[i] <= '9') || command[i] == '.')) {
                    if (saw_sine) {
                        sine_offset = static_cast<int>(curr_token);
                        saw_sine = false;

                        // Create a new breakpoint at the current position
                        float last_fuel = seq_fuel_breakpoints.back();
                        float last_lox  = seq_lox_breakpoints.back();
                        seq_fuel_breakpoints.push_back(last_fuel);
                        seq_lox_breakpoints.push_back(last_lox);

                        // Segment from previous bp to this bp is sinusoidal
                        if (std::ssize(seq_fuel_breakpoints) > 1) {
                            seq_sine_offsets.push_back(static_cast<float>(sine_offset));
                        }
                        curr_token = 0.0f;
                        saw_decimal = false;
                        num_decimals = 0;
                    }
                    else if (command[i] == '_') {
                        fuel_token = curr_token;
                    }
                    else if (wrote_gap) {
                        seq_fuel_breakpoints.push_back(fuel_token);
                        seq_lox_breakpoints.push_back(curr_token);
                        if (std::ssize(seq_fuel_breakpoints) > 1) {
                                seq_sine_offsets.push_back(0.0f);
                        }
                        fuel_token = 0;
                    }
                    else {
                        gap = curr_token;
                        wrote_gap = true;
                    }
                    curr_token = 0;
                    saw_decimal = false;
                    num_decimals = 0;
                }
                else {
                    if (command[i] == '.') {
                        saw_decimal = true;
                    }
                    else if (!saw_decimal) {
                        curr_token = 10.0f * curr_token + (command[i] - '0');
                    }
                    else {
                        float multiplier = 0.1f;
                        for (int j = 0; j < num_decimals; ++j) {
                            multiplier *= 0.1f;
                        }
                        num_decimals++;
                        curr_token += (command[i] - '0') * multiplier;
                    }
                }

            }

            if (seq_fuel_breakpoints.size() != seq_lox_breakpoints.size()) {
                send_string_fully(client_guard.socket, "Fuel breakpoints length not same as lox breakpoints\n");
                continue;
            }

            if (seq_lox_breakpoints.size() <= 1) {
                send_string_fully(client_guard.socket, "Breakpoints too short\n");
                continue;
            }
            int time_ms = (std::ssize(seq_lox_breakpoints) - 1) * gap;
            if (sequencer_prepare_combo(gap, seq_fuel_breakpoints, seq_lox_breakpoints, seq_sine_offsets, motor_only)) {
                send_string_fully(client_guard.socket, "Failed to prepare sequence");
                continue;
            }
            std::string msg = "Breakpoints prepared, length is: " + std::to_string(time_ms) + "ms\n";
            send_string_fully(client_guard.socket, msg.c_str());
        }


        else if (command == "getfuelpos#") {
            double fuel_pos = FuelValve::get_pos_internal();
            std::string payload = "valve pos: " + std::to_string(fuel_pos) + " deg\n";
            int err = send_fully(client_guard.socket, payload.c_str(), std::ssize(payload));
            if (err) {
                LOG_ERR("Failed to fully send valve pos: err %d", err);
            }
        }
        else if (command == "getloxpos#") {
            double lox_pos = LoxValve::get_pos_internal();
            std::string payload = "valve pos: " + std::to_string(lox_pos) + " deg\n";
            int err = send_fully(client_guard.socket, payload.c_str(), std::ssize(payload));
            if (err) {
                LOG_ERR("Failed to fully send valve pos: err %d", err);
            }
        }
        else if (command == "getpts#") {
            pt_readings readings = pts_sample();
            std::string payload =
                "pt102: " + std::to_string(readings.pt102) + "\n"
                + "pt103: " + std::to_string(readings.pt103) + "\n"
                + "pt202: " + std::to_string(readings.pt202) + "\n"
                + "pt203: " + std::to_string(readings.pt203) + "\n"
                + "ptf401: " + std::to_string(readings.ptf401) + "\n"
                + "pto401: " + std::to_string(readings.pto401) + "\n"
                + "ptc401: " + std::to_string(readings.ptc401) + "\n"
                + "ptc402: " + std::to_string(readings.ptc402) + "\n";
            int err = send_fully(client_guard.socket, payload.c_str(), std::ssize(payload));
            if (err) {
                LOG_ERR("Failed to fully send pt readings: err %d", err);
            }
        }
        else if (command == "START#") {
            send_string_fully(client_guard.socket, "ACK#");
            // Triggered in DAQ sequencer.
            LOG_INF("Triggering sequence from DAQ.");
            int err = sequencer_start_trace();
            if (err) {
                LOG_ERR("Failed to run sequence: err %d", err);
                continue;
            }
        }
        else if (command == "listen#") {
            send_string_fully(client_guard.socket,
                              "Don't send additional commands till the sequence is done, lest the output be mangled.\n");
            send_string_fully(client_guard.socket, "Listening for sequence...\n");
            sequencer_set_data_recipient(client_guard.socket);
        }
        else if (command == "dstart#") {
            // Triggered manually.
            sequencer_set_data_recipient(client_guard.socket);
            int err = sequencer_start_trace();
            if (err) {
                LOG_ERR("Failed to run sequence: err %d", err);
                send_string_fully(client_guard.socket, "Failed to run sequence\n");
                continue;
            }
            send_string_fully(client_guard.socket, "Done sequence.\n");
        }
        else if (command.starts_with("configpt")) {
            // Configure the pt bias as such:
            // configptbias,pt203,-5#
            // Or set the PT range (e.g., 1k PT) as such:
            // configptrang,pt203,2000#

            bool config_bias_not_range = false;
            std::string config_what = command.substr(8, 4);
            if (config_what == "bias") {
                config_bias_not_range = true;
            }
            else if (config_what == "rang") {
                config_bias_not_range = false;
            }
            else {
                LOG_ERR("Invalid config option for PT");
                continue;
            }

            std::string pt_name;
            float value = 0;
            bool bias_is_negative = false;
            bool in_label_segment = true;
            for (int i = 13; i < std::ssize(command) - 1; ++i) {
                if (command[i] != ',') {
                    if (in_label_segment) {
                        pt_name += command[i];
                    }
                    else {
                        if (command[i] == '-') {
                            bias_is_negative = true;
                        }
                        else {
                            value = value * 10.0f + static_cast<float>(command[i] - '0');
                        }
                    }
                }
                else {
                    in_label_segment = false;
                }
            }
            if (bias_is_negative) {
                value *= -1;
            }

            int pt_index = -1;
            if (pt_name == "ptc401") {
                pt_index = 0;
            }
            else if (pt_name == "pto401") {
                pt_index = 1;
            }
            else if (pt_name == "pt103") {
                pt_index = 2;
            }
            else if (pt_name == "pt202") {
                pt_index = 3;
            }
            else if (pt_name == "pt102") {
                pt_index = 4;
            }
            else if (pt_name == "ptf401") {
                pt_index = 5;
            }
            else if (pt_name == "pt203") {
                pt_index = 6;
            }
            else if (pt_name == "ptc402") {
                pt_index = 7;
            }
            else {
                LOG_ERR("Invalid pt name: %s", pt_name.c_str());
                continue;
            }

            int err = 0;
            if (config_bias_not_range) {
                err = pts_set_bias(pt_index, value);
            }
            else {
                err = pts_set_range(pt_index, value);
            }
            if (err) {
                LOG_ERR("Failed to set PT bias: err %d", err);
                continue;
            }
            if (config_bias_not_range) {
                send_string_fully(client_guard.socket, "Set PT bias.\n");
            }
            else {
                send_string_fully(client_guard.socket, "Set PT range.\n");
            }
        }
        else if (command == "getptconfigs#") {
            std::string payload;
            std::array<std::string, 4> index_to_pt{
                "UNUSED",
                "pt202",
                "pt203",
                "ptf401"
            };
            for (int i = 1; i < 4; ++i) {
                payload += index_to_pt[i] + ": bias=" + std::to_string(pt_configs[i].bias) + " psig, range=" +
                           std::to_string(pt_configs[i].range) + " psig\n";
            }
            send_string_fully(client_guard.socket, payload);
        }
        else {
            LOG_WRN("Unknown command.");
        }
    }
}

/// Attempts to join connection handler threads, allowing the thread slots to be reused to service new connection.
[[noreturn]] static void reap_dead_connections(void*, void*, void*)
{
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
                }
                else if (ret == -EBUSY) {
                    // Thread still running
                }
                else {
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

K_THREAD_DEFINE(server_reaper,
                1024, reap_dead_connections, nullptr, nullptr, nullptr, 1, 0, 0);


/// Opens a TCP server, listens for incoming clients, and spawns new threads to serve these connections. This function
/// blocks indefinitely.
void serve_connections()
{
    LOG_INF("Opening server socket");
    int server_socket = zsock_socket(AF_INET, SOCK_STREAM, IPPROTO_TCP);
    if (server_socket < 0) {
        LOG_ERR("Failed to create TCP socket: %d", errno);
        return;
    }

    sockaddr_in bind_addr = {};
    bind_addr.sin_family = AF_INET;
    bind_addr.sin_addr.s_addr = htonl(INADDR_ANY);
    bind_addr.sin_port = htons(19690);

    LOG_INF("Binding server socket to address");
    int err = zsock_bind(server_socket, reinterpret_cast<sockaddr*>(&bind_addr), sizeof(bind_addr));
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
                        reinterpret_cast<k_thread_stack_t*>(&client_stacks[connection_index]),
                        CONNECTION_THREAD_STACK_SIZE,
                        handle_client,
                        reinterpret_cast<void*>(client_socket), nullptr, nullptr, 5, 0, K_NO_WAIT
        );

        k_mutex_lock(&has_thread_lock, K_FOREVER);
        has_thread[connection_index] = true;
        k_mutex_unlock(&has_thread_lock);
    }
}
