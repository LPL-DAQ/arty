#include <array>
#include <cctype>
#include <climits>
#include <cstdint>
#include <pb_decode.h>
#include <pb_encode.h>
#include <sstream>
#include <string>
#include <zephyr/logging/log.h>
#include <zephyr/net/net_pkt.h>
#include <zephyr/net/socket.h>
#include <zephyr/posix/arpa/inet.h>
#include <zephyr/sys/errno_private.h>

#include "ThrottleValve.h"
#include "clover.pb.h"
#include "guards/SocketGuard.h"
#include "pts.h"
// ADDED: Replaced sequencer.h with our new static Controller which handles the state machine safely.
#include "Controller.h"
#include "server.h"

LOG_MODULE_REGISTER(Server, CONFIG_LOG_DEFAULT_LEVEL);

constexpr size_t MAX_MESSAGE_SIZE = 1024 * 8;
static_assert(Request_size <= MAX_MESSAGE_SIZE);
static_assert(Response_size <= MAX_MESSAGE_SIZE);

#define MAX_OPEN_CLIENTS 3

/// Main server thread must acquire one of these before accepting a connection. It must then scan through the thread
/// array to find an open slot.
K_SEM_DEFINE(num_open_connections, MAX_OPEN_CLIENTS, MAX_OPEN_CLIENTS);

bool has_thread[MAX_OPEN_CLIENTS] = {false};
K_MUTEX_DEFINE(has_thread_lock);

static k_thread client_threads[MAX_OPEN_CLIENTS] = {nullptr};
#define CONNECTION_THREAD_STACK_SIZE (6 * 1024)
K_THREAD_STACK_ARRAY_DEFINE(client_stacks, MAX_OPEN_CLIENTS, CONNECTION_THREAD_STACK_SIZE);

constexpr int MAX_THREAD_NAME_LENGTH = 10;
static std::array<std::string, MAX_OPEN_CLIENTS> thread_names;
K_MUTEX_DEFINE(thread_info_guard);

constexpr int MAX_DATA_CLIENTS = 1;
static std::array<sockaddr, MAX_DATA_CLIENTS> data_client_addrs;
static std::array<socklen_t, MAX_DATA_CLIENTS> data_client_addr_lens;
static std::array<int, MAX_DATA_CLIENTS> data_client_slot_indexes;
K_MUTEX_DEFINE(data_client_info_guard);

/// Synchronizes command/data server threads to ensure they only start accepting connections after startup is fully
/// done (i.e., when serve_connections() is called).
K_SEM_DEFINE(allow_serve_connections_sem, 0, 2);

/// Helper that sends a payload completely through an socket
int send_fully(int sock, const char* buf, int len)
{
    int bytes_sent = 0;
    while (bytes_sent < len) {
        int ret = zsock_send(sock, buf + bytes_sent, len - bytes_sent, 0);
        if (ret < 0) {
            LOG_ERR("Unexpected error while sending response to sock %d: err %d", sock, ret);
            return ret;
        }
        bytes_sent += ret;
    }
    return 0;
}

int send_string_fully(int sock, const std::string& payload)
{
    return send_fully(sock, payload.c_str(), std::ssize(payload));
}

/// Internal callback for pb_istream, used to read from socket to internal buffer and manage count.
bool pb_socket_write_callback(pb_ostream_t* stream, const uint8_t* buf, size_t count)
{
    int sock = reinterpret_cast<int>(stream->state);

    int bytes_sent = zsock_send(sock, buf, count, 0);
    if (bytes_sent < 0) {
        LOG_ERR("Error while sending data over socket %d: %s", sock, strerror(bytes_sent));
        return false;
    }

    if (bytes_sent != static_cast<int>(count)) {
        LOG_ERR("zsock_send only partially wrote its buffer from socket %d, this should be impossible - sent: %d, requested: %d", sock, bytes_sent, count);
        return false;
    }

    return true;
}

pb_ostream_t pb_ostream_from_socket(int sock)
{
    return pb_ostream_t{.callback = pb_socket_write_callback, .state = reinterpret_cast<void*>(sock), .max_size = SIZE_MAX, .bytes_written = 0};
}

/// Internal callback for pb_istream, used to read from socket to internal buffer.
bool pb_socket_read_callback(pb_istream_t* stream, uint8_t* buf, size_t count)
{
    int sock = reinterpret_cast<int>(stream->state);

    int bytes_read = zsock_recv(sock, buf, count, ZSOCK_MSG_WAITALL);

    // Hit EOF
    if (bytes_read == 0) {
        stream->bytes_left = 0;
        return false;
    }

    if (bytes_read != static_cast<int>(count)) {
        LOG_ERR(
            "zsock_recv only partially filled its buffer from sock %d, this should be impossible as we pass ZSOCK_MSG_WAITALL - got: %d, requested: %d",
            sock,
            bytes_read,
            count);
        return false;
    }

    return true;
}

/// Create a nanopb input stream from socket fd.
pb_istream_t pb_istream_from_socket(int sock)
{
    return pb_istream_t{.callback = pb_socket_read_callback, .state = reinterpret_cast<void*>(sock), .bytes_left = SIZE_MAX};
}

/// Handles a client connection. Should run in its own thread.
static void handle_client(void* p1_thread_index, void* p2_client_socket, void*)
{
    int thread_index = reinterpret_cast<int>(p1_thread_index);
    SocketGuard client_guard{reinterpret_cast<int>(p2_client_socket)};
    LOG_INF("Handling socket: %d", client_guard.socket);

    pb_istream_t pb_input = pb_istream_from_socket(client_guard.socket);
    pb_ostream_t pb_output = pb_ostream_from_socket(client_guard.socket);

    while (true) {
        Request request = Request_init_default;
        bool valid = pb_decode_ex(&pb_input, Request_fields, &request, PB_DECODE_DELIMITED);
        if (!valid) {
            LOG_INF("Failed to decode next message, this can happen if connection is severed.");
            break;
        }

        // ADDED: We use cmd_result to capture any domain logic errors returned by the Controller.
        // If a command (like loading a trace) fails, we bubble this error directly to the response.
        std::expected<void, Error> cmd_result = {};

        // Handle request
        switch (request.which_payload) {
        case Request_subscribe_data_stream_tag: {
            LOG_INF("Subscribe data stream");
            k_mutex_lock(&data_client_info_guard, K_FOREVER);
            bool found_data_client_slot = false;
            for (int i = 0; i < MAX_DATA_CLIENTS; ++i) {
                if (data_client_slot_indexes[i] != -1) {
                    continue;
                }
                found_data_client_slot = true;
                data_client_slot_indexes[i] = thread_index;

                int err = getpeername(client_guard.socket, &data_client_addrs[i], &data_client_addr_lens[i]);
                if (err) {
                    LOG_ERR("Failed to get peername when subscribing to data stream: err %d", err);
                }

                // Set client port
                reinterpret_cast<sockaddr_in*>(&data_client_addrs[i])->sin_port = htons(19691);

                // ADDED: Break out of loop once a slot is found so we don't accidentally overwrite multiple slots
                break;
            }

            if (!found_data_client_slot) {
                LOG_ERR("Did not find a data client slot");
            }

            k_mutex_unlock(&data_client_info_guard);
            break;
        }
        case Request_identify_client_tag: {
            LOG_INF("Identify client");
            break;
        }
        case Request_reset_valve_position_tag: {
            ResetValvePositionRequest& req = request.payload.reset_valve_position;
            switch (req.valve) {
            case Valve_FUEL:
                LOG_INF("Reset value fuel");
                // ADDED: Hooked up to actual ThrottleValve class
                FuelValve::reset_pos(0.0f);
                break;
            case Valve_LOX:
                LOG_INF("Reset valve lox");
                // ADDED: Hooked up to actual ThrottleValve class
                LoxValve::reset_pos(0.0f);
                break;
            default:
                LOG_ERR("Bad value.");
                break;
            }
            break;
        }
        case Request_load_motor_sequence_tag: {
            LOG_INF("Open loop motor sequence");
            // ADDED: Defer to the static controller to parse the protobuf trace.
            // If the trace has logic errors (e.g. invalid sequence points), it will return an Error.
            cmd_result = Controller::handle_load_motor_sequence(request.payload.load_motor_sequence);
            break;
        }
        case Request_start_sequence_tag: {
            LOG_INF("Start sequence");
            // ADDED: Kick off the 1ms timer in the Controller.
            cmd_result = Controller::handle_start_sequence(request.payload.start_sequence);
            break;
        }
        case Request_halt_sequence_tag: {
            LOG_INF("halt seq");
            // ADDED: Trigger the safe abort mode in the Controller.
            cmd_result = Controller::handle_halt_sequence(request.payload.halt_sequence);
            break;
        }
        default: {
            LOG_ERR(
                "Request has invalid tag, this should be impossible as pb_decode should have produced a valid Request - got tag: %u", request.which_payload);
            break;
        }
        }

        Response response = Response_init_default;

        // ADDED: If the Controller rejected the command, we format the Variadic Error
        // into the Protobuf response and send it back to the Ground Station immediately.
        if (!cmd_result.has_value()) {
            response.has_err = true;
            MaxLengthString<MAX_ERR_MESSAGE_SIZE> err_msg = cmd_result.error().build_message();

            // Safely copy the string using strncpy to prevent buffer overflows, ensuring null-termination
            strncpy(response.err, err_msg.c_str(), sizeof(response.err) - 1);
            response.err[sizeof(response.err) - 1] = '\0';

            LOG_ERR("Command rejected, returning error to client: %s", response.err);
        }

        // Send message over TCP with varint length prefix.
        bool ok = pb_encode_ex(&pb_output, Response_fields, &response, PB_ENCODE_DELIMITED);
        if (!ok) {
            LOG_ERR("Failed to encode command response: %s", pb_output.errmsg);
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
                    k_mutex_lock(&thread_info_guard, K_FOREVER);
                    thread_names[i].clear();
                    k_mutex_unlock(&thread_info_guard);

                    // Clean up potential data client subscription
                    k_mutex_lock(&data_client_info_guard, K_FOREVER);
                    for (int j = 0; j < MAX_DATA_CLIENTS; ++j) {
                        if (data_client_slot_indexes[j] == i) {
                            data_client_slot_indexes[j] = -1;
                            data_client_addr_lens[j] = sizeof(sockaddr);
                        }
                    }
                    k_mutex_unlock(&data_client_info_guard);

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

K_THREAD_DEFINE(server_reaper, 1024, reap_dead_connections, nullptr, nullptr, nullptr, 1, 0, 0);

/// Opens a TCP server, listens for incoming clients, and spawns new threads to serve these connections. This function
/// blocks indefinitely.
void serve_command_connections()
{
    k_sem_take(&allow_serve_connections_sem, K_FOREVER);

    LOG_INF("Opening command server socket");
    int server_socket = zsock_socket(AF_INET, SOCK_STREAM, IPPROTO_TCP);
    if (server_socket < 0) {
        LOG_ERR("Failed to create TCP socket: %d", errno);
        return;
    }

    LOG_INF("Binding command server socket to address");
    sockaddr_in bind_addr = {
        .sin_family = AF_INET,
        .sin_port = htons(19690),
        .sin_addr = in_addr{.s_addr = htonl(INADDR_ANY)},
    };
    int err = zsock_bind(server_socket, reinterpret_cast<sockaddr*>(&bind_addr), sizeof(bind_addr));
    if (err) {
        LOG_ERR("Failed to bind to socket `%d` for command server: %d", server_socket, err);
        return;
    }

    LOG_INF("Listening for command connections");
    err = zsock_listen(server_socket, 0);
    if (err) {
        LOG_ERR("Failed to listen on socket `%d` for command server: %d", server_socket, err);
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
        k_thread_create(
            &client_threads[connection_index],
            reinterpret_cast<k_thread_stack_t*>(&client_stacks[connection_index]),
            CONNECTION_THREAD_STACK_SIZE,
            handle_client,
            reinterpret_cast<void*>(connection_index),
            reinterpret_cast<void*>(client_socket),
            nullptr,
            5,
            0,
            K_NO_WAIT);

        k_mutex_lock(&has_thread_lock, K_FOREVER);
        has_thread[connection_index] = true;
        k_mutex_unlock(&has_thread_lock);
    }
}

K_THREAD_DEFINE(command_server, 4096, serve_command_connections, nullptr, nullptr, nullptr, 2, 0, 0);

/// Broadcasts UDP data packets using a multicast IP.
void serve_data_connections()
{
    k_sem_take(&allow_serve_connections_sem, K_FOREVER);

    LOG_INF("Opening data server socket");
    int server_socket = zsock_socket(AF_INET, SOCK_DGRAM, IPPROTO_UDP);
    if (server_socket < 0) {
        LOG_ERR("Failed to create UDP socket: %d", errno);
        return;
    }

    LOG_INF("Binding data server socket to address");
    sockaddr_in bind_addr = {
        .sin_family = AF_INET,
        .sin_port = htons(19691),
        .sin_addr = in_addr{.s_addr = htonl(INADDR_ANY)},
    };
    int err = zsock_bind(server_socket, reinterpret_cast<sockaddr*>(&bind_addr), sizeof(bind_addr));
    if (err) {
        LOG_ERR("Failed to bind to socket `%d` for data server: %d", server_socket, err);
        return;
    }

    // Serve new connections indefinitely
    while (true) {
        DataPacket data_packet;

        // ADDED: Instead of a hardcoded k_sleep loop pumping dummy data, this thread now blocks natively on the
        // thread-safe Zephyr message queue (telemetry_msgq) defined in Controller.cpp. The 1ms motor control loop
        // will drop packets into this queue asynchronously. This decouples the real-time control from networking!
        if (k_msgq_get(&telemetry_msgq, &data_packet, K_FOREVER) == 0) {

            // Encode data packet exactly ONCE per tick, regardless of how many UDP clients are subscribed
            uint8_t buf[DataPacket_size];
            pb_ostream_t data_packet_ostream = pb_ostream_from_buffer(buf, DataPacket_size);
            bool ok = pb_encode(&data_packet_ostream, DataPacket_fields, &data_packet);
            if (!ok) {
                LOG_ERR("Failed to encode data packet: %s", data_packet_ostream.errmsg);
                continue;
            }

            // Lock the client info guard and broadcast the already-encoded buffer to all subscribed IPs
            k_mutex_lock(&data_client_info_guard, K_FOREVER);
            for (int i = 0; i < MAX_DATA_CLIENTS; ++i) {
                if (data_client_slot_indexes[i] == -1) {
                    continue;
                }

                const int bytes_sent = zsock_sendto(server_socket, buf, data_packet_ostream.bytes_written, 0, &data_client_addrs[i], data_client_addr_lens[i]);

                // ADDED: static_cast to size_t to safely compare signed zsock_sendto return with unsigned bytes_written
                if (static_cast<size_t>(bytes_sent) != data_packet_ostream.bytes_written) {
                    LOG_ERR("Failed to send data packet over UDP: %d", bytes_sent);
                }
            }
            k_mutex_unlock(&data_client_info_guard);
        }
    }
}

K_THREAD_DEFINE(data_server, 1024 * 4, serve_data_connections, nullptr, nullptr, nullptr, 2, 0, 0);

/// Called at the end of startup, allowing the command and data server threads to initialize their respective sockets
/// and serve connections.
void serve_connections()
{
    data_client_slot_indexes.fill(-1);
    data_client_addr_lens.fill(sizeof(sockaddr));
    k_sem_give(&allow_serve_connections_sem);
    k_sem_give(&allow_serve_connections_sem);
}
