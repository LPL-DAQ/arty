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

#include "Controller.h"
#include "MutexGuard.h"
#include "clover.pb.h"
#include "server.h"

#ifdef CONFIG_HORNET
#include "hornet/PwmActuator.h"
#elif CONFIG_RANGER
#include "ranger/ThrottleValve.h"
#include "ranger/Valves.h"

#else
#error Either CONFIG_HORNET or CONFIG_RANGER must be set.
#endif

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
K_MUTEX_DEFINE(thread_info_lock);

/// Tracks when DAQ last pinged (AKA, ID'd itself)
static int daq_thread_index = -1;
static int64_t daq_last_pinged_ms = 0;
K_MUTEX_DEFINE(daq_status_lock);

constexpr int MAX_DATA_CLIENTS = 1;
static std::array<sockaddr, MAX_DATA_CLIENTS> data_client_addrs;
static std::array<socklen_t, MAX_DATA_CLIENTS> data_client_addr_lens;
static std::array<int, MAX_DATA_CLIENTS> data_client_slot_indexes;
K_MUTEX_DEFINE(data_client_info_lock);

/// Synchronizes command/data server threads to ensure they only start accepting connections after startup is fully
/// done (i.e., when serve_connections() is called).
K_SEM_DEFINE(allow_serve_connections_sem, 0, 2);

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
            "zsock_recv only partially filled its buffer from sock %d, this should be impossible as we pass ZSOCK_MSG_WAITALL - got: %d, requested: %d - errno "
            "%d",
            sock,
            bytes_read,
            count,
            errno);
        return false;
    }

    return true;
}

/// Create a nanopb input stream from socket fd.
pb_istream_t pb_istream_from_socket(int sock)
{
    return pb_istream_t{.callback = pb_socket_read_callback, .state = reinterpret_cast<void*>(sock), .bytes_left = SIZE_MAX};
}

static std::expected<void, Error> handle_identify_client(const IdentifyClientRequest& req, int thread_index)
{
    switch (req.client) {
    case ClientType_DAQ: {
        MutexGuard daq_status_guard{&daq_status_lock};
        daq_thread_index = thread_index;
        daq_last_pinged_ms = k_uptime_get();
        return {};
    }

    case ClientType_GNC:
        return {};

    default:
        return std::unexpected(Error::from_cause("Unknown client: %d", req.client));
    }
}

daq_client_status get_daq_client_status()
{
    MutexGuard daq_status_guard{&daq_status_lock};
    return daq_client_status{.connected = daq_thread_index != -1, .last_pinged_ms = daq_thread_index != -1 ? k_uptime_get() - daq_last_pinged_ms : 0};
}

/// Handles a client connection. Should run in its own thread.
static void handle_client(void* p1_thread_index, void* p2_client_socket, void*)
{
    int thread_index = reinterpret_cast<int>(p1_thread_index);
    int client_socket = reinterpret_cast<int>(p2_client_socket);
    LOG_INF("Handling socket: %d", client_socket);

    pb_istream_t pb_input = pb_istream_from_socket(client_socket);
    pb_ostream_t pb_output = pb_ostream_from_socket(client_socket);

    while (true) {
        Request request = Request_init_default;
        bool valid = pb_decode_ex(&pb_input, Request_fields, &request, PB_DECODE_DELIMITED);
        if (!valid) {
            LOG_ERR("Failed to decode next message: errno=%d pb_err='%s'", errno, PB_GET_ERROR(&pb_input));
            break;
        }

        // ADDED: We use cmd_result to capture any domain logic errors returned by the Controller.
        // If a command (like loading a trace) fails, we bubble this error directly to the response.
        std::expected<void, Error> cmd_result = {};

        // Handle request
        switch (request.which_payload) {
        case Request_subscribe_data_stream_tag: {
            LOG_INF("Subscribe data stream");
            MutexGuard daq_status_guard{&data_client_info_lock};

            bool found_data_client_slot = false;
            for (int i = 0; i < MAX_DATA_CLIENTS; ++i) {
                if (data_client_slot_indexes[i] != -1) {
                    continue;
                }
                found_data_client_slot = true;
                data_client_slot_indexes[i] = thread_index;

                int err = getpeername(client_socket, &data_client_addrs[i], &data_client_addr_lens[i]);
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

            break;
        }
        case Request_identify_client_tag: {
            LOG_INF("Identify client");
            cmd_result = handle_identify_client(request.payload.identify_client, thread_index);
            break;
        }

        // Provided by AnalogSensors
        case Request_configure_analog_sensors_tag: {
            LOG_INF("Configure analog sensors");
            cmd_result = AnalogSensors::handle_configure_analog_sensors(request.payload.configure_analog_sensors);
            break;
        }

        // Provided by Valves
        case Request_configure_valves_request_tag: {
            LOG_INF("configure_valves_request");
#ifdef CONFIG_RANGER
            cmd_result = Valves::handle_configure_valves_request(request.payload.configure_valves_request);
#else
            cmd_result = std::unexpected(Error::from_cause("configure_valves_request is unsupported; CONFIG_RANGER must be set"));
#endif
            break;
        }

        case Request_actuate_valve_request_tag: {
            LOG_INF("actuate_valve_request");
#ifdef CONFIG_RANGER
            cmd_result = Valves::handle_actuate_valve_request(request.payload.actuate_valve_request);
#else
            cmd_result = std::unexpected(Error::from_cause("actuate_valve_request is unsupported; CONFIG_RANGER must be set"));
#endif
            break;
        }

        case Request_abort_tag: {
            LOG_INF("abort command");
            cmd_result = Controller::handle_abort(request.payload.abort);
            break;
        }

        case Request_halt_tag: {
            LOG_INF("halt command");
            cmd_result = Controller::handle_halt(request.payload.halt);
            break;
        }

        case Request_unprime_tag: {
            LOG_INF("unprime command");
            cmd_result = Controller::handle_unprime(request.payload.unprime);
            break;
        }

        case Request_calibrate_throttle_valve_tag: {
            LOG_INF("calibrate_throttle_valve command");
            cmd_result = Controller::handle_calibrate_throttle_valve(request.payload.calibrate_throttle_valve);
            break;
        }

        case Request_load_throttle_valve_sequence_tag: {
            LOG_INF("load_throttle_valve_sequence command");
            cmd_result = Controller::handle_load_throttle_valve_sequence(request.payload.load_throttle_valve_sequence);
            break;
        }

        case Request_start_throttle_valve_sequence_tag: {
            LOG_INF("start_throttle_valve_sequence command");
            cmd_result = Controller::handle_start_throttle_valve_sequence(request.payload.start_throttle_valve_sequence);
            break;
        }

        case Request_load_throttle_sequence_tag: {
            LOG_INF("load_throttle_sequence command");
            cmd_result = Controller::handle_load_throttle_sequence(request.payload.load_throttle_sequence);
            break;
        }

        case Request_start_throttle_sequence_tag: {
            LOG_INF("start_throttle_sequence command");
            cmd_result = Controller::handle_start_throttle_sequence(request.payload.start_throttle_sequence);
            break;
        }

        case Request_calibrate_tvc_tag: {
            LOG_INF("calibrate_tvc command");
            cmd_result = Controller::handle_calibrate_tvc(request.payload.calibrate_tvc);
            break;
        }

        case Request_load_tvc_sequence_tag: {
            LOG_INF("load_tvc_sequence command");
            cmd_result = Controller::handle_load_tvc_sequence(request.payload.load_tvc_sequence);
            break;
        }

        case Request_start_tvc_sequence_tag: {
            LOG_INF("start_tvc_sequence command");
            cmd_result = Controller::handle_start_tvc_sequence(request.payload.start_tvc_sequence);
            break;
        }

        case Request_load_rcs_valve_sequence_tag: {
            LOG_INF("load_rcs_valve_sequence command");
            cmd_result = Controller::handle_load_rcs_valve_sequence(request.payload.load_rcs_valve_sequence);
            break;
        }

        case Request_start_rcs_valve_sequence_tag: {
            LOG_INF("start_rcs_valve_sequence command");
            cmd_result = Controller::handle_start_rcs_valve_sequence(request.payload.start_rcs_valve_sequence);
            break;
        }

        case Request_load_rcs_sequence_tag: {
            LOG_INF("load_rcs_sequence command");
            cmd_result = Controller::handle_load_rcs_sequence(request.payload.load_rcs_sequence);
            break;
        }

        case Request_start_rcs_sequence_tag: {
            LOG_INF("start_rcs_sequence command");
            cmd_result = Controller::handle_start_rcs_sequence(request.payload.start_rcs_sequence);
            break;
        }

        case Request_load_flight_sequence_tag: {
            LOG_INF("load_flight_sequence command");
            cmd_result = Controller::handle_load_flight_sequence(request.payload.load_flight_sequence);
            break;
        }

        case Request_start_flight_sequence_tag: {
            LOG_INF("start_flight_sequence command");
            cmd_result = Controller::handle_start_flight_sequence(request.payload.start_flight_sequence);
            break;
        }

        default: {
            LOG_ERR(
                "Request has invalid tag, this should be impossible as pb_decode should have produced a valid Request - got tag: %u", request.which_payload);
            break;
        }
        }

        Response response = Response_init_default;

        // Populate error message in response if required.
        if (!cmd_result.has_value()) {
            response.has_err = true;
            MaxLengthString<MAX_ERR_MESSAGE_SIZE> err_msg = cmd_result.error().build_message();

            err_msg.copy_buf(response.err, sizeof(response.err));

            LOG_ERR("Command failed with error: %s", response.err);
        }
        else {
            LOG_INF("Command OK");
        }

        // Send message over TCP with varint length prefix.
        bool ok = pb_encode_ex(&pb_output, Response_fields, &response, PB_ENCODE_DELIMITED);
        if (!ok) {
            LOG_ERR("Failed to encode command response: %s", pb_output.errmsg);
        }
    }
    zsock_close(client_socket);
}

/// Attempts to join connection handler threads, allowing the thread slots to be reused to service new connection.
[[noreturn]] static void reap_dead_connections(void*, void*, void*)
{
    bool freed_threads[MAX_OPEN_CLIENTS] = {false};
    while (true) {
        {
            MutexGuard has_thread_guard{&has_thread_lock};
            for (int i = 0; i < MAX_OPEN_CLIENTS; ++i) {
                if (has_thread[i]) {
                    int ret = k_thread_join(&client_threads[i], K_NO_WAIT);
                    if (ret == -EBUSY) {
                        // Thread still running
                        continue;
                    }
                    else if (ret != 0) {
                        LOG_ERR("Unexpected code from joining client thread: err %d", ret);
                        continue;
                    }

                    {
                        MutexGuard thread_info_guard{&thread_info_lock};
                        thread_names[i].clear();
                    }

                    // Clean up potential data client subscription
                    {
                        MutexGuard data_client_info_guard{&data_client_info_lock};
                        for (int j = 0; j < MAX_DATA_CLIENTS; ++j) {
                            if (data_client_slot_indexes[j] == i) {
                                data_client_slot_indexes[j] = -1;
                                data_client_addr_lens[j] = sizeof(sockaddr);
                            }
                        }
                    }

                    // Clean up potential DAQ connection
                    {
                        MutexGuard daq_status_guard{&daq_status_lock};
                        if (daq_thread_index == i) {
                            daq_thread_index = -1;
                            daq_last_pinged_ms = k_uptime_get();
                        }
                    }

                    has_thread[i] = false;
                    freed_threads[i] = true;
                    k_sem_give(&num_open_connections);
                }
            }
        }

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
        int connection_index;
        {
            MutexGuard has_thread_guard{&has_thread_lock};
            for (connection_index = 0; connection_index < MAX_OPEN_CLIENTS; ++connection_index) {
                if (!has_thread[connection_index]) {
                    break;
                }
            }
        }
        if (connection_index == MAX_OPEN_CLIENTS) {
            LOG_ERR("Consistency error: Server acquired connection semaphore but no thread slots were open");
            return;
        }

        // Spawn thread to service client connection
        int client_socket = zsock_accept(server_socket, nullptr, nullptr);
        if (client_socket < 0) {
            LOG_ERR("zsock_accept failed: errno=%d — releasing slot and retrying", errno);
            k_sem_give(&num_open_connections);
            continue;
        }
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

        {
            MutexGuard has_thread_guard{&has_thread_lock};
            has_thread[connection_index] = true;
        }
    }
}

K_THREAD_DEFINE(command_server, 8192, serve_command_connections, nullptr, nullptr, nullptr, 2, 0, 0);

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
        DataPacket data_packet = Controller::get_next_data_packet();

        // Encode data packet exactly ONCE per tick, regardless of how many UDP clients are subscribed
        uint8_t buf[DataPacket_size];
        pb_ostream_t data_packet_ostream = pb_ostream_from_buffer(buf, DataPacket_size);
        bool ok = pb_encode(&data_packet_ostream, DataPacket_fields, &data_packet);
        if (!ok) {
            LOG_ERR("Failed to encode data packet: %s", data_packet_ostream.errmsg);
            continue;
        }

        // Lock the client info guard and broadcast the already-encoded buffer to all subscribed IPs
        MutexGuard data_client_info_guard{&data_client_info_lock};
        for (int i = 0; i < MAX_DATA_CLIENTS; ++i) {
            if (data_client_slot_indexes[i] == -1) {
                continue;
            }

            const int bytes_sent = zsock_sendto(server_socket, buf, data_packet_ostream.bytes_written, 0, &data_client_addrs[i], data_client_addr_lens[i]);

            // ADDED: static_cast to size_t to safely compare signed zsock_sendto return with unsigned bytes_written
            if (static_cast<size_t>(bytes_sent) != data_packet_ostream.bytes_written) {
                LOG_ERR("sendto failed: bytes_sent=%d errno=%d", bytes_sent, errno);
            }
        }
    }
}

K_THREAD_DEFINE(data_server, 8192, serve_data_connections, nullptr, nullptr, nullptr, 2, 0, 0);

/// Called at the end of startup, allowing the command and data server threads to initialize their respective sockets
/// and serve connections.
void serve_connections()
{
    data_client_slot_indexes.fill(-1);
    data_client_addr_lens.fill(sizeof(sockaddr));
    k_sem_give(&allow_serve_connections_sem);
    k_sem_give(&allow_serve_connections_sem);
}
