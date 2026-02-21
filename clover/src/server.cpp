#include <array>
#include <cctype>
#include <climits>
#include <cstdint>
#include <cstring>
#include <pb_decode.h>
#include <pb_encode.h>
#include <sstream>
#include <string>
#include <zephyr/logging/log.h>
#include <zephyr/net/net_pkt.h>
#include <zephyr/net/socket.h>
#include <zephyr/posix/arpa/inet.h>

#include "ThrottleValve.h"
#include "clover.pb.h"
#include "pts.h"
#include "sequencer.h"
#include "server.h"
#include "Controller.h"
#include "Error.h"

LOG_MODULE_REGISTER(Server, CONFIG_LOG_DEFAULT_LEVEL);

// --- Internal Helper Classes ---

class SocketGuard {
public:
    explicit SocketGuard(int fd) : socket(fd) {}
    ~SocketGuard() {
        if (socket >= 0) {
            zsock_close(socket);
        }
    }
    SocketGuard(const SocketGuard&) = delete;
    SocketGuard& operator=(const SocketGuard&) = delete;

    int socket;
};

// --- Global State & Configuration ---

constexpr size_t MAX_MESSAGE_SIZE = 1024 * 8;
static_assert(Request_size <= MAX_MESSAGE_SIZE);
static_assert(Response_size <= MAX_MESSAGE_SIZE);

#define MAX_OPEN_CLIENTS 3
K_SEM_DEFINE(num_open_connections, MAX_OPEN_CLIENTS, MAX_OPEN_CLIENTS);

bool has_thread[MAX_OPEN_CLIENTS] = {false};
K_MUTEX_DEFINE(has_thread_lock);

static k_thread client_threads[MAX_OPEN_CLIENTS];
#define CONNECTION_THREAD_STACK_SIZE (6 * 1024)
K_THREAD_STACK_ARRAY_DEFINE(client_stacks, MAX_OPEN_CLIENTS, CONNECTION_THREAD_STACK_SIZE);

static std::array<ClientType, MAX_OPEN_CLIENTS> client_types;
static std::array<int, MAX_OPEN_CLIENTS> client_sockets;
static std::array<int64_t, MAX_OPEN_CLIENTS> last_pinged;
K_MUTEX_DEFINE(thread_info_guard);

constexpr int MAX_DATA_CLIENTS = 1;
static std::array<sockaddr, MAX_DATA_CLIENTS> data_client_addrs;
static std::array<socklen_t, MAX_DATA_CLIENTS> data_client_addr_lens;
static std::array<int, MAX_DATA_CLIENTS> data_client_slot_indexes;
K_MUTEX_DEFINE(data_client_info_guard);

K_SEM_DEFINE(allow_serve_connections_sem, 0, 2);

// --- Networking Helper Functions ---

int send_fully(int sock, const char* buf, int len) {
    int bytes_sent = 0;
    while (bytes_sent < len) {
        int ret = zsock_send(sock, buf + bytes_sent, len - bytes_sent, 0);
        if (ret < 0) return ret;
        bytes_sent += ret;
    }
    return 0;
}

bool pb_socket_write_callback(pb_ostream_t* stream, const uint8_t* buf, size_t count) {
    int sock = (int)(intptr_t)stream->state;
    return send_fully(sock, reinterpret_cast<const char*>(buf), count) == 0;
}

pb_ostream_t pb_ostream_from_socket(int sock) {
    return { .callback = pb_socket_write_callback, .state = reinterpret_cast<void*>((intptr_t)sock), .max_size = SIZE_MAX, .bytes_written = 0 };
}

bool pb_socket_read_callback(pb_istream_t* stream, uint8_t* buf, size_t count) {
    int sock = (int)(intptr_t)stream->state;
    int bytes_read = zsock_recv(sock, buf, count, ZSOCK_MSG_WAITALL);
    if (bytes_read <= 0) { stream->bytes_left = 0; return false; }
    return true;
}

pb_istream_t pb_istream_from_socket(int sock) {
    return { .callback = pb_socket_read_callback, .state = reinterpret_cast<void*>((intptr_t)sock), .bytes_left = SIZE_MAX };
}

// --- Request Handlers ---

static std::expected<void, Error> handle_subscribe_data_stream(const SubscribeDataStreamRequest&, const int thread_index, const int socket) {
    k_mutex_lock(&data_client_info_guard, K_FOREVER);
    for (int i = 0; i < MAX_DATA_CLIENTS; ++i) {
        if (data_client_slot_indexes[i] == -1) {
            data_client_slot_indexes[i] = thread_index;
            zsock_getpeername(socket, &data_client_addrs[i], &data_client_addr_lens[i]);
            reinterpret_cast<sockaddr_in*>(&data_client_addrs[i])->sin_port = htons(19691);
            k_mutex_unlock(&data_client_info_guard);
            return {};
        }
    }
    k_mutex_unlock(&data_client_info_guard);
    return std::unexpected(Error::from_cause("No data slots available"));
}

static std::expected<void, Error> handle_identify_client(const IdentifyClientRequest& req, const int thread_index, const int socket) {
    k_mutex_lock(&thread_info_guard, K_FOREVER);
    client_types[thread_index] = req.client;
    client_sockets[thread_index] = socket;
    last_pinged[thread_index] = k_uptime_get();
    k_mutex_unlock(&thread_info_guard);
    return {};
}

// --- Main Client Thread ---

static void handle_client(void* p1_thread_index, void* p2_client_socket, void*) {
    int thread_index = (int)(intptr_t)p1_thread_index;
    SocketGuard client_guard{(int)(intptr_t)p2_client_socket};

    pb_istream_t pb_input = pb_istream_from_socket(client_guard.socket);
    pb_ostream_t pb_output = pb_ostream_from_socket(client_guard.socket);

    while (true) {
        Request request = Request_init_default;
        if (!pb_decode_ex(&pb_input, Request_fields, &request, PB_DECODE_DELIMITED)) break;

        std::expected<void, Error> result;

        switch (request.which_payload) {
            case Request_subscribe_data_stream_tag:
                result = handle_subscribe_data_stream(request.payload.subscribe_data_stream, thread_index, client_guard.socket);
                break;
            case Request_identify_client_tag:
                result = handle_identify_client(request.payload.identify_client, thread_index, client_guard.socket);
                break;
            case Request_load_motor_sequence_tag:
                result = Controller::get().handle_load_motor_sequence(request.payload.load_motor_sequence);
                break;
            case Request_start_sequence_tag:
                result = Controller::get().handle_start_sequence(request.payload.start_sequence);
                break;
            case Request_halt_sequence_tag:
                result = Controller::get().handle_halt_sequence(request.payload.halt_sequence);
                break;
            case Request_reset_valve_position_tag:
                // Calls the helper in throttlevalve.cpp
                result = handle_reset_valve_position(request.payload.reset_valve_position);
                break;
            default:
                result = std::unexpected(Error::from_cause("Invalid request tag"));
                break;
        }

        Response response = Response_init_default;
        if (!result) {
            response.has_err = true;
            std::strncpy(response.err, result.error().build_message().c_str(), sizeof(response.err) - 1);
        }

        if (!pb_encode_ex(&pb_output, Response_fields, &response, PB_ENCODE_DELIMITED)) {
            LOG_ERR("Response encoding failed");
            break;
        }
    }
}

// --- Background Services ---

[[noreturn]] static void reap_dead_connections(void*, void*, void*) {
    while (true) {
        k_mutex_lock(&has_thread_lock, K_FOREVER);
        for (int i = 0; i < MAX_OPEN_CLIENTS; ++i) {
            if (has_thread[i] && k_thread_join(&client_threads[i], K_NO_WAIT) == 0) {
                k_mutex_lock(&data_client_info_guard, K_FOREVER);
                for (int j = 0; j < MAX_DATA_CLIENTS; ++j) {
                    if (data_client_slot_indexes[j] == i) data_client_slot_indexes[j] = -1;
                }
                k_mutex_unlock(&data_client_info_guard);
                has_thread[i] = false;
                k_sem_give(&num_open_connections);
            }
        }
        k_mutex_unlock(&has_thread_lock);
        k_sleep(K_MSEC(100));
    }
}
K_THREAD_DEFINE(server_reaper, 1024, reap_dead_connections, nullptr, nullptr, nullptr, 1, 0, 0);

void serve_command_connections() {
    k_sem_take(&allow_serve_connections_sem, K_FOREVER);
    int server_socket = zsock_socket(AF_INET, SOCK_STREAM, IPPROTO_TCP);
    sockaddr_in bind_addr = { .sin_family = AF_INET, .sin_port = htons(19690), .sin_addr = {htonl(INADDR_ANY)} };
    zsock_bind(server_socket, reinterpret_cast<sockaddr*>(&bind_addr), sizeof(bind_addr));
    zsock_listen(server_socket, 5);

    while (true) {
        k_sem_take(&num_open_connections, K_FOREVER);
        int slot = -1;
        k_mutex_lock(&has_thread_lock, K_FOREVER);
        for (int i = 0; i < MAX_OPEN_CLIENTS; ++i) if (!has_thread[i]) { slot = i; break; }
        k_mutex_unlock(&has_thread_lock);

        int client_socket = zsock_accept(server_socket, nullptr, nullptr);
        k_thread_create(&client_threads[slot], reinterpret_cast<k_thread_stack_t*>(&client_stacks[slot]),
                        CONNECTION_THREAD_STACK_SIZE, handle_client, (void*)(intptr_t)slot,
                        (void*)(intptr_t)client_socket, nullptr, 5, 0, K_NO_WAIT);
        k_mutex_lock(&has_thread_lock, K_FOREVER);
        has_thread[slot] = true;
        k_mutex_unlock(&has_thread_lock);
    }
}
K_THREAD_DEFINE(command_server, 4096, serve_command_connections, nullptr, nullptr, nullptr, 2, 0, 0);

void serve_data_connections() {
    k_sem_take(&allow_serve_connections_sem, K_FOREVER);
    int server_socket = zsock_socket(AF_INET, SOCK_DGRAM, IPPROTO_UDP);

    while (true) {
        k_mutex_lock(&data_client_info_guard, K_FOREVER);
        for (int i = 0; i < MAX_DATA_CLIENTS; ++i) {
            if (data_client_slot_indexes[i] == -1) continue;

            DataPacket packet = DataPacket_init_default;
            packet.time = static_cast<float>(k_uptime_get()) / 1000.0f;

            pt_readings pts = pts_sample();
            packet.sensors.has_pt102 = true; packet.sensors.pt102 = pts.pt102;
            packet.sensors.has_pt103 = true; packet.sensors.pt103 = pts.pt103;

            // Using Static Calls directly on the Valve Types
            packet.fuel_valve.target_pos_deg = FuelValve::get_pos_encoder();
            packet.lox_valve.target_pos_deg = LoxValve::get_pos_encoder();

            uint8_t buf[DataPacket_size];
            pb_ostream_t os = pb_ostream_from_buffer(buf, sizeof(buf));
            if (pb_encode(&os, DataPacket_fields, &packet)) {
                zsock_sendto(server_socket, buf, os.bytes_written, 0, &data_client_addrs[i], data_client_addr_lens[i]);
            }
        }
        k_mutex_unlock(&data_client_info_guard);
        k_sleep(K_MSEC(10));
    }
}
K_THREAD_DEFINE(data_server, 4096, serve_data_connections, nullptr, nullptr, nullptr, 2, 0, 0);

void serve_connections() {
    data_client_slot_indexes.fill(-1);
    data_client_addr_lens.fill(sizeof(sockaddr));
    k_sem_give(&allow_serve_connections_sem);
    k_sem_give(&allow_serve_connections_sem);
}
