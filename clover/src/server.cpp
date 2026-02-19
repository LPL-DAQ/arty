#include "server.h"
#include "clover.pb.h"
#include "Controller.h"
#include "sequencer.h"
#include "guards/SocketGuard.h"
#include <pb_decode.h>
#include <pb_encode.h>
#include <zephyr/net/socket.h>
#include <zephyr/logging/log.h>

LOG_MODULE_REGISTER(server_api);

extern Controller controller;
extern "C" { extern struct k_msgq control_data_msgq; }

#define MAX_OPEN_CLIENTS 3
K_SEM_DEFINE(num_open_connections, MAX_OPEN_CLIENTS, MAX_OPEN_CLIENTS);
K_SEM_DEFINE(allow_serve_connections_sem, 0, 1);
K_MUTEX_DEFINE(has_thread_lock);

static bool has_thread[MAX_OPEN_CLIENTS] = {false};
static k_thread client_threads[MAX_OPEN_CLIENTS];
K_THREAD_STACK_ARRAY_DEFINE(client_stacks, MAX_OPEN_CLIENTS, 4096);
static std::array<ClientType, MAX_OPEN_CLIENTS> client_types;

bool pb_socket_write_callback(pb_ostream_t* stream, const uint8_t* buf, size_t count) {
    int sock = static_cast<int>(reinterpret_cast<intptr_t>(stream->state));
    return zsock_send(sock, buf, count, 0) == static_cast<int>(count);
}

bool pb_socket_read_callback(pb_istream_t* stream, uint8_t* buf, size_t count) {
    int sock = static_cast<int>(reinterpret_cast<intptr_t>(stream->state));
    int ret = zsock_recv(sock, buf, count, ZSOCK_MSG_WAITALL);
    if (ret <= 0) { stream->bytes_left = 0; return false; }
    return true;
}

static void handle_client(void* p1, void* p2, void*) {
    int thread_idx = static_cast<int>(reinterpret_cast<intptr_t>(p1));
    int sock_raw = static_cast<int>(reinterpret_cast<intptr_t>(p2));
    SocketGuard client_guard{sock_raw};
    
    pb_istream_t pb_in = { .callback = pb_socket_read_callback, .state = reinterpret_cast<void*>(static_cast<intptr_t>(client_guard.socket)), .bytes_left = SIZE_MAX };
    pb_ostream_t pb_out = { .callback = pb_socket_write_callback, .state = reinterpret_cast<void*>(static_cast<intptr_t>(client_guard.socket)), .max_size = SIZE_MAX };

    while (true) {
        Request req = Request_init_default;
        if (!pb_decode_ex(&pb_in, Request_fields, &req, PB_DECODE_DELIMITED)) break;

        std::expected<void, Error> res;
        switch (req.which_payload) {
            case Request_load_motor_sequence_tag: res = controller.handle_load_motor_sequence(req.payload.load_motor_sequence); break;
            case Request_start_sequence_tag: res = controller.handle_start_sequence(req.payload.start_sequence); break;
            case Request_halt_sequence_tag: res = controller.handle_halt_sequence(req.payload.halt_sequence); break;
            case Request_identify_client_tag: client_types[thread_idx] = req.payload.identify_client.client; res = {}; break;
            default: res = std::unexpected(Error::from_cause("Unknown tag")); break;
        }

        Response resp = Response_init_default;
        if (!res) { 
            resp.has_err = true; 
            strncpy(resp.err, res.error().build_message().c_str(), sizeof(resp.err) - 1); 
        }
        pb_encode_ex(&pb_out, Response_fields, &resp, PB_ENCODE_DELIMITED);
    }
    k_mutex_lock(&has_thread_lock, K_FOREVER);
    has_thread[thread_idx] = false;
    k_mutex_unlock(&has_thread_lock);
    k_sem_give(&num_open_connections);
    client_types[thread_idx] = ClientType_UNKNOWN_CLIENT;
}

[[noreturn]] void serve_command_connections(void*, void*, void*) {
    k_sem_take(&allow_serve_connections_sem, K_FOREVER);
    int sock = zsock_socket(AF_INET, SOCK_STREAM, IPPROTO_TCP);
    struct sockaddr_in addr = { .sin_family = AF_INET, .sin_port = htons(19690), .sin_addr = { .s_addr = htonl(INADDR_ANY) } };
    zsock_bind(sock, (struct sockaddr*)&addr, sizeof(addr));
    zsock_listen(sock, MAX_OPEN_CLIENTS);

    while (true) {
        k_sem_take(&num_open_connections, K_FOREVER);
        int idx = -1;
        k_mutex_lock(&has_thread_lock, K_FOREVER);
        for (int i = 0; i < MAX_OPEN_CLIENTS; i++) { if (!has_thread[i]) { idx = i; has_thread[idx] = true; break; } }
        k_mutex_unlock(&has_thread_lock);

        int client_sock = zsock_accept(sock, nullptr, nullptr);
        if (client_sock >= 0) {
            k_thread_create(&client_threads[idx], client_stacks[idx], 4096, handle_client, 
                            reinterpret_cast<void*>(static_cast<intptr_t>(idx)), 
                            reinterpret_cast<void*>(static_cast<intptr_t>(client_sock)), 
                            nullptr, 5, 0, K_NO_WAIT);
        } else {
            k_mutex_lock(&has_thread_lock, K_FOREVER);
            has_thread[idx] = false;
            k_mutex_unlock(&has_thread_lock);
            k_sem_give(&num_open_connections);
        }
    }
}
K_THREAD_DEFINE(command_server, 4096, serve_command_connections, nullptr, nullptr, nullptr, 2, 0, 0);

// Restored missing entry point
void serve_connections() {
    k_sem_give(&allow_serve_connections_sem);
}