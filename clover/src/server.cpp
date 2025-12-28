#include <cctype>
#include <zephyr/logging/log.h>
#include <zephyr/net/socket.h>
#include <zephyr/sys/errno_private.h>
#include <zephyr/net/net_pkt.h>
#include <sstream>
#include <string>
#include <array>
#include <climits>
#include <cstdint>
#include <pb_encode.h>
#include <pb_decode.h>

#include "clover.pb.h"
#include "throttle_valve.h"
#include "server.h"
#include "guards/SocketGuard.h"
#include "pts.h"
#include "sequencer.h"


LOG_MODULE_REGISTER(Server, CONFIG_LOG_DEFAULT_LEVEL);

constexpr size_t MAX_MESSAGE_SIZE = 512;
static_assert(Request_size <= MAX_MESSAGE_SIZE);
static_assert(Response_size <= MAX_MESSAGE_SIZE);

#define MAX_OPEN_CLIENTS 3

/// Main server thread must acquire one of these before accepting a connection. It must then scan through the thread
/// array to find an open slot.
K_SEM_DEFINE(num_open_connections,
             MAX_OPEN_CLIENTS, MAX_OPEN_CLIENTS);

bool has_thread[MAX_OPEN_CLIENTS] = {false};
K_MUTEX_DEFINE(has_thread_lock);

static k_thread client_threads[MAX_OPEN_CLIENTS] = {nullptr};
#define CONNECTION_THREAD_STACK_SIZE (6 * 1024)
K_THREAD_STACK_ARRAY_DEFINE(client_stacks,
                            MAX_OPEN_CLIENTS, CONNECTION_THREAD_STACK_SIZE);

constexpr int MAX_THREAD_NAME_LENGTH = 10;
static std::array<std::string, MAX_OPEN_CLIENTS> thread_names;
static std::array<int, MAX_OPEN_CLIENTS> thread_sockets;
static std::array<int64_t, MAX_OPEN_CLIENTS> last_pinged;
K_MUTEX_DEFINE(thread_info_guard);

/// Helper that sends a payload completely through an socket
int send_fully(int sock, const char* buf, int len)
{
    int bytes_sent = 0;
    while (bytes_sent < len)
    {
        int ret = zsock_send(sock, buf + bytes_sent,
                             len - bytes_sent, 0);
        if (ret < 0)
        {
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
    if (bytes_sent < 0)
    {
        LOG_ERR("Error while sending data over socket %d: %s", sock, strerror(bytes_sent));
        return false;
    }

    if (bytes_sent != static_cast<int>(count))
    {
        LOG_ERR(
            "zsock_send only partially wrote its buffer from socket %d, this should be impossible - sent: %d, requested: %d",
            sock, bytes_sent, count);
        return false;
    }

    return true;
}

pb_ostream_t pb_ostream_from_socket(int sock)
{
    return pb_ostream_t{
        .callback = pb_socket_write_callback,
        .state = reinterpret_cast<void*>(sock),
        .max_size = SIZE_MAX,
        .bytes_written = 0
    };
}

/// Internal callback for pb_istream, used to read from socket to internal buffer.
bool pb_socket_read_callback(pb_istream_t* stream, uint8_t* buf, size_t count)
{
    int sock = reinterpret_cast<int>(stream->state);

    int bytes_read = zsock_recv(sock, buf, count, ZSOCK_MSG_WAITALL);

    // Hit EOF
    if (bytes_read == 0)
    {
        stream->bytes_left = 0;
        return false;
    }

    if (bytes_read != static_cast<int>(count))
    {
        LOG_ERR(
            "zsock_recv only partially filled its buffer from sock %d, this should be impossible as we pass ZSOCK_MSG_WAITALL - got: %d, requested: %d",
            sock, bytes_read, count);
        return false;
    }

    return true;
}

/// Create a nanopb input stream from socket fd.
pb_istream_t pb_istream_from_socket(int sock)
{
    return pb_istream_t{
        .callback = pb_socket_read_callback,
        .state = reinterpret_cast<void*>(sock),
        .bytes_left = SIZE_MAX
    };
}


/// Handles a client connection. Should run in its own thread.
static void handle_client(void* p1_thread_index, void* p2_client_socket, void*)
{
    int thread_index = reinterpret_cast<int>(p1_thread_index);
    SocketGuard client_guard{reinterpret_cast<int>(p2_client_socket)};
    LOG_INF("Handling socket: %d", client_guard.socket);
    k_sleep(K_MSEC(500));

    pb_istream_t pb_input = pb_istream_from_socket(client_guard.socket);

    while (true)
    {
        Request request = Request_init_default;
        bool valid = pb_decode_ex(&pb_input, Request_fields, &request, PB_DECODE_DELIMITED);
        if (!valid)
        {
            LOG_INF("Failed to decode next message, this can happen if connection is severed.");
            break;
        }

        // Handle request
        switch (request.which_payload)
        {
        case Request_reset_valve_position_tag:
            {
                ResetValvePositionRequest& req = request.payload.reset_valve_position;
                switch (req.valve)
                {
                case Valve_FUEL:
                    LOG_INF("Reset value fuel");
                    break;
                case Valve_LOX:
                    LOG_INF("Reset valve lox");
                    break;
                default:
                    LOG_ERR("Bad value.");
                    break;
                }
                break;
            }
        case Request_read_pts_tag:
            {
                LOG_INF("Read pts");
                break;
            }
        case Request_load_open_loop_motor_sequence_tag:
            {
                LOG_INF("Open loop motor sequence");
                break;
            }

        case Request_start_sequence_tag:
            {
                LOG_INF("Start sequence");
                break;
            }
        case Request_halt_sequence_tag:
            {
                LOG_INF("halt seq");
                break;
            }

        default:
            {
                LOG_ERR(
                    "Request has invalid tag, this should be impossible as pb_decode should have produced a valid request - got tag: %u",
                    request.which_payload);
                break;
            }
        }
    }
}

/// Attempts to join connection handler threads, allowing the thread slots to be reused to service new connection.
[[noreturn]] static void reap_dead_connections(void*, void*, void*)
{
    bool freed_threads[MAX_OPEN_CLIENTS] = {false};
    while (true)
    {
        k_mutex_lock(&has_thread_lock, K_FOREVER);
        for (int i = 0; i < MAX_OPEN_CLIENTS; ++i)
        {
            if (has_thread[i])
            {
                int ret = k_thread_join(&client_threads[i], K_NO_WAIT);
                if (ret == 0)
                {
                    k_mutex_lock(&thread_info_guard, K_FOREVER);
                    thread_names[i].clear();
                    k_mutex_unlock(&thread_info_guard);

                    has_thread[i] = false;
                    freed_threads[i] = true;
                    k_sem_give(&num_open_connections);
                }
                else if (ret == -EBUSY)
                {
                    // Thread still running
                }
                else
                {
                    LOG_ERR("Unexpected code from joining client thread: err %d", ret);
                }
            }
        }
        k_mutex_unlock(&has_thread_lock);

        // Log freed threads outside mutex
        for (int i = 0; i < MAX_OPEN_CLIENTS; ++i)
        {
            if (freed_threads[i])
            {
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
    if (server_socket < 0)
    {
        LOG_ERR("Failed to create TCP socket: %d", errno);
        return;
    }

    sockaddr_in bind_addr = {};
    bind_addr.sin_family = AF_INET;
    bind_addr.sin_addr.s_addr = htonl(INADDR_ANY);
    bind_addr.sin_port = htons(19690);

    LOG_INF("Binding server socket to address");
    int err = zsock_bind(server_socket, reinterpret_cast<sockaddr*>(&bind_addr), sizeof(bind_addr));
    if (err)
    {
        LOG_ERR("Failed to bind to socket `%d`: %d", server_socket, err);
        return;
    }
    LOG_INF("Listening for open connections");
    err = zsock_listen(server_socket, 0);
    if (err)
    {
        LOG_ERR("Failed to listen on socket `%d`: %d", server_socket, err);
        return;
    }

    // Serve new connections indefinitely
    while (true)
    {
        // Wait for free thread slot
        err = k_sem_take(&num_open_connections, K_FOREVER);
        if (err)
        {
            LOG_INF("Failed to acquire semaphore: %d", err);
            return;
        }

        // Find open connection index
        int connection_index = 0;
        k_mutex_lock(&has_thread_lock, K_FOREVER);
        for (connection_index = 0; connection_index < MAX_OPEN_CLIENTS; ++connection_index)
        {
            if (!has_thread[connection_index])
            {
                break;
            }
        }
        k_mutex_unlock(&has_thread_lock);
        if (connection_index == MAX_OPEN_CLIENTS)
        {
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
                        reinterpret_cast<void*>(connection_index),
                        reinterpret_cast<void*>(client_socket), nullptr, 5, 0, K_NO_WAIT
        );

        k_mutex_lock(&has_thread_lock, K_FOREVER);
        has_thread[connection_index] = true;
        k_mutex_unlock(&has_thread_lock);
    }
}
