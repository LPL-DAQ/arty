#include <errno.h>
#include <string.h>
#include <pb_decode.h>
#include <pb_encode.h>
#include <zephyr/kernel.h>
#include <zephyr/logging/log.h>
#include <zephyr/net/socket.h>
#include "sequence_protocol.h"

LOG_MODULE_REGISTER(sequence_tcp, LOG_LEVEL_INF);

#define CLIENTS 2
#define FRAME_SIZE 256
#define PARTIAL_TIMEOUT_MS 5000
struct client {
    uint8_t frame[FRAME_SIZE];
    size_t used;
    uint32_t length;
    unsigned prefix_bytes;
    bool reading_body;
    int64_t last_byte;
};

static void serve(void *a, void *b, void *c)
{
    ARG_UNUSED(a); ARG_UNUSED(b); ARG_UNUSED(c);
    static struct client clients[CLIENTS];
    /* Large protobuf unions and wire buffers live off the thread stack. */
    static Request request;
    static Response response;
    static uint8_t reply[3072];
    struct zsock_pollfd fds[CLIENTS + 1];
    for (size_t i = 0; i < ARRAY_SIZE(fds); ++i) {
        fds[i] = (struct zsock_pollfd){.fd = -1, .events = ZSOCK_POLLIN};
    }
    struct sockaddr_in address = {
        .sin_family = AF_INET,
        .sin_port = htons(19690),
        .sin_addr.s_addr = INADDR_ANY,
    };
    int listener = zsock_socket(AF_INET, SOCK_STREAM, IPPROTO_TCP);
    if (listener < 0 || zsock_bind(listener, (struct sockaddr *)&address, sizeof(address)) < 0 ||
        zsock_listen(listener, CLIENTS) < 0) {
        LOG_ERR("Cannot start TCP port 19690: errno=%d", errno);
        if (listener >= 0) zsock_close(listener);
        return;
    }
    fds[0].fd = listener;
    LOG_INF("Sequence TCP server ready on port 19690");
    for (;;) {
        int result = zsock_poll(fds, ARRAY_SIZE(fds), 100);
        if (result < 0) {
            if (errno == EINTR) continue;
            LOG_ERR("TCP poll failed: errno=%d", errno);
            break;
        }
        if (fds[0].revents & ZSOCK_POLLIN) {
            int fd = zsock_accept(listener, NULL, NULL);
            if (fd >= 0) {
                size_t slot;
                for (slot = 1; slot <= CLIENTS; ++slot) {
                    if (fds[slot].fd < 0) {
                        fds[slot].fd = fd;
                        memset(&clients[slot - 1], 0, sizeof(clients[0]));
                        struct timeval timeout = {.tv_sec = 0, .tv_usec = 100000};
                        zsock_setsockopt(fd, SOL_SOCKET, SO_SNDTIMEO, &timeout, sizeof(timeout));
                        break;
                    }
                }
                if (slot > CLIENTS) zsock_close(fd);
            }
        }
        for (size_t i = 1; i <= CLIENTS; ++i) {
            if (fds[i].fd < 0) continue;
            struct client *client = &clients[i - 1];
            bool close_client = fds[i].revents & (ZSOCK_POLLERR | ZSOCK_POLLHUP | ZSOCK_POLLNVAL);
            if ((client->prefix_bytes || client->reading_body) &&
                k_uptime_get() - client->last_byte > PARTIAL_TIMEOUT_MS) close_client = true;
            /* Consume one byte per poll iteration; partial clients cannot block others. */
            if (!close_client && (fds[i].revents & ZSOCK_POLLIN)) {
                uint8_t byte;
                if (zsock_recv(fds[i].fd, &byte, 1, 0) != 1) {
                    close_client = true;
                } else {
                    client->last_byte = k_uptime_get();
                    if (!client->reading_body) {
                        if (client->prefix_bytes == 4 ||
                            (client->prefix_bytes == 3 && (byte & 0xf0))) {
                            close_client = true;
                        } else {
                            client->length |= (uint32_t)(byte & 0x7f) << (7 * client->prefix_bytes++);
                            if (!(byte & 0x80)) {
                                client->reading_body = true;
                                if (!client->length || client->length > FRAME_SIZE) close_client = true;
                            }
                        }
                    } else {
                        client->frame[client->used++] = byte;
                        if (client->used == client->length) {
                            request = (Request)Request_init_zero;
                            pb_istream_t input = pb_istream_from_buffer(client->frame, client->length);
                            if (!pb_decode(&input, Request_fields, &request)) {
                                close_client = true;
                            } else {
                                sequence_handle_request(&request, &response);
                                pb_ostream_t output = pb_ostream_from_buffer(reply, sizeof(reply));
                                if (!pb_encode_delimited(&output, Response_fields, &response)) {
                                    close_client = true;
                                } else {
                                    size_t sent = 0;
                                    while (sent < output.bytes_written) {
                                        int count = zsock_send(fds[i].fd, reply + sent, output.bytes_written - sent, 0);
                                        if (count <= 0) { close_client = true; break; }
                                        sent += count;
                                    }
                                }
                            }
                            memset(client, 0, sizeof(*client));
                        }
                    }
                }
            }
            if (close_client) {
                zsock_close(fds[i].fd);
                fds[i].fd = -1;
                memset(client, 0, sizeof(*client));
            }
        }
    }
    for (size_t i = 0; i < ARRAY_SIZE(fds); ++i) {
        if (fds[i].fd >= 0) zsock_close(fds[i].fd);
    }
}

K_THREAD_DEFINE(sequence_server_thread, 4096, serve, NULL, NULL, NULL, 7, 0, 0);
