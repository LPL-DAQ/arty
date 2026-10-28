/* Host-only adapters for exercising firmware code with real TCP and mock GPIO. */
#pragma once
#define _POSIX_C_SOURCE 200809L
#include <assert.h>
#include <errno.h>
#include <pthread.h>
#include <stdbool.h>
#include <stdint.h>
#include <stddef.h>
#include <stdio.h>
#include <stdarg.h>
#include <stdlib.h>
#include <string.h>
#include <time.h>
#include <sys/socket.h>
#include <sys/time.h>
#include <arpa/inet.h>
#include <poll.h>
#include <unistd.h>
#define ARRAY_SIZE(a) (sizeof(a) / sizeof((a)[0]))
#define BUILD_ASSERT(c, m) _Static_assert(c, m)
#define BIT(n) (1U << (n))
#define ARG_UNUSED(a) (void)(a)
#define K_FOREVER (-1)
#define K_MSEC(ms) (ms)
#define K_MUTEX_DEFINE(n) pthread_mutex_t n = PTHREAD_MUTEX_INITIALIZER
#define k_mutex_lock(m, t) pthread_mutex_lock(m)
#define k_mutex_unlock(m) pthread_mutex_unlock(m)
struct host_sem { pthread_mutex_t lock; pthread_cond_t cond; int count; };
#define K_SEM_DEFINE(n, a, b) struct host_sem n = {PTHREAD_MUTEX_INITIALIZER, PTHREAD_COND_INITIALIZER, a}
static inline void k_sem_give(struct host_sem *s) {
    pthread_mutex_lock(&s->lock); s->count = 1; pthread_cond_signal(&s->cond); pthread_mutex_unlock(&s->lock);
}
static inline void k_sem_reset(struct host_sem *s) {
    pthread_mutex_lock(&s->lock); s->count = 0; pthread_mutex_unlock(&s->lock);
}
static inline int k_sem_take(struct host_sem *s, int ms) {
    pthread_mutex_lock(&s->lock);
    int result = 0;
    struct timespec until; clock_gettime(CLOCK_REALTIME, &until);
    if (ms >= 0) {
        until.tv_sec += ms / 1000; until.tv_nsec += (ms % 1000) * 1000000L;
        if (until.tv_nsec >= 1000000000L) { until.tv_sec++; until.tv_nsec -= 1000000000L; }
    }
    while (!s->count && !result) {
        result = ms < 0 ? pthread_cond_wait(&s->cond, &s->lock) : pthread_cond_timedwait(&s->cond, &s->lock, &until);
    }
    if (!result) s->count--;
    pthread_mutex_unlock(&s->lock);
    return result ? -EAGAIN : 0;
}
static inline int64_t k_uptime_get(void) {
    struct timespec t; clock_gettime(CLOCK_MONOTONIC, &t);
    return (int64_t)t.tv_sec * 1000 + t.tv_nsec / 1000000;
}
static inline void host_trace(const char *format, ...) {
    va_list args; va_start(args, format); vfprintf(stdout, format, args); va_end(args);
    fputc('\n', stdout); fflush(stdout);
}
#define trace_event host_trace
#define trace_start host_trace
#define LOG_MODULE_REGISTER(...)
#define LOG_INF host_trace
#define LOG_ERR host_trace
#define LOG_LEVEL_INF 3
#define K_THREAD_DEFINE(...)
#define SHELL_CMD_ARG(...) 0
#define SHELL_SUBCMD_SET_END 0
#define SHELL_STATIC_SUBCMD_SET_CREATE(n, ...) static int n[] = {__VA_ARGS__}
#define SHELL_CMD_REGISTER(...)
struct shell { int unused; };
#define shell_print(sh, ...) host_trace(__VA_ARGS__)
#define shell_error(sh, ...) host_trace(__VA_ARGS__)
struct device { const char *name; };
static struct device host_gpio = {"mock"};
struct gpio_dt_spec { struct device *port; unsigned pin; unsigned dt_flags; };
#define DT_FOREACH_PROP_ELEM_SEP(...) {&host_gpio,0,0},{&host_gpio,1,0},{&host_gpio,2,0}
#define GPIO_ACTIVE_LOW 1
#define GPIO_OUTPUT_INACTIVE 0
static int outputs[3];
static bool inject_fault;
static unsigned host_port;
static inline bool gpio_is_ready_dt(const struct gpio_dt_spec *p) { (void)p; return true; }
static inline int gpio_pin_configure_dt(const struct gpio_dt_spec *p, int flags) {
    assert(flags == GPIO_OUTPUT_INACTIVE); outputs[p->pin] = 0; return 0;
}
static inline int gpio_pin_set_dt(const struct gpio_dt_spec *p, int value) {
    if (inject_fault && value && p->pin == 1) return -EIO;
    outputs[p->pin] = value; return 0;
}
static inline int gpio_pin_get_raw(const struct device *p, unsigned pin) { (void)p; return outputs[pin]; }
#define zsock_pollfd pollfd
#define zsock_socket socket
#define zsock_bind bind
#define zsock_listen listen
#define zsock_accept accept
#define zsock_poll poll
#define zsock_setsockopt setsockopt
#define zsock_recv recv
#define zsock_send send
#define zsock_close close
#define ZSOCK_POLLIN POLLIN
#define ZSOCK_POLLERR POLLERR
#define ZSOCK_POLLHUP POLLHUP
#define ZSOCK_POLLNVAL POLLNVAL
