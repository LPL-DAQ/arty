#pragma once

#include <stdarg.h>
#include <stdio.h>
#include <zephyr/kernel.h>
#include <zephyr/logging/log.h>
#include <zephyr/shell/shell.h>
#include <zephyr/usb/usb_device.h>

LOG_MODULE_REGISTER(valve_test, LOG_LEVEL_INF);

/* Keep recent events so connecting after boot does not lose the diagnosis. */
#define TRACE_COUNT 128
static char trace_lines[TRACE_COUNT][160];
static unsigned int trace_total;
K_MUTEX_DEFINE(trace_lock);

static void trace_event(const char *format, ...)
{
    char message[128];
    va_list args;
    va_start(args, format);
    vsnprintf(message, sizeof(message), format, args);
    va_end(args);
    k_mutex_lock(&trace_lock, K_FOREVER);
    snprintf(trace_lines[trace_total % TRACE_COUNT], sizeof(trace_lines[0]),
             "[%lld ms] %s", (long long)k_uptime_get(), message);
    ++trace_total;
    k_mutex_unlock(&trace_lock);
    LOG_INF("%s", message);
}

static int cmd_trace(const struct shell *sh, size_t argc, char **argv)
{
    ARG_UNUSED(argc);
    ARG_UNUSED(argv);
    k_mutex_lock(&trace_lock, K_FOREVER);
    unsigned int start = trace_total > TRACE_COUNT ? trace_total - TRACE_COUNT : 0;
    shell_print(sh, "Retained events %u..%u (latest %u maximum)", start, trace_total, TRACE_COUNT);
    for (unsigned int i = start; i < trace_total; ++i) {
        shell_print(sh, "%s", trace_lines[i % TRACE_COUNT]);
    }
    k_mutex_unlock(&trace_lock);
    return 0;
}
SHELL_CMD_REGISTER(trace, NULL, "Show retained valve execution trace", cmd_trace);

static void trace_start(const char *mode)
{
    int err = usb_enable(NULL);
    trace_event("Mode=%s; USB enable result=%d", mode, err);
}
