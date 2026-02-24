#include "Error.h"
#include <zephyr/kernel.h>
#include <cstdio>

static char static_error_buffer[MAX_ERR_MESSAGE_SIZE];

// --- New Helpers to support the header templates ---
void Error::lock_scheduler() {
    k_sched_lock();
}

char* Error::get_buffer() {
    return static_error_buffer;
}
// ---------------------------------------------------

Error Error::from_cause(std::string_view cause) {
    if (cause.data() != static_error_buffer) {
        k_sched_lock();
        snprintf(static_error_buffer, MAX_ERR_MESSAGE_SIZE, "%.*s", (int)cause.size(), cause.data());
    }
    return Error();
}

Error Error::from_code(int code) {
    return from_cause("Internal Error Code: %d", code);
}

Error Error::from_device_not_ready(const struct device* dev) {
    return from_cause("Hardware device not ready");
}

MaxLengthString<MAX_ERR_MESSAGE_SIZE> Error::build_message() const {
    MaxLengthString<MAX_ERR_MESSAGE_SIZE> msg(static_error_buffer);
    k_sched_unlock();
    return msg;
}
