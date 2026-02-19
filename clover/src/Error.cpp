#include "Error.h"
#include <zephyr/kernel.h>
#include <cstdio>

// Static buffer used for error formatting
static char static_error_buffer[MAX_ERR_MESSAGE_SIZE];

void Error::format_to_static_buffer(std::string_view format, va_list args) {
    k_sched_lock(); // Lock to protect the static buffer during write
    vsnprintf(static_error_buffer, MAX_ERR_MESSAGE_SIZE, format.data(), args);
}

Error Error::from_cause(std::string_view cause) {
    static Error instance;
    // Base implementation remains here
    return instance;
}

MaxLengthString<MAX_ERR_MESSAGE_SIZE> Error::build_message() {
    MaxLengthString<MAX_ERR_MESSAGE_SIZE> msg(static_error_buffer);
    k_sched_unlock(); // Unlock only after the message is safely copied
    return msg;
}
