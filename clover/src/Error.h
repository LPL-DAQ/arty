#ifndef APP_ERROR_H
#define APP_ERROR_H

#include "MaxLengthString.h"
#include "clover.pb.h"
#include <string_view>
#include <cstdio>
#include <cstring>
#include <expected>

struct device;
constexpr int MAX_ERR_MESSAGE_SIZE = sizeof(static_cast<Response*>(nullptr)->err) - 1;

class Error {
private:
    Error() = default; // The internal constructor is private

    // Helpers to manage the static buffer and locks without exposing Zephyr headers
    static void lock_scheduler();
    static char* get_buffer();

public:
    // C++ Variadic Template replacing the old C-style va_list
    template<typename... Args>
    static Error from_cause(const char* format, Args... args) {
        lock_scheduler(); // Locks once
        // Add "%s" as the format string literal
        snprintf(get_buffer(), MAX_ERR_MESSAGE_SIZE, "%s", format, args...);
        return Error();
    }

    static Error from_cause(std::string_view cause);
    static Error from_code(int code);
    static Error from_device_not_ready(const struct device* dev);

    // C++ Variadic Template for context-adding
    template<typename... Args>
    Error& context(const char* format, Args... args) {
        char* buf = get_buffer();
        size_t len = strlen(buf);
        if (len < MAX_ERR_MESSAGE_SIZE - 1) {
            // Append the context safely to the end of the currently locked buffer
            snprintf(buf + len, MAX_ERR_MESSAGE_SIZE - len, format, args...);
        }
        return *this;
    }

    MaxLengthString<MAX_ERR_MESSAGE_SIZE> build_message() const;
};

#endif // APP_ERROR_H
