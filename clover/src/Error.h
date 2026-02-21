#ifndef APP_ERROR_H
#define APP_ERROR_H

#include "MaxLengthString.h"
#include "clover.pb.h"
#include <string_view>
#include <cstdarg>
#include <cstdio>
#include <expected>

struct device;
constexpr int MAX_ERR_MESSAGE_SIZE = sizeof(static_cast<Response*>(nullptr)->err) - 1;

class Error {
private:
    // The internal constructor is private to force use of static factory methods
    Error() = default;

    // The "Bridge": This C-style variadic function handles the actual formatting
    // safely for the compiler.
    static void format_buffer(const char* format, ...);

public:
    static Error from_cause(const char* format, ...);
    static Error from_cause(std::string_view cause);
    static Error from_code(int code);
    static Error from_device_not_ready(const struct device* dev);

    // C++ Variadic Template for context-adding
    template<typename... Args>
    Error& context(const char* format, Args... args) {
        format_buffer(format, args...);
        return *this;
    }

    MaxLengthString<MAX_ERR_MESSAGE_SIZE> build_message() const;
};

#endif // APP_ERROR_H
