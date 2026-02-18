#ifndef APP_ERROR_H
#define APP_ERROR_H

#include "MaxLengthString.h"
#include "clover.pb.h"
#include <string_view>
#include <cstdarg>

struct device;

constexpr int MAX_ERR_MESSAGE_SIZE = sizeof(static_cast<Response*>(nullptr)->err)-1;

class Error {
private:
    Error() = default;
    static void format_to_static_buffer(std::string_view format, va_list args);

public:
    static Error from_cause(std::string_view cause);
    static Error from_code(int code);
    static Error from_device_not_ready(const struct device* dev);

    // Templates moved to header to fix undefined references in other files
    template<typename... Args>
    static Error from_cause(std::string_view format, Args... args) {
        va_list vl; va_start(vl, format);
        format_to_static_buffer(format, vl);
        va_end(vl);
        return from_cause(format);
    }

    template<typename... Args>
    Error& context(std::string_view format, Args... args) {
        va_list vl; va_start(vl, format);
        format_to_static_buffer(format, vl);
        va_end(vl);
        return *this;
    }

    MaxLengthString<MAX_ERR_MESSAGE_SIZE> build_message();
};
#endif
