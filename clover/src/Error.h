#ifndef APP_ERROR_H
#define APP_ERROR_H

#include "MaxLengthString.h"
#include "clover.pb.h"
#include <string_view>

struct device;
constexpr int MAX_ERR_MESSAGE_SIZE = sizeof(static_cast<Response*>(nullptr)->err) - 1;
constexpr int MAX_FORMATTED_CONTEXT_LENGTH = 75 + 64; // Exposed for formatting

class Error {
private:
    Error() = default; // The internal constructor is private

    // Backend implementation hidden in Error.cpp to keep Zephyr headers out
    Error& add_context_backend(std::string_view formatted_context);

public:
    // Core non-templated base implemented in Error.cpp
    static Error from_cause(std::string_view cause);

    // Template formats the string locally, then passes the view to the backend
    template<typename... Args>
    static Error from_cause(std::string_view format, Args... args) {
        MaxLengthString<MAX_FORMATTED_CONTEXT_LENGTH> formatted{format, args...};
        return from_cause(formatted.string_view());
    }

    static Error from_code(int code);
    static Error from_device_not_ready(const struct device* dev);

    // Template formats the string locally, then passes the view to the backend
    template<typename... Args>
    Error& context(std::string_view format, Args... args) {
        MaxLengthString<MAX_FORMATTED_CONTEXT_LENGTH> formatted{format, args...};
        return add_context_backend(formatted.string_view());
    }

    MaxLengthString<MAX_ERR_MESSAGE_SIZE> build_message();
};

#endif // APP_ERROR_H
