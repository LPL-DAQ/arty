#ifndef APP_MAXLENGTHSTRING_H
#define APP_MAXLENGTHSTRING_H

#include <array>
#include <string_view>
#include <cstdarg>
#include <zephyr/sys/cbprintf.h>
#include <algorithm>

template <int max_len>
    requires(max_len > 0)
class MaxLengthString {
private:
    int len;
    std::array<char, max_len + 1> buf;

public:
    constexpr MaxLengthString();

    template<typename... Args>
    MaxLengthString(std::string_view format, Args... args);

    template <int max_rhs_len>
    void append(const MaxLengthString<max_rhs_len>& rhs);

    void append(const std::string_view& rhs);

    [[nodiscard]] int size() const;
    [[nodiscard]] const char* c_str() const;
    [[nodiscard]] std::string_view string_view() const;
    int copy_buf(char* dest, int max_bytes) const;
};

template <int max_len>
    requires(max_len > 0)
constexpr MaxLengthString<max_len>::MaxLengthString() : len(0), buf{'\0'}
{
}

template <int max_len>
    requires(max_len > 0)
template <typename... Args>
MaxLengthString<max_len>::MaxLengthString(std::string_view format, Args... args)
{
    int ideal_chars_written;
    // Fix: Explicitly handle the case where no extra arguments are provided
    // to satisfy -Wformat-security.
    if constexpr (sizeof...(Args) == 0) {
        ideal_chars_written = snprintfcb(buf.data(), max_len + 1, "%s", format.data());
    } else {
        ideal_chars_written = snprintfcb(buf.data(), max_len + 1, format.data(), args...);
    }

    len = std::min(ideal_chars_written, max_len);
    buf[len] = '\0';
}

template <int max_len>
    requires(max_len > 0)
template <int max_rhs_len>
void MaxLengthString<max_len>::append(const MaxLengthString<max_rhs_len>& rhs)
{
    append(rhs.string_view());
}

template <int max_len>
    requires(max_len > 0)
void MaxLengthString<max_len>::append(const std::string_view& rhs)
{
    const int chars_of_rhs = std::min(static_cast<int>(rhs.size()), max_len - len);
    std::copy(rhs.begin(), rhs.begin() + chars_of_rhs, buf.begin() + len);
    len += chars_of_rhs;
    buf[len] = '\0';
}

template <int max_len>
    requires(max_len > 0)
int MaxLengthString<max_len>::size() const
{
    return len;
}

template <int max_len>
    requires(max_len > 0)
const char* MaxLengthString<max_len>::c_str() const
{
    return buf.data();
}

template <int max_len>
    requires(max_len > 0)
std::string_view MaxLengthString<max_len>::string_view() const
{
    return std::string_view{buf.data(), static_cast<std::size_t>(len)};
}

template <int max_len>
    requires(max_len > 0)
int MaxLengthString<max_len>::copy_buf(char* dest, int max_bytes) const
{
    int bytes_to_copy = std::min(max_bytes, len);
    std::copy(buf.begin(), buf.begin() + bytes_to_copy, dest);
    return bytes_to_copy;
}

#endif  // APP_MAXLENGTHSTRING_H
