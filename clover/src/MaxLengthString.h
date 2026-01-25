#ifndef APP_MAXLENGTHSTRING_H
#define APP_MAXLENGTHSTRING_H
#include <array>
#include <string_view>
#include <cstdarg>
#include <zephyr/sys/cbprintf.h>

template <int max_len>
    requires(max_len > 0)
class MaxLengthString {
private:
    int len;
    std::array<char, max_len + 1> buf;

public:
    constexpr MaxLengthString();
    MaxLengthString(std::string_view format, ...);
    MaxLengthString(std::string_view format, std::va_list args);

    template <int max_rhs_len> void append(const MaxLengthString<max_rhs_len>& rhs);
    void append(const std::string_view& rhs);

    [[nodiscard]] int size() const;
    [[deprecated("Prefer size() for consistency with other STL-like containers")]] [[nodiscard]] int length() const;

    [[nodiscard]] const char* c_str() const;
    [[nodiscard]] std::string_view string_view() const;
    int copy_buf(char* dest, int max_bytes) const;
};

template <int max_len>
    requires(max_len > 0)
constexpr MaxLengthString<max_len>::MaxLengthString() : len(0), buf{'\0'}  // buf is a zero-length cstring.
{
}

template <int max_len>
    requires(max_len > 0)
MaxLengthString<max_len>::MaxLengthString(std::string_view format, ...)
{
    std::va_list args;
    va_start(args, format);
    const int ideal_chars_written = vsnprintfcb(buf.data(), max_len+1, format.data(), args);
    va_end(args);
    len = std::min(ideal_chars_written, max_len);
    buf[len] = '\0';
}

template <int max_len>
    requires(max_len > 0)
MaxLengthString<max_len>::MaxLengthString(std::string_view format, std::va_list args)
{
    const int ideal_chars_written = vsnprintfcb(buf.data(), max_len+1, format.data(), args);
    len = std::min(ideal_chars_written, max_len);
    buf[len] = '\0';
}

/// Append another MaxLengthString to this string, truncating if rhs is longer than this string has unused bytes.
template <int max_len>
    requires(max_len > 0)
template <int max_rhs_len>
void MaxLengthString<max_len>::append(const MaxLengthString<max_rhs_len>& rhs)
{
    append(rhs.string_view());
}

/// Append a string_view to this string, truncating if rhs is longer than this string has unused bytes.
template <int max_len>
    requires(max_len > 0)
void MaxLengthString<max_len>::append(const std::string_view& rhs)
{
    const int chars_of_rhs = std::min(std::ssize(rhs), max_len - len);
    rhs.copy(buf.begin() + len, chars_of_rhs);
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
int MaxLengthString<max_len>::length() const
{
    return len;
}

/// Returns a cstring corresponding to the max length string, using its internal buffer. When the MaxLengthString goes
/// out of scope, the cstring pointer is no longer valid.
template <int max_len>
    requires(max_len > 0)
const char* MaxLengthString<max_len>::c_str() const
{
    return buf.data();
}

/// Returns a std::string_view corresponding to the max length string, using its internal buffer. When the
/// MaxLengthString goes out of scope, the std::string_view is no longer valid.
template <int max_len>
    requires(max_len > 0)
std::string_view MaxLengthString<max_len>::string_view() const
{
    return std::string_view{buf.data(), static_cast<std::size_t>(len)};
}

/// Copies the contents of the string into another buffer, not including the trailing null byte. Copies up to max_bytes.
/// Returns the number of bytes actually copied.
template <int max_len>
    requires(max_len > 0)
int MaxLengthString<max_len>::copy_buf(char* dest, int max_bytes) const
{
    int bytes_to_copy = std::min(max_bytes, len);
    std::copy(buf.begin(), buf.begin() + bytes_to_copy, dest);
    return bytes_to_copy;
}

#endif  // APP_MAXLENGTHSTRING_H
