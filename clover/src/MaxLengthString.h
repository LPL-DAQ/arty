#ifndef APP_MAXLENGTHSTRING_H
#define APP_MAXLENGTHSTRING_H
#include <array>
#include <string_view>

template <int max_len>
    requires(max_len > 0)
class MaxLengthString {
private:
    int len;
    std::array<char, max_len+1> buf;

public:
    constexpr MaxLengthString();
    MaxLengthString(std::string_view str);

    MaxLengthString operator+(const MaxLengthString& rhs);
    MaxLengthString& operator+=(const MaxLengthString& rhs);

    [[nodiscard]] int size() const;
    [[deprecated("Prefer size() for consistency with other STL-like containers")]] [[nodiscard]] int length() const;

    [[nodiscard]] const char* c_str() const;
};

template <int max_len>
    requires(max_len > 0)
constexpr MaxLengthString<max_len>::MaxLengthString() : len(0), buf {'\0'} // buf is a zero-length cstring.
{

}

/// Truncates str if it's longer than max_len.
template <int max_len>
    requires(max_len > 0)
MaxLengthString<max_len>::MaxLengthString(std::string_view str)
{
    if (str.size() > max_len) {
        len = max_len;
        buf = str.substr(0, max_len).data();
    }
    else {
        len = static_cast<int>(str.size());
        buf = str.data();
    }
    buf[len] = '\0';
}

template <int max_len>
    requires(max_len > 0)
MaxLengthString<max_len> MaxLengthString<max_len>::operator+(const MaxLengthString& rhs)
{
    MaxLengthString ret {*this};
    ret += rhs;
    return ret;
}

template <int max_len>
    requires(max_len > 0)
MaxLengthString<max_len>& MaxLengthString<max_len>::operator+=(const MaxLengthString& rhs)
{
    int chars_of_rhs = std::min(rhs.size(), max_len - len);
    std::copy(buf.begin() + len, rhs.buf.begin(), rhs.buf.begin() + chars_of_rhs);
    len += chars_of_rhs;
    return *this;
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

template <int max_len>
    requires(max_len > 0)
const char* MaxLengthString<max_len>::c_str() const
{
    return buf.data();
}

#endif  // APP_MAXLENGTHSTRING_H
