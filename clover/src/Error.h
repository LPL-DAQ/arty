#ifndef APP_ERROR_H
#define APP_ERROR_H
#include <string_view>

/// Error propagation class. Used with std::expected to attach context for fallible operations. Such errors are not
/// intended to be inspected or handled -- rather, it should just be propagated to the top-level caller, which can then
/// convert it into a string error message to be logged or transmitted in response to a command. Thus, there will only
/// ever be at most one Error in existence, so we can statically pre-allocate all context messages.
///
/// Our slim memory footprint is achieved by locking the scheduler from when the Error is built to when it its error
/// message is assembled. Thus, every Error *must* be resolved into a message, lest the scheduler stay locked forever.
/// This is enforced via a private destructor. Also, this is not ISR safe.
///
/// This is analogous to Rust's anyhow crate, but adapted for memory-constrained, single-CPU embedded environments.
class Error {
private:
    Error() = default;
    ~Error() = default;

public:
    static Error error(std::string_view cause);
    void context(std::string_view ctx);
    static std::string_view build_message(Error error);
};

#endif  // APP_ERROR_H
