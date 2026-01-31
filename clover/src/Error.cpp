#include "Error.h"

#include "MaxLengthString.h"
#include "clover.pb.h"
#include "zephyr/logging/log_ctrl.h"

#include <array>
#include <cmath>
#include <zephyr/kernel.h>
#include <zephyr/logging/log.h>

constexpr int MAX_CONTEXTS = 6;
constexpr int MAX_CONTEXT_LENGTH = 128;

/// Length of buffer for formatted string contents so we can elide the middle instead of truncating.
constexpr int MAX_FORMATTED_CONTEXT_LENGTH = MAX_CONTEXT_LENGTH + 64;

/// Number of deepest-level error contexts that'll be kept even if intermediate contexts are elided.
constexpr int MAX_DEEP_CONTEXTS = MAX_CONTEXTS / 2;

/// String used as placeholder for context messages over the length cap. E.g. " [...] " in "my message [...] too long".
constexpr std::string_view LONG_CONTEXT_ELISION_TEXT{" [...] "};

/// String used as placeholder for elid ed intermediate contexts.
constexpr std::string_view TOO_MANY_CONTEXT_ELISION_TEXT{"[elided context]"};

static int used_contexts = 0;
static bool elided_contexts = false;
static std::array<MaxLengthString<MAX_CONTEXT_LENGTH>, MAX_CONTEXTS> contexts;
K_MUTEX_DEFINE(context_guard);

LOG_MODULE_REGISTER(Error);

/// Trims an arbitrary-length input context into a context message + length that's at most MAX_CONTENT_LENGTH chars.
static MaxLengthString<MAX_CONTEXT_LENGTH> trim_context(const std::string_view& raw)
{
    // Raw input is unchanged if it's under our length cap.
    if (raw.size() <= MAX_CONTEXT_LENGTH) {
        return MaxLengthString<MAX_CONTEXT_LENGTH>{raw};
    }

    constexpr int MAX_ORIGINAL_CHARS = MAX_CONTEXT_LENGTH - LONG_CONTEXT_ELISION_TEXT.size();
    constexpr int LEFT_CHARS = MAX_ORIGINAL_CHARS / 2;
    constexpr int RIGHT_CHARS = MAX_ORIGINAL_CHARS - LEFT_CHARS;
    static_assert(LEFT_CHARS > 0, "Unable to place meaningful text on LHS");
    static_assert(RIGHT_CHARS > 0, "Unable to place meaningful text on RHS");

    // Populate trimmed message with its left half taken from start of raw message, right half from end of raw message,
    // and elision text in between to show there may be missing text.
    static_assert(
        MAX_CONTEXT_LENGTH >
        LEFT_CHARS + RIGHT_CHARS);  // raw.size() > MAX_CONTENT_LENGTH, so we will not overrun the input buffer.
    static_assert(LEFT_CHARS + LONG_CONTEXT_ELISION_TEXT.size() + RIGHT_CHARS == MAX_CONTEXT_LENGTH);

    MaxLengthString<MAX_CONTEXT_LENGTH> trimmed{raw.substr(0, LEFT_CHARS)};
    trimmed.append(raw.substr(raw.size() - RIGHT_CHARS, RIGHT_CHARS));
    trimmed.append(LONG_CONTEXT_ELISION_TEXT);

    return trimmed;
}

Error Error::from_cause(std::string_view cause)
{
    // We are writing to shared buffers which is not thread safe, so we must lock the scheduler while the error is
    // propagated.
    k_sched_lock();

    int ret = k_mutex_lock(&context_guard, K_NO_WAIT);
    if (ret == -EBUSY) {
        LOG_ERR("!!!! UNSOUND CODE: multiple Errors instantiated at once, Errors may only be immediately propogated upward till consumed into a message. !!!!");
        // Rather than panic, we attempt to recover by waiting for the context guard. Either are pretty horrendous
        // options, as we are certainly clobbering another error message. But that's potentially better than causing
        // a hard abort of the whole system. Note that the scheduler lock is maintained even if we yield here.
        k_mutex_lock(&context_guard, K_FOREVER);
    }

    static_assert(MAX_CONTEXTS > 0);
    contexts[0] = trim_context(cause);
    used_contexts = 1;
    elided_contexts = false;

    return Error{};
}

/// Instantiate a new Error with a root cause. This locks the scheduler till the Error is built into a message.
template<typename... Args>
Error Error::from_cause(std::string_view format, Args... args)
{
    // We are writing to shared buffers which is not thread safe, so we must lock the scheduler while the error is
    // propagated.
    k_sched_lock();

    int ret = k_mutex_lock(&context_guard, K_NO_WAIT);
    if (ret == -EBUSY) {
        LOG_ERR("!!!! UNSOUND CODE: multiple Errors instantiated at once, Errors may only be immediately propogated upward till consumed into a message. !!!!");
        // Rather than panic, we attempt to recover by waiting for the context guard. Either are pretty horrendous
        // options, as we are certainly clobbering another error message. But that's potentially better than causing
        // a hard abort of the whole system. Note that the scheduler lock is maintained even if we yield here.
        k_mutex_lock(&context_guard, K_FOREVER);
    }

    const MaxLengthString<MAX_FORMATTED_CONTEXT_LENGTH> cause {format, args...};

    static_assert(MAX_CONTEXTS > 0);
    contexts[0] = trim_context(cause.string_view());
    used_contexts = 1;
    elided_contexts = false;

    return Error{};
}

/// Instantiates an error from an error code, assuming that it uses standard OS error codes.
Error Error::from_code(int code)
{
    const MaxLengthString<MAX_CONTEXT_LENGTH> cause {"%s (err code %d)", strerror(std::abs(code)), code};
    return from_cause(cause.string_view());
}

/// Instantiate an error from a device's init_res field, populated if the device is not ready and failed initialization.
Error Error::from_device_not_ready(const device* dev)
{
    return from_code(dev->state->init_res).context("device `%s` not ready", dev->name);
}

/// Attach context to an error while propagating it upward.
template<typename... Args>
Error& Error::context(std::string_view format, Args... args)
{
    // If necessary, shuffle off an intermediate context to make room for this new context, keeping only highest and
    // lowest messages.
    if (used_contexts >= MAX_CONTEXTS) {
        for (int i = MAX_DEEP_CONTEXTS; i < MAX_CONTEXTS - 1; i++) {
            contexts[i] = contexts[i + 1];
        }
        used_contexts = MAX_CONTEXTS - 1;
        elided_contexts = true;
    }

    const MaxLengthString<MAX_FORMATTED_CONTEXT_LENGTH> context {format, args...};

    contexts[used_contexts] = trim_context(context.string_view());
    used_contexts++;

    return *this;
}

/// Consume an Error by converting it into a message. This is the only way to destroy an Error; it may not go out of
/// scope implicitly.
MaxLengthString<MAX_ERR_MESSAGE_SIZE> Error::build_message()
{
    constexpr std::string_view CONTEXT_SEPERATOR{": "};

    // Max length is achieved with all contexts at max length plus elided contexts.
    constexpr int CALCULATED_MAX_MESSAGE_SIZE = MAX_CONTEXTS * MAX_CONTEXT_LENGTH +
                                                TOO_MANY_CONTEXT_ELISION_TEXT.size() +
                                                CONTEXT_SEPERATOR.size() * MAX_CONTEXTS;
    static_assert(CALCULATED_MAX_MESSAGE_SIZE == MAX_ERR_MESSAGE_SIZE);

    MaxLengthString<MAX_ERR_MESSAGE_SIZE> message;
    for (int i = 0; i < used_contexts; ++i) {
        message.append(contexts[i]);
        if (i != used_contexts - 1) {
            message.append(CONTEXT_SEPERATOR);
        }
        if (elided_contexts && i == MAX_DEEP_CONTEXTS - 1) {
            message.append(TOO_MANY_CONTEXT_ELISION_TEXT);
            message.append(CONTEXT_SEPERATOR);
        }
    }

    // We are done with the shared context buffers, other threads may preempt us now.
    k_sched_unlock();

    return message;
}
