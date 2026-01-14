#include "Error.h"

#include <array>
#include <zephyr/kernel.h>

constexpr int MAX_CONTEXTS = 8;
constexpr int MAX_CONTENT_LENGTH = 128;

static int used_contexts = 0;
static bool elided_contexts = false;
static std::array<std::array<char, MAX_CONTENT_LENGTH>, MAX_CONTEXTS> contexts;
static std::array<int, MAX_CONTEXTS> context_lengths;

/// String used as placeholder for context messages over the length cap. E.g. " [...] " in "my message [...] too long".
constexpr std::string_view LONG_CONTEXT_ELISION_TEXT {" [...] "};
constexpr std::string_view TOO_MANY_CONTEXT_ELISION_TEXT {" [omitted context] "};

/// Trims an arbitrary-length input context into a context message + length that's at most MAX_CONTENT_LENGTH chars.
static std::pair<std::array<char, MAX_CONTENT_LENGTH>, int> trim_context(std::string_view raw)
{
    std::array<char, MAX_CONTENT_LENGTH> trimmed;

    // Raw input is unchanged if it's under our length cap.
    if (raw.size() <= MAX_CONTENT_LENGTH) {
        raw.copy(trimmed.data(), raw.size());
        return std::make_pair(trimmed, raw.size());
    }

    constexpr int MAX_ORIGINAL_CHARS = MAX_CONTENT_LENGTH - LONG_CONTEXT_ELISION_TEXT.size();
    constexpr int LEFT_CHARS = MAX_ORIGINAL_CHARS / 2;
    constexpr int RIGHT_CHARS = MAX_ORIGINAL_CHARS - LEFT_CHARS;
    static_assert(LEFT_CHARS > 0, "Unable to place meaningful text on LHS");
    static_assert(RIGHT_CHARS > 0, "Unable to place meaningful text on RHS");

    // Populate trimmed message with its left half taken from start of raw message, right half from end of raw message,
    // and elision text in between to show there may be missing text.
    static_assert(MAX_CONTENT_LENGTH > LEFT_CHARS + RIGHT_CHARS); // raw.size() > MAX_CONTENT_LENGTH, so we will not overrun the input buffer.
    static_assert(LEFT_CHARS + LONG_CONTEXT_ELISION_TEXT.size() + RIGHT_CHARS == MAX_CONTENT_LENGTH);
    raw.copy(trimmed.data(), LEFT_CHARS);
    LONG_CONTEXT_ELISION_TEXT.copy(trimmed.data() + LEFT_CHARS, LONG_CONTEXT_ELISION_TEXT.size());
    raw.copy(trimmed.data() + LEFT_CHARS + LONG_CONTEXT_ELISION_TEXT.size(), RIGHT_CHARS, raw.size() - RIGHT_CHARS);

    return std::make_pair(trimmed, trimmed.size());
}

/// Instantiate a new Error with a root cause. This locks the scheduler till the Error is built into a message.
Error Error::error(std::string_view cause)
{
    k_sched_lock();

    auto [trimmed, trimmed_length] = trim_context(cause);

    static_assert(MAX_CONTEXTS > 0);
    contexts[0] = trimmed;
    context_lengths[0] = trimmed_length;
    used_contexts = 1;
    elided_contexts = false;

    return Error {};
}

/// Attach context to an error while propagating it upward.
void Error::context(std::string_view ctx)
{
    // If necessary, shuffle off an intermediate context to make room for this new context, keeping only highest and
    // lowest messages.
    if (used_contexts >= MAX_CONTEXTS) {
        constexpr int NUM_DEEP_CONTEXTS = MAX_CONTEXTS / 2; // This many deep contexts will be protected from erasure.

        for (int i = NUM_DEEP_CONTEXTS; i < MAX_CONTEXTS - 1; i++) {
            contexts[i] = contexts[i + 1];
            context_lengths[i] = context_lengths[i + 1];
        }
        used_contexts = MAX_CONTEXTS - 1;
        elided_contexts = true;
    }

    auto [trimmed, trimmed_length] = trim_context(ctx);

    contexts[used_contexts] = trimmed;
    context_lengths[used_contexts] = trimmed_length;
    used_contexts++;
}

/// Consume an Error by converting it into a message. This is the only way to destroy an Error; it may not go out of
/// scope implicitly.
std::string_view Error::build_message(Error error, )
{


    k_sched_unlock();
}
