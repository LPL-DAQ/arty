#include "Error.h"

#include "MaxLengthString.h"
#include "clover.pb.h"
#include "zephyr/logging/log_ctrl.h"

#include <array>
#include <cmath>
#include <zephyr/kernel.h>
#include <zephyr/logging/log.h>

constexpr int MAX_CONTEXTS = 6;
constexpr int MAX_CONTEXT_LENGTH = 75; // Safely fits inside the 499 limit

constexpr int MAX_DEEP_CONTEXTS = MAX_CONTEXTS / 2;
constexpr std::string_view LONG_CONTEXT_ELISION_TEXT{" [...] "};
constexpr std::string_view TOO_MANY_CONTEXT_ELISION_TEXT{"[elided context]"};

static int used_contexts = 0;
static bool elided_contexts = false;
static std::array<MaxLengthString<MAX_CONTEXT_LENGTH>, MAX_CONTEXTS> contexts;
K_MUTEX_DEFINE(context_guard);

LOG_MODULE_REGISTER(Error);

static MaxLengthString<MAX_CONTEXT_LENGTH> trim_context(const std::string_view& raw)
{
    if (raw.size() <= MAX_CONTEXT_LENGTH) {
        return MaxLengthString<MAX_CONTEXT_LENGTH>{raw};
    }

    constexpr int MAX_ORIGINAL_CHARS = MAX_CONTEXT_LENGTH - LONG_CONTEXT_ELISION_TEXT.size();
    constexpr int LEFT_CHARS = MAX_ORIGINAL_CHARS / 2;
    constexpr int RIGHT_CHARS = MAX_ORIGINAL_CHARS - LEFT_CHARS;
    static_assert(LEFT_CHARS > 0, "Unable to place meaningful text on LHS");
    static_assert(RIGHT_CHARS > 0, "Unable to place meaningful text on RHS");
    static_assert(MAX_CONTEXT_LENGTH > LEFT_CHARS + RIGHT_CHARS);
    static_assert(LEFT_CHARS + LONG_CONTEXT_ELISION_TEXT.size() + RIGHT_CHARS == MAX_CONTEXT_LENGTH);

    MaxLengthString<MAX_CONTEXT_LENGTH> trimmed{raw.substr(0, LEFT_CHARS)};
    trimmed.append(raw.substr(raw.size() - RIGHT_CHARS, RIGHT_CHARS));
    trimmed.append(LONG_CONTEXT_ELISION_TEXT);

    return trimmed;
}

Error Error::from_cause(std::string_view cause)
{
    k_sched_lock();

    int ret = k_mutex_lock(&context_guard, K_NO_WAIT);
    if (ret == -EBUSY) {
        LOG_ERR("!!!! UNSOUND CODE: multiple Errors instantiated at once, Errors may only be immediately propogated upward till consumed into a message. !!!!");
        k_mutex_lock(&context_guard, K_FOREVER);
    }

    static_assert(MAX_CONTEXTS > 0);
    contexts[0] = trim_context(cause);
    used_contexts = 1;
    elided_contexts = false;

    return Error{};
}

Error Error::from_code(int code)
{
    const MaxLengthString<MAX_CONTEXT_LENGTH> cause {"%s (err code %d)", strerror(std::abs(code)), code};
    return from_cause(cause.string_view());
}

Error Error::from_device_not_ready(const device* dev)
{
    return from_code(dev->state->init_res).context("device `%s` not ready", dev->name);
}

// Replaces the old templated context() function
Error& Error::add_context_backend(std::string_view formatted_context)
{
    if (used_contexts >= MAX_CONTEXTS) {
        for (int i = MAX_DEEP_CONTEXTS; i < MAX_CONTEXTS - 1; i++) {
            contexts[i] = contexts[i + 1];
        }
        used_contexts = MAX_CONTEXTS - 1;
        elided_contexts = true;
    }

    contexts[used_contexts] = trim_context(formatted_context);
    used_contexts++;

    return *this;
}

MaxLengthString<MAX_ERR_MESSAGE_SIZE> Error::build_message()
{
    constexpr std::string_view CONTEXT_SEPERATOR{": "};

    constexpr int CALCULATED_MAX_MESSAGE_SIZE = MAX_CONTEXTS * MAX_CONTEXT_LENGTH +
                                                TOO_MANY_CONTEXT_ELISION_TEXT.size() +
                                                CONTEXT_SEPERATOR.size() * MAX_CONTEXTS;
    static_assert(CALCULATED_MAX_MESSAGE_SIZE <= MAX_ERR_MESSAGE_SIZE, "Calculated message size exceeds protobuf buffer limit");

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

    k_sched_unlock();

    return message;
}
