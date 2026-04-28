#pragma once
#include <zephyr/kernel.h>

#define __ARG_PLACEHOLDER
#define __ARG_PLACEHOLDER_1 0,
#define __take_second_arg(__ignored, val, ...) val

/*
 * Getting something that works in C and CPP for an arg that may or may
 * not be defined is tricky.  Here, if we have "#define CONFIG_BOOGER 1"
 * we match on the placeholder define, insert the "0," for arg1 and generate
 * the triplet (0, 1, 0).  Then the last step cherry picks the 2nd arg (a one).
 * When CONFIG_BOOGER is not defined, we generate a (... 1, 0) pair, and when
 * the last step cherry picks the 2nd arg, we get a zero.
 */
#define __is_defined(x) ___is_defined(x)
#define ___is_defined(val) ____is_defined(__ARG_PLACEHOLDER_##val)
#define ____is_defined(arg1_or_junk) __take_second_arg(arg1_or_junk 1, 0)

/// Helper macro that returnes a std::unexpected Error if a Kconfig flag isn't set.
#define ENSURE_CONFIG(config_name) \
    if constexpr (!__is_defined(config_name)) { \
        return std::unexpected(Error::from_cause("invalid config -- " #config_name " must be set")); \
    }

float nsec_since_cycle(uint64_t start_cycle);
