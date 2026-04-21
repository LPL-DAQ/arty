#include "../../../../clover/src/hornet/HornetThrottle.h"
#include <zephyr/ztest.h>

// Throttle mapping:
//   thrust < 44.044 N : throttle = thrust / 69.911, clamped [0, 0.63]
//   thrust >= 44.044 N: throttle = (thrust + 32.5271) / 121.53, clamped [0.63, 1.0]
// Pulse width: 0.0 -> 1000 µs, 1.0 -> 2000 µs

ZTEST(HornetThrottle_tests, test_zero_thrust_gives_minimum_pulse)
{
    HornetThrottle::reset();

    auto result = HornetThrottle::tick(0.0f);

    zassert_true(result.has_value(), "tick should succeed for zero thrust");
    auto [pulse_us, metrics] = *result;
    zassert_within(pulse_us, 1000.0f, 1.0f, "zero thrust should produce ~1000 µs pulse");
}

ZTEST(HornetThrottle_tests, test_low_thrust_below_10lbf_uses_lower_branch)
{
    HornetThrottle::reset();

    // 20 N < 44.044 N => lower branch: throttle = 20/69.911 ~= 0.286
    // pulse = 1000 + 0.286 * 1000 ~= 1286
    auto result = HornetThrottle::tick(20.0f);

    zassert_true(result.has_value(), "tick should succeed for 20 N");
    auto [pulse_us, metrics] = *result;
    zassert_within(pulse_us, 1286.0f, 2.0f, "20 N should produce ~1286 µs pulse");
}

ZTEST(HornetThrottle_tests, test_operating_range_thrust_uses_upper_branch)
{
    HornetThrottle::reset();

    // 80 N > 44.044 N => upper branch: throttle = (80 + 32.5271) / 121.53 ~= 0.926
    // pulse = 1000 + 0.926 * 1000 ~= 1926
    auto result = HornetThrottle::tick(80.0f);

    zassert_true(result.has_value(), "tick should succeed for 80 N");
    auto [pulse_us, metrics] = *result;
    zassert_within(pulse_us, 1926.0f, 2.0f, "80 N should produce ~1926 µs pulse");
}

ZTEST(HornetThrottle_tests, test_full_throttle_gives_maximum_pulse)
{
    HornetThrottle::reset();

    // (N + 32.5271) / 121.53 = 1.0 => N = 89.0 N
    auto result = HornetThrottle::tick(89.0f);

    zassert_true(result.has_value(), "tick should succeed at full throttle");
    auto [pulse_us, metrics] = *result;
    zassert_within(pulse_us, 2000.0f, 1.0f, "full throttle should produce ~2000 µs pulse");
}

ZTEST(HornetThrottle_tests, test_excessive_thrust_clamped_to_maximum_pulse)
{
    HornetThrottle::reset();

    auto result = HornetThrottle::tick(200.0f);

    zassert_true(result.has_value(), "tick should succeed for over-commanded thrust");
    auto [pulse_us, metrics] = *result;
    zassert_within(pulse_us, 2000.0f, 1.0f, "over-commanded thrust should clamp to 2000 µs");
}

ZTEST(HornetThrottle_tests, test_pulse_width_always_within_valid_range)
{
    float thrusts[] = {-100.0f, -10.0f, 0.0f, 10.0f, 30.0f, 44.044f, 60.0f, 80.0f, 89.0f, 150.0f, 500.0f};
    for (float thrust : thrusts) {
        HornetThrottle::reset();
        auto result = HornetThrottle::tick(thrust);
        zassert_true(result.has_value(), "tick should succeed");
        auto [pulse_us, metrics] = *result;
        zassert_true(pulse_us >= 1000.0f, "pulse should be >= 1000 µs");
        zassert_true(pulse_us <= 2000.0f, "pulse should be <= 2000 µs");
    }
}

ZTEST(HornetThrottle_tests, test_reset_clears_state)
{
    HornetThrottle::reset();
    // No crash and subsequent tick works normally
    auto result = HornetThrottle::tick(0.0f);
    zassert_true(result.has_value(), "tick after reset should succeed");
}

ZTEST_SUITE(HornetThrottle_tests, NULL, NULL, NULL, NULL, NULL);
