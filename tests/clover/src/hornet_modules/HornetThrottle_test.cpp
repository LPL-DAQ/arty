#include "../../../../clover/src/hornet/HornetThrottle.h"
#include <zephyr/ztest.h>

// Throttle mapping:
//   thrust < 10 lbf : throttle = thrust / 15.7166, clamped [0, 0.63]
//   thrust >= 10 lbf: throttle = (thrust + 7.3123) / 27.321, clamped [0.63, 1.0]
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

    // 5 lbf < 10 lbf => lower branch: throttle = 5/15.7166 ~= 0.318
    // pulse = 1000 + 0.318 * 1000 ~= 1318
    auto result = HornetThrottle::tick(5.0f);

    zassert_true(result.has_value(), "tick should succeed for 5 lbf");
    auto [pulse_us, metrics] = *result;
    zassert_within(pulse_us, 1318.0f, 2.0f, "5 lbf should produce ~1318 µs pulse");
}

ZTEST(HornetThrottle_tests, test_operating_range_thrust_uses_upper_branch)
{
    HornetThrottle::reset();

    // 15 lbf > 10 lbf => upper branch: throttle = (15 + 7.3123) / 27.321 ~= 0.8167
    // pulse = 1000 + 0.8167 * 1000 ~= 1817
    auto result = HornetThrottle::tick(15.0f);

    zassert_true(result.has_value(), "tick should succeed for 15 lbf");
    auto [pulse_us, metrics] = *result;
    zassert_within(pulse_us, 1817.0f, 2.0f, "15 lbf should produce ~1817 µs pulse");
}

ZTEST(HornetThrottle_tests, test_full_throttle_gives_maximum_pulse)
{
    HornetThrottle::reset();

    // (lbf + 7.3123) / 27.321 = 1.0 => lbf = 20 lbf
    auto result = HornetThrottle::tick(20.009f);

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
