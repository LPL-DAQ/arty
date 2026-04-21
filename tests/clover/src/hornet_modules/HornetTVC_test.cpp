#include "../../../../clover/src/hornet/HornetTvc.h"
#include <zephyr/ztest.h>
#include <cmath>

// TVC mapping (r_gimble = r_servo = 1.0):
//   servo_angle = asinf(1.0) * sinf(command_deg * pi/180)   [radians]
//               = (pi/2) * sin(command_deg * pi/180)
// Pulse width: -90 deg -> 1000 us, 0 deg -> 1500 us, +90 deg -> 2000 us
//   pitch pulse = 1500 + servo_pitch_angle * 5.55556
//   yaw pulse   = 1500 + servo_yaw_angle  * 5.555556

ZTEST(HornetTvc_tests, test_zero_commands_give_neutral_pulse)
{
    HornetTvc::reset();

    auto result = HornetTvc::tick(0.0f, 0.0f);

    zassert_true(result.has_value(), "tick should succeed for zero commands");
    auto [pitch_pulse, yaw_pulse, metrics] = *result;
    zassert_within(pitch_pulse, 1500.0f, 1.0f, "zero pitch command should give 1500 us");
    zassert_within(yaw_pulse, 1500.0f, 1.0f, "zero yaw command should give 1500 us");
}

ZTEST(HornetTvc_tests, test_positive_pitch_increases_pulse)
{
    HornetTvc::reset();

    auto result = HornetTvc::tick(45.0f, 0.0f);

    zassert_true(result.has_value(), "tick should succeed for positive pitch");
    auto [pitch_pulse, yaw_pulse, metrics] = *result;
    zassert_true(pitch_pulse > 1500.0f, "positive pitch should produce pulse above neutral");
    zassert_within(yaw_pulse, 1500.0f, 1.0f, "yaw should remain neutral with zero yaw command");
}

ZTEST(HornetTvc_tests, test_negative_pitch_decreases_pulse)
{
    HornetTvc::reset();

    auto result = HornetTvc::tick(-45.0f, 0.0f);

    zassert_true(result.has_value(), "tick should succeed for negative pitch");
    auto [pitch_pulse, yaw_pulse, metrics] = *result;
    zassert_true(pitch_pulse < 1500.0f, "negative pitch should produce pulse below neutral");
    zassert_within(yaw_pulse, 1500.0f, 1.0f, "yaw should remain neutral with zero yaw command");
}

ZTEST(HornetTvc_tests, test_positive_yaw_increases_pulse)
{
    HornetTvc::reset();

    auto result = HornetTvc::tick(0.0f, 45.0f);

    zassert_true(result.has_value(), "tick should succeed for positive yaw");
    auto [pitch_pulse, yaw_pulse, metrics] = *result;
    zassert_within(pitch_pulse, 1500.0f, 1.0f, "pitch should remain neutral with zero pitch command");
    zassert_true(yaw_pulse > 1500.0f, "positive yaw should produce pulse above neutral");
}

ZTEST(HornetTvc_tests, test_negative_yaw_decreases_pulse)
{
    HornetTvc::reset();

    auto result = HornetTvc::tick(0.0f, -45.0f);

    zassert_true(result.has_value(), "tick should succeed for negative yaw");
    auto [pitch_pulse, yaw_pulse, metrics] = *result;
    zassert_within(pitch_pulse, 1500.0f, 1.0f, "pitch should remain neutral with zero pitch command");
    zassert_true(yaw_pulse < 1500.0f, "negative yaw should produce pulse below neutral");
}

ZTEST(HornetTvc_tests, test_pitch_and_yaw_are_independent)
{
    HornetTvc::reset();

    // Apply both pitch and yaw simultaneously
    auto result = HornetTvc::tick(30.0f, -30.0f);

    zassert_true(result.has_value(), "tick should succeed for combined pitch+yaw");
    auto [pitch_pulse, yaw_pulse, metrics] = *result;
    zassert_true(pitch_pulse > 1500.0f, "positive pitch should increase pitch pulse");
    zassert_true(yaw_pulse < 1500.0f, "negative yaw should decrease yaw pulse");
}

ZTEST(HornetTvc_tests, test_symmetric_commands_give_symmetric_pulses)
{
    HornetTvc::reset();

    auto pos_result = HornetTvc::tick(45.0f, 0.0f);
    auto neg_result = HornetTvc::tick(-45.0f, 0.0f);

    zassert_true(pos_result.has_value(), "tick should succeed");
    zassert_true(neg_result.has_value(), "tick should succeed");
    auto [pos_pitch, pos_yaw, pos_m] = *pos_result;
    auto [neg_pitch, neg_yaw, neg_m] = *neg_result;

    // Symmetric about neutral (1500 us)
    zassert_within(pos_pitch + neg_pitch, 3000.0f, 2.0f,
                   "symmetric commands should produce symmetric pulses around 1500 us");
}

ZTEST(HornetTvc_tests, test_pulse_width_always_within_valid_range)
{
    float commands[] = {-120.0f, -90.0f, -45.0f, 0.0f, 45.0f, 90.0f, 120.0f};
    for (float cmd : commands) {
        HornetTvc::reset();
        auto result = HornetTvc::tick(cmd, cmd);
        zassert_true(result.has_value(), "tick should succeed");
        auto [pitch_pulse, yaw_pulse, metrics] = *result;
        zassert_true(pitch_pulse >= 1000.0f, "pitch pulse should be >= 1000 us");
        zassert_true(pitch_pulse <= 2000.0f, "pitch pulse should be <= 2000 us");
        zassert_true(yaw_pulse >= 1000.0f, "yaw pulse should be >= 1000 us");
        zassert_true(yaw_pulse <= 2000.0f, "yaw pulse should be <= 2000 us");
    }
}

ZTEST(HornetTvc_tests, test_reset_doesnt_cause_errors_on_next_tick)
{
    HornetTvc::reset();
    auto result = HornetTvc::tick(0.0f, 0.0f);
    zassert_true(result.has_value(), "tick after reset should succeed");
}

ZTEST_SUITE(HornetTvc_tests, NULL, NULL, NULL, NULL, NULL);
