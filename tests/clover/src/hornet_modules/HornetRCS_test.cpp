#include "../../../../clover/src/hornet/HornetRcs.h"
#include <zephyr/ztest.h>

// RCS bang-bang controller with PID and hysteresis.
// control() returns p_valve: -1 (CW), 0 (neutral), or 1 (CCW).
// tick() returns tuple<float, float, HornetRcsMetrics> where the floats
// represent CW and CCW thruster pulse widths in microseconds:
//   1000.0f = off (0% throttle)
//   2000.0f = on  (100% throttle)
//
// Use THRUSTER_OFF / THRUSTER_ON constants for clear assertions.
static constexpr float THRUSTER_OFF = 1000.0f;
static constexpr float THRUSTER_ON  = 2000.0f;

static EstimatedState make_identity_state()
{
    EstimatedState state = EstimatedState_init_default;
    state.R_WB.qw = 1.0f;
    state.R_WB.qx = 0.0f;
    state.R_WB.qy = 0.0f;
    state.R_WB.qz = 0.0f;
    state.position.x = 0.0f;
    state.position.y = 0.0f;
    state.position.z = 0.0f;
    state.velocity.x = 0.0f;
    state.velocity.y = 0.0f;
    state.velocity.z = 0.0f;
    return state;
}

ZTEST(HornetRcs_tests, test_tick_returns_value_for_zero_roll_error)
{
    HornetRcs::reset();

    EstimatedState state = make_identity_state();
    // Identity quaternion -> roll = 0, command = 0 -> no control effort
    auto result = HornetRcs::tick(state, 0.0f);

    zassert_true(result.has_value(), "tick should succeed for zero roll error");
}

ZTEST(HornetRcs_tests, test_tick_returns_value_for_positive_roll_command)
{
    HornetRcs::reset();

    EstimatedState state = make_identity_state();
    // Command 30 deg CCW roll with zero actual roll -> positive error
    auto result = HornetRcs::tick(state, 30.0f);

    zassert_true(result.has_value(), "tick should succeed for positive roll command");
    auto [cw, ccw, metrics] = *result;
    // With roll error > deadzone (0.15), CCW thruster should fire on first tick
    zassert_equal(ccw, THRUSTER_ON,  "CCW thruster should be commanded for positive roll error");
    zassert_equal(cw,  THRUSTER_OFF, "CW thruster should not be commanded for positive roll error");
}

ZTEST(HornetRcs_tests, test_tick_returns_value_for_negative_roll_command)
{
    HornetRcs::reset();

    EstimatedState state = make_identity_state();
    // Command -30 deg CW roll with zero actual roll -> negative error
    auto result = HornetRcs::tick(state, -30.0f);

    zassert_true(result.has_value(), "tick should succeed for negative roll command");
    auto [cw, ccw, metrics] = *result;
    // With roll error < -deadzone (-0.15), CW thruster should fire on first tick
    zassert_equal(cw,  THRUSTER_ON,  "CW thruster should be commanded for negative roll error");
    zassert_equal(ccw, THRUSTER_OFF, "CCW thruster should not be commanded for negative roll error");
}

ZTEST(HornetRcs_tests, test_reset_clears_pid_and_valve_state)
{
    // Fire a command then reset, verify subsequent neutral command produces no thrust
    EstimatedState state = make_identity_state();
    HornetRcs::tick(state, 30.0f);  // activate CCW thruster
    HornetRcs::reset();

    // After reset, no roll error -> valve should be off (p_valve = 0)
    auto result = HornetRcs::tick(state, 0.0f);
    zassert_true(result.has_value(), "tick after reset should succeed");
    auto [cw, ccw, metrics] = *result;
    zassert_equal(cw,  THRUSTER_OFF, "CW should be off after reset with zero error");
    zassert_equal(ccw, THRUSTER_OFF, "CCW should be off after reset with zero error");
}

ZTEST(HornetRcs_tests, test_hysteresis_keeps_valve_on_when_error_reduces_slightly)
{
    HornetRcs::reset();

    EstimatedState state = make_identity_state();

    // Activate the CCW thruster with a large positive error
    HornetRcs::tick(state, 30.0f);

    // Reduce error but stay above (deadzone - hysteresis) = 0.10 deg
    // p_valve should remain 1 (CCW) due to hysteresis
    auto result = HornetRcs::tick(state, 0.12f);  // 0.12 > 0.10 threshold

    zassert_true(result.has_value(), "tick should succeed");
    auto [cw, ccw, metrics] = *result;
    zassert_equal(ccw, THRUSTER_ON,  "CCW thruster should remain on within hysteresis band");
    zassert_equal(cw,  THRUSTER_OFF, "CW thruster should stay off within hysteresis band");
}

ZTEST(HornetRcs_tests, test_valve_turns_off_when_error_below_hysteresis_threshold)
{
    HornetRcs::reset();

    EstimatedState state = make_identity_state();

    // Activate the CCW thruster
    HornetRcs::tick(state, 30.0f);

    // Drop error below (deadzone - hysteresis) = 0.10 deg -> valve should turn off
    auto result = HornetRcs::tick(state, 0.05f);  // 0.05 < 0.10 threshold

    zassert_true(result.has_value(), "tick should succeed");
    auto [cw, ccw, metrics] = *result;
    zassert_equal(cw,  THRUSTER_OFF, "CW thruster should be off after error drops below threshold");
    zassert_equal(ccw, THRUSTER_OFF, "CCW thruster should turn off after error drops below threshold");
}

// --- Reset-specific tests ---

ZTEST(HornetRcs_tests, test_reset_clears_cw_valve_state)
{
    // Activate CW, then verify reset brings both thrusters to idle
    EstimatedState state = make_identity_state();
    HornetRcs::reset();
    HornetRcs::tick(state, -30.0f);  // activate CW
    HornetRcs::reset();

    auto result = HornetRcs::tick(state, 0.0f);
    zassert_true(result.has_value(), "tick after reset should succeed");
    auto [cw, ccw, metrics] = *result;
    zassert_equal(cw,  THRUSTER_OFF, "CW should be off after reset with zero error");
    zassert_equal(ccw, THRUSTER_OFF, "CCW should be off after reset with zero error");
}

ZTEST(HornetRcs_tests, test_reset_allows_cw_after_ccw_session)
{
    // After a CCW session, reset must allow CW to fire normally (no leftover p_valve=1)
    EstimatedState state = make_identity_state();
    HornetRcs::reset();
    HornetRcs::tick(state, 30.0f);   // activate CCW (p_valve = 1)
    HornetRcs::reset();

    auto result = HornetRcs::tick(state, -30.0f);  // large CW command
    zassert_true(result.has_value(), "tick should succeed");
    auto [cw, ccw, metrics] = *result;
    zassert_equal(cw,  THRUSTER_ON,  "CW thruster should fire after reset with large negative error");
    zassert_equal(ccw, THRUSTER_OFF, "CCW thruster should not fire for negative error after reset");
}

ZTEST(HornetRcs_tests, test_reset_allows_ccw_after_cw_session)
{
    // After a CW session, reset must allow CCW to fire normally (no leftover p_valve=-1)
    EstimatedState state = make_identity_state();
    HornetRcs::reset();
    HornetRcs::tick(state, -30.0f);  // activate CW (p_valve = -1)
    HornetRcs::reset();

    auto result = HornetRcs::tick(state, 30.0f);  // large CCW command
    zassert_true(result.has_value(), "tick should succeed");
    auto [cw, ccw, metrics] = *result;
    zassert_equal(ccw, THRUSTER_ON,  "CCW thruster should fire after reset with large positive error");
    zassert_equal(cw,  THRUSTER_OFF, "CW thruster should not fire for positive error after reset");
}

ZTEST(HornetRcs_tests, test_reset_clears_min_pulse_timer)
{
    // p_timer is set to min_pulse when a thruster fires. Reset must clear it so
    // a sub-deadzone error on the very next tick does NOT keep the thruster on.
    EstimatedState state = make_identity_state();
    HornetRcs::reset();

    // Fire CCW to set p_timer = min_pulse
    HornetRcs::tick(state, 30.0f);

    // Reset immediately (before timer would expire) and tick with sub-deadzone error
    HornetRcs::reset();
    auto result = HornetRcs::tick(state, 0.05f);  // well below deadzone=0.15

    zassert_true(result.has_value(), "tick should succeed after reset");
    auto [cw, ccw, metrics] = *result;
    zassert_equal(cw,  THRUSTER_OFF, "CW should be off: timer was cleared by reset");
    zassert_equal(ccw, THRUSTER_OFF, "CCW should be off: timer cleared, error below deadzone");
}

ZTEST(HornetRcs_tests, test_consecutive_resets_are_idempotent)
{
    // Two consecutive resets should behave identically to a single reset
    EstimatedState state = make_identity_state();
    HornetRcs::reset();
    HornetRcs::tick(state, 30.0f);
    HornetRcs::reset();
    HornetRcs::reset();  // second reset must not corrupt state

    auto result = HornetRcs::tick(state, 0.0f);
    zassert_true(result.has_value(), "tick after double reset should succeed");
    auto [cw, ccw, metrics] = *result;
    zassert_equal(cw,  THRUSTER_OFF, "CW should be off after double reset");
    zassert_equal(ccw, THRUSTER_OFF, "CCW should be off after double reset");
}

ZTEST(HornetRcs_tests, test_reset_mid_hysteresis_disables_valve)
{
    // While p_valve=1 is held open by hysteresis, reset must bring it to neutral.
    // After reset, an error in the hysteresis band should NOT fire the thruster
    // because p_valve was cleared (the valve needs a fresh deadzone cross).
    //
    // With Kp=2, the control effort = 2 * error. For this test to be meaningful:
    //   - error = 0.06 -> control_effort = 0.12
    //   - 0.12 > (deadzone - hysteresis) = 0.10  -> valve stays ON if p_valve=1
    //   - 0.12 < deadzone = 0.15               -> valve does NOT fire from p_valve=0
    EstimatedState state = make_identity_state();
    HornetRcs::reset();

    HornetRcs::tick(state, 30.0f);      // open CCW (p_valve = 1)
    HornetRcs::tick(state, 0.06f);      // control_effort=0.12 in hysteresis band, CCW stays on
    HornetRcs::reset();                 // reset while still in hysteresis region

    // From clean p_valve=0 state: control_effort=0.12 < deadzone=0.15 -> no fire
    auto result = HornetRcs::tick(state, 0.06f);
    zassert_true(result.has_value(), "tick should succeed after reset mid-hysteresis");
    auto [cw, ccw, metrics] = *result;
    zassert_equal(cw,  THRUSTER_OFF, "CW should be off: error in hysteresis band after reset");
    zassert_equal(ccw, THRUSTER_OFF, "CCW should be off: reset cleared p_valve, error below deadzone");
}

ZTEST(HornetRcs_tests, test_reset_then_large_error_activates_thruster_cleanly)
{
    // After many ticks in one direction followed by reset, the very first tick
    // with a large error in the opposite direction should activate the correct thruster.
    // This validates both p_valve and PID (prev_meas) are cleared.
    EstimatedState state = make_identity_state();
    HornetRcs::reset();

    for (int i = 0; i < 10; i++) {
        HornetRcs::tick(state, 30.0f);  // accumulate PID history in CCW direction
    }
    HornetRcs::reset();

    // First tick after reset with large negative error must activate CW cleanly
    auto result = HornetRcs::tick(state, -30.0f);
    zassert_true(result.has_value(), "tick should succeed after session reset");
    auto [cw, ccw, metrics] = *result;
    zassert_equal(cw,  THRUSTER_ON,  "CW should fire on first tick after reset with large negative error");
    zassert_equal(ccw, THRUSTER_OFF, "CCW should not fire for negative error");
}

ZTEST_SUITE(HornetRcs_tests, NULL, NULL, NULL, NULL, NULL);
