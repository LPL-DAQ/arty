#include "../../../../clover/src/flight/FlightController.h"
#include <zephyr/ztest.h>
#include <cmath>

ZTEST(FlightController_tests, test_flight_tick_given_default_state)
{
    // Reset the flight controller state
    FlightController::reset();

    // Create an estimated state with identity quaternion (no rotation)
    EstimatedState state = EstimatedState_init_default;
    state.R_WB.qw = 1.0f;  // Identity quaternion
    state.R_WB.qx = 0.0f;
    state.R_WB.qy = 0.0f;
    state.R_WB.qz = 0.0f;
    // Position and velocity all zero
    state.position.x = 0.0f;
    state.position.y = 0.0f;
    state.position.z = 0.0f;
    state.velocity.x = 0.0f;
    state.velocity.y = 0.0f;
    state.velocity.z = 0.0f;

    // Call tick with zero desired positions
    auto result = FlightController::tick(state, 0.0f, 0.0f, 0.0f);

    // Verify the call succeeded
    zassert_true(result.has_value(), "FlightController::tick should succeed");

    auto [thrust_cmd, pitch_cmd, yaw_cmd, metrics] = *result;

    // Check actual tilt angles - with identity quaternion, should be near zero
    zassert_within(metrics.actual_world_tilt_x_rad, 0.0f, 0.01f, "actual_world_tilt_x_rad should be near zero for identity quaternion");
    zassert_within(metrics.actual_world_tilt_y_rad, 0.0f, 0.01f, "actual_world_tilt_y_rad should be near zero for identity quaternion");

    // Check desired tilt angles - with zero position error and zero velocity, should be near zero
    zassert_within(metrics.desired_world_tilt_x_rad, 0.0f, 0.01f, "desired_world_tilt_x_rad should be near zero for zero error");
    zassert_within(metrics.desired_world_tilt_y_rad, 0.0f, 0.01f, "desired_world_tilt_y_rad should be near zero for zero error");

    // Check desired vertical velocity - should be zero with zero position error
    zassert_within(metrics.desired_vertical_velocity_m_s, 0.0f, 0.01f, "desired_vertical_velocity_m_s should be near zero for zero error");

    // Check commanded accelerations - with zero error and zero velocity, should be near zero
    zassert_within(metrics.commanded_vertical_acceleration_m_s2, 0.0f, 0.01f, "commanded_vertical_acceleration_m_s2 should be near zero for zero error");
    zassert_within(metrics.commanded_pitch_acceleration_rad_s2, 0.0f, 0.01f, "commanded_pitch_acceleration_rad_s2 should be near zero for zero error");
    zassert_within(metrics.commanded_yaw_acceleration_rad_s2, 0.0f, 0.01f, "commanded_yaw_acceleration_rad_s2 should be near zero for zero error");
}

ZTEST(FlightController_tests, test_actual_tilt_with_pitched_quaternion)
{
    // Reset the flight controller state
    FlightController::reset();

    // Create a quaternion representing a ~45 degree pitch
    // quat from axis-angle: axis=(1,0,0), angle=pi/4 (~45 deg)
    float angle = 3.1415f / 4.0f;
    float qw = cosf(angle / 2.0f);
    float qx = sinf(angle / 2.0f);

    EstimatedState state = EstimatedState_init_default;
    state.R_WB.qw = qw;
    state.R_WB.qx = qx;
    state.R_WB.qy = 0.0f;
    state.R_WB.qz = 0.0f;
    state.position.x = 0.0f;
    state.position.y = 0.0f;
    state.position.z = 0.0f;
    state.velocity.x = 0.0f;
    state.velocity.y = 0.0f;
    state.velocity.z = 0.0f;

    auto result = FlightController::tick(state, 0.0f, 0.0f, 0.0f);
    zassert_true(result.has_value(), "FlightController::tick should succeed for pitched quaternion");

    auto [thrust_cmd, pitch_cmd, yaw_cmd, metrics] = *result;

    // Actual pitch tilt should reflect the quaternion rotation (~45 deg = ~0.785 rad)
    zassert_true(metrics.actual_world_tilt_x_rad > 0.6f, "Pitch tilt should be positive and substantial");
    zassert_true(metrics.actual_world_tilt_x_rad < 1.0f, "Pitch tilt should not exceed ~45 degrees");
    zassert_within(metrics.actual_world_tilt_y_rad, 0.0f, 0.01f, "actual_world_tilt_y_rad should be near zero for pure pitch quaternion");
}

ZTEST(FlightController_tests, test_lateral_position_error_creates_tilt_command)
{
    // Reset the flight controller state
    FlightController::reset();

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

    // Request 5m lateral displacement
    auto result = FlightController::tick(state, 5.0f, 0.0f, 0.0f);
    zassert_true(result.has_value(), "FlightController::tick should succeed for lateral error");

    auto [thrust_cmd, pitch_cmd, yaw_cmd, metrics] = *result;

    // With 5m error in X direction, desired X tilt should be substantial (> 5 deg)
    zassert_true(metrics.desired_world_tilt_x_rad > 5.0f * 3.1415f/180.0f || metrics.desired_world_tilt_x_rad < -5.0f * 3.1415f/180.0f,
                 "Desired tilt X should be non-zero with position error");
}

ZTEST(FlightController_tests, test_vertical_position_error_creates_velocity_command)
{
    // Reset the flight controller state
    FlightController::reset();

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

    // Request 10m altitude gain
    auto result = FlightController::tick(state, 0.0f, 0.0f, 10.0f);
    zassert_true(result.has_value(), "FlightController::tick should succeed for vertical error");

    auto [thrust_cmd, pitch_cmd, yaw_cmd, metrics] = *result;

    // With 10m vertical error, desired vertical velocity should be positive
    zassert_true(metrics.desired_vertical_velocity_m_s > 0.0f, "Desired vertical velocity should be positive with altitude error");
}

ZTEST(FlightController_tests, test_metrics_do_not_contain_nan_or_inf)
{
    // Reset the flight controller state
    FlightController::reset();

    EstimatedState state = EstimatedState_init_default;
    state.R_WB.qw = 1.0f;
    state.R_WB.qx = 0.0f;
    state.R_WB.qy = 0.0f;
    state.R_WB.qz = 0.0f;
    state.position.x = -100.0f;  // Large error
    state.position.y = 100.0f;   // Large error
    state.position.z = -50.0f;   // Large error
    state.velocity.x = 20.0f;
    state.velocity.y = -20.0f;
    state.velocity.z = 15.0f;

    auto result = FlightController::tick(state, 100.0f, -100.0f, 50.0f);
    zassert_true(result.has_value(), "FlightController::tick should succeed for large errors");

    auto [thrust_cmd, pitch_cmd, yaw_cmd, metrics] = *result;

    // Check that no metrics are NaN or Inf
    zassert_false(std::isnan(metrics.actual_world_tilt_x_rad), "actual_world_tilt_x_rad should not be NaN");
    zassert_false(std::isnan(metrics.actual_world_tilt_y_rad), "actual_world_tilt_y_rad should not be NaN");
    zassert_false(std::isnan(metrics.desired_world_tilt_x_rad), "desired_world_tilt_x_rad should not be NaN");
    zassert_false(std::isnan(metrics.desired_world_tilt_y_rad), "desired_world_tilt_y_rad should not be NaN");
    zassert_false(std::isnan(metrics.desired_vertical_velocity_m_s), "desired_vertical_velocity_m_s should not be NaN");
    zassert_false(std::isnan(metrics.commanded_vertical_acceleration_m_s2), "commanded_vertical_acceleration_m_s2 should not be NaN");
    zassert_false(std::isnan(metrics.commanded_pitch_acceleration_rad_s2), "commanded_pitch_acceleration_rad_s2 should not be NaN");
    zassert_false(std::isnan(metrics.commanded_yaw_acceleration_rad_s2), "commanded_yaw_acceleration_rad_s2 should not be NaN");
    zassert_false(std::isnan(metrics.tvc_pitch_command_deg), "tvc_pitch_command_deg should not be NaN");
    zassert_false(std::isnan(metrics.tvc_yaw_command_deg), "tvc_yaw_command_deg should not be NaN");

    zassert_false(std::isinf(metrics.actual_world_tilt_x_rad), "actual_world_tilt_x_rad should not be Inf");
    zassert_false(std::isinf(metrics.actual_world_tilt_y_rad), "actual_world_tilt_y_rad should not be Inf");
    zassert_false(std::isinf(metrics.desired_world_tilt_x_rad), "desired_world_tilt_x_rad should not be Inf");
    zassert_false(std::isinf(metrics.desired_world_tilt_y_rad), "desired_world_tilt_y_rad should not be Inf");
    zassert_false(std::isinf(metrics.desired_vertical_velocity_m_s), "desired_vertical_velocity_m_s should not be Inf");
    zassert_false(std::isinf(metrics.commanded_vertical_acceleration_m_s2), "commanded_vertical_acceleration_m_s2 should not be Inf");
    zassert_false(std::isinf(metrics.commanded_pitch_acceleration_rad_s2), "commanded_pitch_acceleration_rad_s2 should not be Inf");
    zassert_false(std::isinf(metrics.commanded_yaw_acceleration_rad_s2), "commanded_yaw_acceleration_rad_s2 should not be Inf");
    zassert_false(std::isinf(metrics.tvc_pitch_command_deg), "tvc_pitch_command_deg should not be Inf");
    zassert_false(std::isinf(metrics.tvc_yaw_command_deg), "tvc_yaw_command_deg should not be Inf");
}

ZTEST(FlightController_tests, test_tvc_commands_are_within_gimbal_limits)
{
    // Reset the flight controller state
    FlightController::reset();

    EstimatedState state = EstimatedState_init_default;
    state.R_WB.qw = 1.0f;
    state.R_WB.qx = 0.0f;
    state.R_WB.qy = 0.0f;
    state.R_WB.qz = 0.0f;
    state.position.x = 50.0f;  // Large lateral error
    state.position.y = 50.0f;
    state.position.z = 0.0f;
    state.velocity.x = 0.0f;
    state.velocity.y = 0.0f;
    state.velocity.z = 0.0f;

    auto result = FlightController::tick(state, 0.0f, 0.0f, 0.0f);
    zassert_true(result.has_value(), "FlightController::tick should succeed for large lateral error");

    auto [thrust_cmd, pitch_cmd, yaw_cmd, metrics] = *result;

    // TVC commands should be clamped to reasonable gimbal limits (~12 degrees)
    zassert_true(metrics.tvc_pitch_command_deg >= -15.0f, "TVC pitch should not be excessively negative");
    zassert_true(metrics.tvc_pitch_command_deg <= 15.0f, "TVC pitch should not be excessively positive");
    zassert_true(metrics.tvc_yaw_command_deg >= -15.0f, "TVC yaw should not be excessively negative");
    zassert_true(metrics.tvc_yaw_command_deg <= 15.0f, "TVC yaw should not be excessively positive");
}

ZTEST(FlightController_tests, test_multiple_ticks_with_consistent_state)
{
    // Reset the flight controller state
    FlightController::reset();

    EstimatedState state = EstimatedState_init_default;
    state.R_WB.qw = 1.0f;
    state.R_WB.qx = 0.0f;
    state.R_WB.qy = 0.0f;
    state.R_WB.qz = 0.0f;
    state.position.x = 1.0f;
    state.position.y = 1.0f;
    state.position.z = 1.0f;
    state.velocity.x = 0.0f;
    state.velocity.y = 0.0f;
    state.velocity.z = 0.0f;

    // First tick
    auto result1 = FlightController::tick(state, 0.0f, 0.0f, 0.0f);
    zassert_true(result1.has_value(), "First tick should succeed");
    auto [thrust1, pitch1, yaw1, metrics1] = *result1;

    // Second tick with same state - should be different due to integral terms
    auto result2 = FlightController::tick(state, 0.0f, 0.0f, 0.0f);
    zassert_true(result2.has_value(), "Second tick should succeed");
    auto [thrust2, pitch2, yaw2, metrics2] = *result2;

    // Third tick - should continue to change
    auto result3 = FlightController::tick(state, 0.0f, 0.0f, 0.0f);
    zassert_true(result3.has_value(), "Third tick should succeed");
    auto [thrust3, pitch3, yaw3, metrics3] = *result3;

    // Due to integral terms, commanded accelerations should change over ticks
    zassert_false(metrics1.commanded_pitch_acceleration_rad_s2 == metrics2.commanded_pitch_acceleration_rad_s2 &&
                  metrics2.commanded_pitch_acceleration_rad_s2 == metrics3.commanded_pitch_acceleration_rad_s2,
                  "Commanded accelerations should change due to integral terms");
    zassert_false(metrics1.commanded_yaw_acceleration_rad_s2 == metrics2.commanded_yaw_acceleration_rad_s2 &&
                  metrics2.commanded_yaw_acceleration_rad_s2 == metrics3.commanded_yaw_acceleration_rad_s2,
                  "Commanded accelerations should change due to integral terms");
    zassert_false(metrics1.commanded_vertical_acceleration_m_s2 == metrics2.commanded_vertical_acceleration_m_s2 &&
                  metrics2.commanded_vertical_acceleration_m_s2 == metrics3.commanded_vertical_acceleration_m_s2,
                  "Commanded accelerations should change due to integral terms");

    FlightController::reset();
    auto result_pos = FlightController::tick(state, 5.0f, 0.0f, 0.0f);
    zassert_true(result_pos.has_value(), "Positive error tick should succeed");
    auto [thrust_pos, pitch_pos, yaw_pos, metrics_pos] = *result_pos;

    // Reset and test negative error in X
    FlightController::reset();
    auto result_neg = FlightController::tick(state, -5.0f, 0.0f, 0.0f);
    zassert_true(result_neg.has_value(), "Negative error tick should succeed");
    auto [thrust_neg, pitch_neg, yaw_neg, metrics_neg] = *result_neg;

    // Desired tilts should have opposite signs
    zassert_true((metrics_pos.desired_world_tilt_x_rad > 0.0f && metrics_neg.desired_world_tilt_x_rad < 0.0f) ||
                 (metrics_pos.desired_world_tilt_x_rad < 0.0f && metrics_neg.desired_world_tilt_x_rad > 0.0f),
                 "Opposing position errors should produce opposing tilt commands");

// Now set all I/D gains to zero and verify outputs are identical across ticks
    ConfigureFlightControllerGainsRequest req = ConfigureFlightControllerGainsRequest_init_default;
    req.pidXTilt_ki = 0.0f; req.has_pidXTilt_ki = true;
    req.pidXTilt_kd = 0.0f; req.has_pidXTilt_kd = true;
    req.pidYTilt_ki = 0.0f; req.has_pidYTilt_ki = true;
    req.pidYTilt_kd = 0.0f; req.has_pidYTilt_kd = true;
    req.pidX_ki = 0.0f; req.has_pidX_ki = true;
    req.pidX_kd = 0.0f; req.has_pidX_kd = true;
    req.pidY_ki = 0.0f; req.has_pidY_ki = true;
    req.pidY_kd = 0.0f; req.has_pidY_kd = true;
    req.pidZ_ki = 0.0f; req.has_pidZ_ki = true;
    req.pidZ_kd = 0.0f; req.has_pidZ_kd = true;
    req.pidZVelocity_ki = 0.0f; req.has_pidZVelocity_ki = true;
    req.pidZVelocity_kd = 0.0f; req.has_pidZVelocity_kd = true;
    auto gain_result = FlightController::handle_configure_gains(req);
    zassert_true(gain_result.has_value(), "Gain configuration should succeed");
    // Run two ticks, should be identical
    auto result_a = FlightController::tick(state, 0.0f, 0.0f, 0.0f);
    auto result_b = FlightController::tick(state, 0.0f, 0.0f, 0.0f);
    zassert_true(result_a.has_value() && result_b.has_value(), "Both ticks should succeed with zero I/D gains");
    auto [ta, pa, ya, ma] = *result_a;
    auto [tb, pb, yb, mb] = *result_b;
    zassert_within(ma.commanded_pitch_acceleration_rad_s2, mb.commanded_pitch_acceleration_rad_s2, 0.0001f, "Pitch acceleration should be identical with zero I/D gains");
    zassert_within(ma.commanded_yaw_acceleration_rad_s2, mb.commanded_yaw_acceleration_rad_s2, 0.0001f, "Yaw acceleration should be identical with zero I/D gains");
    zassert_within(ma.commanded_vertical_acceleration_m_s2, mb.commanded_vertical_acceleration_m_s2, 0.0001f, "Vertical acceleration should be identical with zero I/D gains");

}

ZTEST(FlightController_tests, test_non_unit_quaternion)
{
    // Reset the flight controller state
    FlightController::reset();

    EstimatedState state = EstimatedState_init_default;
    // Non-unit quaternion (should be normalized internally)
    state.R_WB.qw = 2.0f;
    state.R_WB.qx = 1.0f;
    state.R_WB.qy = 0.5f;
    state.R_WB.qz = 0.0f;
    state.position.x = 0.0f;
    state.position.y = 0.0f;
    state.position.z = 0.0f;
    state.velocity.x = 0.0f;
    state.velocity.y = 0.0f;
    state.velocity.z = 0.0f;

    auto result = FlightController::tick(state, 0.0f, 0.0f, 0.0f);
    zassert_true(result.has_value(), "Should handle non-unit quaternion");

    auto [thrust_cmd, pitch_cmd, yaw_cmd, metrics] = *result;

    // Should not produce NaN or Inf
    zassert_false(std::isnan(metrics.actual_world_tilt_x_rad), "actual_world_tilt_x_rad should not be NaN for non-unit quaternion");
    zassert_false(std::isnan(metrics.actual_world_tilt_y_rad), "actual_world_tilt_y_rad should not be NaN for non-unit quaternion");
    zassert_false(std::isinf(metrics.actual_world_tilt_x_rad), "actual_world_tilt_x_rad should not be Inf for non-unit quaternion");
    zassert_false(std::isinf(metrics.actual_world_tilt_y_rad), "actual_world_tilt_y_rad should not be Inf for non-unit quaternion");
}

ZTEST_SUITE(FlightController_tests, NULL, NULL, NULL, NULL, NULL);

ZTEST(FlightController_tests, test_desired_tilt_clamped_to_max)
{
    // Reset the flight controller state
    FlightController::reset();

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

    // Large lateral error that should cause tilt > 8 degrees if not clamped
    auto result = FlightController::tick(state, 100.0f, 0.0f, 0.0f);
    zassert_true(result.has_value(), "FlightController::tick should succeed for large lateral error (clamp test)");

    auto [thrust_cmd, pitch_cmd, yaw_cmd, metrics] = *result;

    // Desired tilt should be clamped to maxTiltDeg (10 degrees = ~0.175 radians)
    zassert_true(std::abs(metrics.desired_world_tilt_x_rad) <= 0.175f, "Desired tilt X should be clamped to <= 10 degrees");
    zassert_true(std::abs(metrics.desired_world_tilt_y_rad) <= 0.175f, "Desired tilt Y should be clamped to <= 10 degrees");
}

// TODO: this and the following test is badly done
ZTEST(FlightController_tests, test_loop_count_effect_on_outer_loop)
{
    // Reset the flight controller state
    FlightController::reset();

    EstimatedState state = EstimatedState_init_default;
    state.R_WB.qw = 1.0f;
    state.R_WB.qx = 0.0f;
    state.R_WB.qy = 0.0f;
    state.R_WB.qz = 0.0f;
    state.position.x = 5.0f;
    state.position.y = 0.0f;
    state.position.z = 0.0f;
    state.velocity.x = 0.0f;
    state.velocity.y = 0.0f;
    state.velocity.z = 0.0f;

    // Run two ticks to get to loopCount=2 (outer loop doesn't run)
    auto result1 = FlightController::tick(state, 0.0f, 0.0f, 0.0f);
    zassert_true(result1.has_value(), "First tick should succeed (loop count test)");
    auto [thrust1, pitch1, yaw1, metrics1] = *result1;

    auto result2 = FlightController::tick(state, 0.0f, 0.0f, 0.0f);
    zassert_true(result2.has_value(), "Second tick should succeed (loop count test)");
    auto [thrust2, pitch2, yaw2, metrics2] = *result2;

    auto result3 = FlightController::tick(state, 0.0f, 0.0f, 0.0f);
    zassert_true(result3.has_value(), "Third tick should succeed (loop count test)");
    auto [thrust3, pitch3, yaw3, metrics3] = *result3;

    // The third tick should have different desired tilts due to outer loop running
    bool tilt_changed = (std::abs(metrics3.desired_world_tilt_x_rad - metrics2.desired_world_tilt_x_rad) > 0.001f) ||
                        (std::abs(metrics3.desired_world_tilt_y_rad - metrics2.desired_world_tilt_y_rad) > 0.001f);
    zassert_true(tilt_changed, "Outer loop should update desired tilts when loopCount % 3 == 0");
}

ZTEST(FlightController_tests, test_loop_count_outer_loop_vs_integral_drift)
{
    // Reset the flight controller state
    FlightController::reset();

    EstimatedState state = EstimatedState_init_default;
    state.R_WB.qw = 1.0f;
    state.R_WB.qx = 0.0f;
    state.R_WB.qy = 0.0f;
    state.R_WB.qz = 0.0f;
    state.position.x = 5.0f;
    state.position.y = 0.0f;
    state.position.z = 0.0f;
    state.velocity.x = 0.0f;
    state.velocity.y = 0.0f;
    state.velocity.z = 0.0f;

    // Run two ticks to get some integral drift
    auto result1 = FlightController::tick(state, 0.0f, 0.0f, 0.0f);
    zassert_true(result1.has_value(), "First tick should succeed (integral drift test)");
    auto [thrust1, pitch1, yaw1, metrics1] = *result1;
    auto result2 = FlightController::tick(state, 0.0f, 0.0f, 0.0f);
    zassert_true(result2.has_value(), "Second tick should succeed (integral drift test)");
    auto [thrust2, pitch2, yaw2, metrics2] = *result2;
    float drift = std::abs(metrics2.desired_world_tilt_x_rad - metrics1.desired_world_tilt_x_rad);
    // Set loopCount to 3 so next tick runs outer loop
    auto result3 = FlightController::tick(state, 0.0f, 0.0f, 0.0f);
    zassert_true(result3.has_value(), "Third tick should succeed (integral drift test)");
    auto [thrust3, pitch3, yaw3, metrics3] = *result3;
    float outerloop_change = std::abs(metrics3.desired_world_tilt_x_rad - metrics2.desired_world_tilt_x_rad);
    // The outer loop update should cause a larger change than the integral drift
    zassert_true(outerloop_change > drift * 2.0f, "Outer loop update should cause a larger change than integral drift");
}

ZTEST(FlightController_tests, test_multiple_ticks_integral_accumulation_and_reset)
{
    // Reset the flight controller state
    FlightController::reset();

    EstimatedState state = EstimatedState_init_default;
    state.R_WB.qw = 1.0f;
    state.R_WB.qx = 0.0f;
    state.R_WB.qy = 0.0f;
    state.R_WB.qz = 0.0f;
    state.position.x = 2.0f;
    state.position.y = 2.0f;
    state.position.z = 2.0f;
    state.velocity.x = 0.0f;
    state.velocity.y = 0.0f;
    state.velocity.z = 0.0f;

    // Run multiple ticks and collect metrics
    FlightControllerMetrics metrics_history[5];
    for (int i = 0; i < 5; ++i) {
        auto result = FlightController::tick(state, 0.0f, 0.0f, 0.0f);
        zassert_true(result.has_value(), "Tick should succeed in accumulation test");
        auto [thrust, pitch, yaw, metrics] = *result;
        metrics_history[i] = metrics;
    }

    // Check that at least one acceleration changes over time due to integral terms
    bool has_accumulated = false;
    for (size_t i = 1; i < 5; ++i) {
        if (std::abs(metrics_history[i].commanded_pitch_acceleration_rad_s2 - metrics_history[i-1].commanded_pitch_acceleration_rad_s2) > 0.001f ||
            std::abs(metrics_history[i].commanded_yaw_acceleration_rad_s2 - metrics_history[i-1].commanded_yaw_acceleration_rad_s2) > 0.001f ||
            std::abs(metrics_history[i].commanded_vertical_acceleration_m_s2 - metrics_history[i-1].commanded_vertical_acceleration_m_s2) > 0.001f) {
            has_accumulated = true;
            break;
        }
    }
    zassert_true(has_accumulated, "Integral terms should cause gradual changes in commanded accelerations");

    // Now reset and run the same sequence again
    FlightController::reset();
    auto result_reset = FlightController::tick(state, 0.0f, 0.0f, 0.0f);
    zassert_true(result_reset.has_value(), "Tick after reset should succeed");
    auto [thrust_reset, pitch_reset, yaw_reset, metrics_reset] = *result_reset;

    // After reset, should match the first tick before accumulation
    zassert_within(metrics_reset.commanded_pitch_acceleration_rad_s2, metrics_history[0].commanded_pitch_acceleration_rad_s2, 0.01f, "Pitch acceleration after reset should match initial");
    zassert_within(metrics_reset.commanded_yaw_acceleration_rad_s2, metrics_history[0].commanded_yaw_acceleration_rad_s2, 0.01f, "Yaw acceleration after reset should match initial");
    zassert_within(metrics_reset.commanded_vertical_acceleration_m_s2, metrics_history[0].commanded_vertical_acceleration_m_s2, 0.01f, "Vertical acceleration after reset should match initial");
}
