#include "../../../../clover/src/ranger/RangerThrottle.h"

#include <cstddef>
#include <cstdio>
#include <zephyr/ztest.h>

#include "alpha.h"
#include "change_alpha_cmd.h"
#include "fuel_valve_deg.h"
#include "lox_valve_deg.h"
#include "predicted_thrust.h"
#include "target_thrust.h"

static constexpr float kToleranceAlpha = 1e-4f;
static constexpr float kToleranceChangeAlphaCmd = 1e-4f;
static constexpr float kTolerancePredictedThrust = 1e-3f;
static constexpr float kToleranceValveDeg = 1e-3f;

// ZTEST(RangerThrottle_tests, test_thrust_predictor_single_case_template)
// {
// 	RangerThrottle::reset();

// 	// Fill this case with one input snapshot and one expected output snapshot.

//     AnalogSensorReadings sensors = AnalogSensorReadings_init_default;

//     float predicted_thrust_lbf = 0.0f, // predicted_thrust_lbf
//         0.0f, //
//     float mdot_fuel = 0.0f, //
//     float  mdot_lox =0.0f, //
//     float tolerance = 0.01f; // tolerance for all comparisons


// 	RangerThrottleMetrics metrics = RangerThrottleMetrics_init_default;
// 	auto result = RangerThrottle::thrust_predictor(sensors, metrics);

//     zassert_true(result.has_value(), "predictor case should succeed");

//     zassert_within(metrics.predicted_thrust_lbf, predicted_thrust_lbf, tolerance,
//                     "predicted thrust mismatch: %s", test_case.name);

//     zassert_within(metrics.predicted_of, expected.predicted_of,  expected.tolerance,
//                     "predicted O/F mismatch: %s", test_case.name);

//     zassert_within(metrics.mdot_fuel, expected.mdot_fuel, expected.tolerance,
//                     "fuel mdot mismatch: %s", test_case.name);

//     zassert_within(metrics.mdot_lox,    expected.mdot_lox, expected.tolerance,
//                     "lox mdot mismatch: %s", test_case.name);
// }

ZTEST(RangerThrottle_tests, test_active_control_time_series_lut)
{
	RangerThrottle::reset();

	static_assert(TIME_ALPHA_X_LEN == TIME_PREDICTED_THRUST_X_LEN);
	static_assert(TIME_ALPHA_X_LEN == TIME_TARGET_THRUST_X_LEN);
	static_assert(TIME_ALPHA_X_LEN == TIME_CHANGE_ALPHA_CMD_X_LEN);
	static_assert(TIME_ALPHA_X_LEN == TIME_FUEL_VALVE_DEG_X_LEN);
	static_assert(TIME_ALPHA_X_LEN == TIME_LOX_VALVE_DEG_X_LEN);

	for (std::size_t i = 300; i < TIME_TARGET_THRUST_X_LEN; ++i) {
		float t = TIME_TARGET_THRUST_X_MIN + static_cast<float>(i) * TIME_TARGET_THRUST_X_GAP;
		float t_prev = t - TIME_TARGET_THRUST_X_GAP;

		float alpha_prev = TimeAlpha::sample(t_prev);
		float predicted_thrust_prev_lbf = TimePredictedThrust::sample(t_prev);
		float target_thrust_lbf = TimeTargetThrust::sample(t);

		float expected_alpha = TimeAlpha::sample(t);
		// float expected_predicted_thrust_lbf = TimePredictedThrust::sample(t);
		// float expected_fuel_deg = TimeFuel_valve_deg::sample(t);
		// float expected_lox_deg = TimeLoxValveDeg::sample(t);

		float alpha_state = alpha_prev;
		RangerThrottleMetrics metrics = RangerThrottleMetrics_init_default;
		auto [fuel_cmd, lox_cmd] = RangerThrottle::active_control_test(
			alpha_state, predicted_thrust_prev_lbf, target_thrust_lbf, metrics);

		char assert_msg[192];

		std::snprintf(
			assert_msg,
			sizeof(assert_msg),
			"alpha mismatch at t=%.3f (expected=%.6f, actual=%.6f)",
			(double)t,
			(double)expected_alpha,
			(double)metrics.alpha);
		zassert_within(metrics.alpha, expected_alpha, kToleranceAlpha, assert_msg);

	// 	std::snprintf(
	// 		assert_msg,
	// 		sizeof(assert_msg),
	// 		"predicted thrust mismatch at t=%.3f (expected=%.6f, actual=%.6f)",
	// 		(double)t,
	// 		(double)expected_predicted_thrust_lbf,
	// 		(double)metrics.thrust_from_alpha_lbf);
	// 	zassert_within(metrics.thrust_from_alpha_lbf, expected_predicted_thrust_lbf, kTolerancePredictedThrust, assert_msg);

	// 	std::snprintf(
	// 		assert_msg,
	// 		sizeof(assert_msg),
	// 		"fuel valve command mismatch at t=%.3f (expected=%.6f, actual=%.6f)",
	// 		(double)t,
	// 		(double)expected_fuel_deg,
	// 		(double)fuel_cmd.target_deg);
	// 	zassert_within(fuel_cmd.target_deg, expected_fuel_deg, kToleranceValveDeg, assert_msg);

	// 	std::snprintf(
	// 		assert_msg,
	// 		sizeof(assert_msg),
	// 		"lox valve command mismatch at t=%.3f (expected=%.6f, actual=%.6f)",
	// 		(double)t,
	// 		(double)expected_lox_deg,
	// 		(double)lox_cmd.target_deg);
	// 	zassert_within(lox_cmd.target_deg, expected_lox_deg, kToleranceValveDeg, assert_msg);
	}
}

ZTEST_SUITE(RangerThrottle_tests, NULL, NULL, NULL, NULL, NULL);
