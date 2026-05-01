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
static constexpr float kToleranceChangeAlphaCmd = 1e-5f;
static constexpr float kTolerancePredictedThrust = 6.0f; //
static constexpr float kToleranceValveDeg = 0.3f;

static_assert(TIME_ALPHA_X_LEN == TIME_PREDICTED_THRUST_X_LEN);
static_assert(TIME_ALPHA_X_LEN == TIME_TARGET_THRUST_X_LEN);
static_assert(TIME_ALPHA_X_LEN == TIME_CHANGE_ALPHA_CMD_X_LEN);
static_assert(TIME_ALPHA_X_LEN == TIME_FUEL_VALVE_DEG_X_LEN);
static_assert(TIME_ALPHA_X_LEN == TIME_LOX_VALVE_DEG_X_LEN);

struct ActiveControlSample {
	float t;
	RangerThrottleMetrics metrics;
	float fuel_target_deg;
	float lox_target_deg;
	float expected_alpha;
	float expected_change_alpha_cmd;
	float expected_thrust_from_alpha_lbf;
	float expected_fuel_deg;
	float expected_lox_deg;
};

static ActiveControlSample run_active_control_sample(std::size_t i)
{
	float t = TIME_TARGET_THRUST_X_MIN + static_cast<float>(i) * TIME_TARGET_THRUST_X_GAP;
	float t_prev = t - TIME_TARGET_THRUST_X_GAP;

	float alpha_prev = TimeAlpha::sample(t_prev);
	float predicted_thrust_prev_lbf = TimePredictedThrust::sample(t_prev);
	float target_thrust_lbf = TimeTargetThrust::sample(t);

	float alpha_state = alpha_prev;
	RangerThrottleMetrics metrics = RangerThrottleMetrics_init_default;
	auto [fuel_cmd, lox_cmd] = RangerThrottle::active_control_test(
		alpha_state, predicted_thrust_prev_lbf, target_thrust_lbf, metrics);

	return ActiveControlSample{
		.t = t,
		.metrics = metrics,
		.fuel_target_deg = fuel_cmd.target_deg,
		.lox_target_deg = lox_cmd.target_deg,
		.expected_alpha = TimeAlpha::sample(t),
		.expected_change_alpha_cmd = TimeChangeAlphaCmd::sample(t) / 1000.0f, // simulink samples before integration
		.expected_thrust_from_alpha_lbf = TimePredictedThrust::sample(t),
		.expected_fuel_deg = TimeFuelValveDeg::sample(t),
		.expected_lox_deg = TimeLoxValveDeg::sample(t),
	};
}

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

ZTEST(RangerThrottle_tests, test_active_control_time_series_alpha_lut)
{
	RangerThrottle::reset();

	for (std::size_t i = 300; i < TIME_TARGET_THRUST_X_LEN; ++i) {
		ActiveControlSample sample = run_active_control_sample(i);

		char assert_msg[192];

		std::snprintf(
			assert_msg,
			sizeof(assert_msg),
			"alpha mismatch at t=%.3f (expected=%.6f, actual=%.6f)",
			(double)sample.t,
			(double)sample.expected_alpha,
			(double)sample.metrics.alpha);
		zassert_within(sample.metrics.alpha, sample.expected_alpha, kToleranceAlpha, assert_msg);
	}
}

ZTEST(RangerThrottle_tests, test_active_control_time_series_change_alpha_cmd_lut)
{
	RangerThrottle::reset();

	for (std::size_t i = 300; i < TIME_TARGET_THRUST_X_LEN; ++i) {
		ActiveControlSample sample = run_active_control_sample(i);

		char assert_msg[192];

		std::snprintf(
			assert_msg,
			sizeof(assert_msg),
			"change alpha cmd mismatch at t=%.3f (expected=%.6f, actual=%.6f)",
			(double)sample.t,
			(double)sample.expected_change_alpha_cmd,
			(double)sample.metrics.change_alpha_cmd);
		zassert_within(
			sample.metrics.change_alpha_cmd,
			sample.expected_change_alpha_cmd,
			kToleranceChangeAlphaCmd,
			assert_msg);
	}
}

ZTEST(RangerThrottle_tests, test_active_control_time_series_thrust_from_alpha_lut)
{
	RangerThrottle::reset();

	for (std::size_t i = 300; i < TIME_TARGET_THRUST_X_LEN; ++i) {
		ActiveControlSample sample = run_active_control_sample(i);

		char assert_msg[192];

		std::snprintf(
			assert_msg,
			sizeof(assert_msg),
			"predicted thrust mismatch at t=%.3f (expected=%.6f, actual=%.6f)",
			(double)sample.t,
			(double)sample.expected_thrust_from_alpha_lbf,
			(double)sample.metrics.thrust_from_alpha_lbf);
		zassert_within(
			sample.metrics.thrust_from_alpha_lbf,
			sample.expected_thrust_from_alpha_lbf,
			kTolerancePredictedThrust,
			assert_msg);
	}
}

ZTEST(RangerThrottle_tests, test_active_control_time_series_fuel_valve_lut)
{
	RangerThrottle::reset();

	for (std::size_t i = 300; i < TIME_TARGET_THRUST_X_LEN; ++i) {
		ActiveControlSample sample = run_active_control_sample(i);

		char assert_msg[192];

		std::snprintf(
			assert_msg,
			sizeof(assert_msg),
			"fuel valve command mismatch at t=%.3f (expected=%.6f, actual=%.6f)",
			(double)sample.t,
			(double)sample.expected_fuel_deg,
			(double)sample.fuel_target_deg);
		zassert_within(sample.fuel_target_deg, sample.expected_fuel_deg, kToleranceValveDeg, assert_msg);
	}
}

ZTEST(RangerThrottle_tests, test_active_control_time_series_lox_valve_lut)
{
	RangerThrottle::reset();

	for (std::size_t i = 300; i < TIME_TARGET_THRUST_X_LEN; ++i) {
		ActiveControlSample sample = run_active_control_sample(i);

		char assert_msg[192];

		std::snprintf(
			assert_msg,
			sizeof(assert_msg),
			"lox valve command mismatch at t=%.3f (expected=%.6f, actual=%.6f)",
			(double)sample.t,
			(double)sample.expected_lox_deg,
			(double)sample.lox_target_deg);
		zassert_within(sample.lox_target_deg, sample.expected_lox_deg, kToleranceValveDeg, assert_msg);
	}
}

ZTEST_SUITE(RangerThrottle_tests, NULL, NULL, NULL, NULL, NULL);
