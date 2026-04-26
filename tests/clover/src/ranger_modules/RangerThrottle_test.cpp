#include "../../../../clover/src/ranger/RangerThrottle.h"

#include <array>
#include <cmath>
#include <zephyr/ztest.h>

struct PredictorExpected {
	float predicted_thrust_lbf;
	float predicted_of;
	float mdot_fuel;
	float mdot_lox;
	float tolerance;
};

struct PredictorCase {
	const char* name;
	AnalogSensorReadings input;
	PredictorExpected expected;
};

struct ActiveControlExpected {
	float fuel_target_deg;
	float lox_target_deg;
	float alpha;
	float change_alpha_cmd;
	float clamped_change_alpha_cmd;
	float thrust_from_alpha_lbf;
	float tolerance;
};

struct ActiveControlStep {
	const char* name;
    float alpha;
	float predicted_thrust_lbf;
	float thrust_command_lbf;
	ActiveControlExpected expected;
};

ZTEST(RangerThrottle_tests, test_thrust_predictor_single_case_template)
{
	RangerThrottle::reset();

	// Fill this case with one input snapshot and one expected output snapshot.
	PredictorCase test_case = {
		"TODO_fill_predictor_case",
		AnalogSensorReadings_init_default,
		{
			0.0f,
			0.0f,
			0.0f,
			0.0f,
			0.001f,
		},
	};

	RangerThrottleMetrics metrics = RangerThrottleMetrics_init_default;
	auto result = RangerThrottle::test_hooks::thrust_predictor(test_case.input, metrics);

    zassert_true(result.has_value(), "predictor case should succeed: %s", test_case.name);

    zassert_within(*result, test_case.expected.predicted_thrust_lbf, test_case.expected.tolerance,
                    "predicted thrust mismatch: %s", test_case.name);

    zassert_within(metrics.predicted_of, test_case.expected.predicted_of, test_case.expected.tolerance,
                    "predicted O/F mismatch: %s", test_case.name);

    zassert_within(metrics.mdot_fuel, test_case.expected.mdot_fuel, test_case.expected.tolerance,
                    "fuel mdot mismatch: %s", test_case.name);

    zassert_within(metrics.mdot_lox, test_case.expected.mdot_lox, test_case.expected.tolerance,
                    "lox mdot mismatch: %s", test_case.name);
}

ZTEST(RangerThrottle_tests, test_active_control_time_series_template)
{
	RangerThrottle::reset();

	// Fill this with your full time-series expected inputs and outputs.
	const std::array<ActiveControlStep, 1> series = {{
		{
			"TODO_fill_step_0",
			-1.0f,
			0.0f,
            0.0f
			{
				0.0f,
				0.0f,
				0.0f,
				0.0f,
				0.0f,
				0.001f,
			},
		},
	}};

	float alpha_state = -1.0f;
	for (const auto& step : series) {
		RangerThrottleMetrics metrics = RangerThrottleMetrics_init_default;
		auto [fuel_cmd, lox_cmd] = RangerThrottle::test_hooks::active_control(
			alpha_state, step.predicted_thrust_lbf, step.thrust_command_lbf, metrics);

		zassert_within(fuel_cmd.target_deg, step.expected.fuel_target_deg, step.expected.tolerance,
					   "fuel command mismatch: %s", step.name);
		zassert_within(lox_cmd.target_deg, step.expected.lox_target_deg, step.expected.tolerance,
					   "lox command mismatch: %s", step.name);
        zassert_within(lox_cmd.target_deg, step.expected.lox_target_deg, step.expected.tolerance,
                        "lox command mismatch: %s", step.name);

        zassert_within(metrics.alpha, step.expected.alpha, step.expected.tolerance,
                        "alpha mismatch: %s", step.name);

        zassert_within(metrics.change_alpha_cmd, step.expected.change_alpha_cmd, step.expected.tolerance,
                        "change alpha mismatch: %s", step.name);

        zassert_within(metrics.clamped_change_alpha_cmd, step.expected.clamped_change_alpha_cmd,
                        step.expected.tolerance, "clamped change alpha mismatch: %s", step.name);

        zassert_within(metrics.thrust_from_alpha_lbf, step.expected.thrust_from_alpha_lbf,
                        step.expected.tolerance, "thrust-from-alpha mismatch: %s", step.name);
    
	}
}

ZTEST_SUITE(RangerThrottle_tests, NULL, NULL, NULL, NULL, NULL);
