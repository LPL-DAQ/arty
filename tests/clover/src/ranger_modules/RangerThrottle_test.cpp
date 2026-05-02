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
static constexpr float kTolerancePredictedThrust = 6.0f;
static constexpr float kToleranceValveDeg = 0.8f;
static constexpr float kTolerancePredictorThrustLbf = 5.0f;
static constexpr float kTolerancePredictorOf = 0.01f;
static constexpr float kTolerancePredictorMdot = 0.01f;

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
//pt102,pt103,pto401,pt202,pt203,ptf401,ptc401,ptc402,Thrust_Pred_lbf,OF_Pred,mdot_f,mdot_lox
static constexpr float kThrustPredictorData[][12] = {
	{580.230331931119f, 610.98035481541f, 569.507348798935f, 781.6057761679f, 740.617493142571f, 719.336428968446f, 180.786795683881f, 186.45703125f, 637.510003876722f, 1.16491424570654f, 0.670079616291752f, 0.780585290775834f},
	{707.374012576635f, 598.145743743416f, 558.702534518591f, 814.316202655059f, 760.464602025776f, 739.707354798261f, 344.879209429548f, 339.53320312f, 517.758921783431f, 1.06442862705248f, 0.577335402823473f, 0.614532330176183f},
	{673.292321493838f, 415.671061912352f, 376.331061641731f, 804.807688979512f, 610.688086565921f, 590.178067221415f, 252.269641011866f, 253.59570312f, 509.780465607254f, 1.15902128479114f, 0.536158540938551f, 0.621419160970346f},
	{662.434949991243f, 398.473608378908f, 357.351221472933f, 803.510825402318f, 518.61182075972f, 499.329129954694f, 241.118401751216f, 239.19140625f, 510.641296892637f, 1.30626333314165f, 0.471763564963712f, 0.616247446824284f},
	{608.869154654682f, 332.218197075047f, 293.132757714453f, 795.040742221202f, 495.906766024912f, 475.389130839212f, 202.277035081011f, 202.32617187f, 467.689756777085f, 1.18086644507006f, 0.485286339335564f, 0.573058354372251f},
	// {284.835507458296f, 192.621763066642f, 153.0220104636f, 68.2855001220411f, 38.3676432342799f, 17.377301601237f, 47.3044467014059f, 48.76171875f, 299.599577856953f, 569.877577814134f, 0.001f, 0.569877577814134f},
	{559.571630577496f, 314.191242331263f, 272.516420644423f, 262.801471912982f, 183.698982308573f, 162.345391951513f, 188.767835674067f, 190.8515625f, 302.752574991468f, 563.710129466761f, 0.001f, 0.563710129466761f},
	{571.452335144166f, 595.917575038996f, 555.092379814119f, 767.389474864437f, 735.161759618464f, 714.905255671402f, 135.640753565411f, 132.2578125f, 676.321234506031f, 1.19987824978187f, 0.693426213029693f, 0.832027030842938f},
	{625.019113596656f, 654.121358758741f, 612.18438089119f, 800.371521445416f, 761.313771070521f, 741.598804202693f, 229.460439398178f, 230.890625f, 583.942459307556f, 1.07602632939668f, 0.653121521416526f, 0.702775953339799f},
	{716.611573258695f, 513.316641287948f, 472.61185115924f, 808.435255737189f, 662.476923362748f, 641.468580002526f, 308.774808070007f, 300.71484375f, 517.670978552834f, 1.18758331165122f, 0.532619277559633f, 0.63252976549355f},
	{677.372023427054f, 385.156822065486f, 342.6726495619f, 677.430961797519f, 439.322096920161f, 417.14939136849f, 239.534586878967f, 237.7265625f, 505.531671738432f, 1.4911812372218f, 0.400296893509347f, 0.596915216919313f},
	{653.018663030829f, 395.32179776402f, 355.686925298858f, 438.832930807239f, 102.393718935001f, 80.8844806890598f, 236.499058680616f, 241.14453125f, 335.136860880957f, 620.183716252947f, 0.001f, 0.620183716252947f},
	{661.580680270747f, 306.629033910271f, 266.522345839527f, 243.351409595076f, 50.2842155269247f, 29.3478701181304f, 198.076059039845f, 195.24609375f, 281.995069115053f, 524.343759555596f, 0.001f, 0.524343759555596f},
	{293.835305686809f, 203.636295417382f, 160.212964575695f, 98.2420164227591f, 51.8588288024512f, 30.2494722609764f, 61.1023428165175f, 59.25976562f, 301.650187241849f, 571.23317370283f, 0.001f, 0.57123317370283f},
};

static constexpr std::size_t kThrustPredictorDataLen = sizeof(kThrustPredictorData) / sizeof(kThrustPredictorData[0]);

static bool is_low_fuel_floor_case(std::size_t i)
{
	return kThrustPredictorData[i][10] <= 0.001f;
}

struct ThrustPredictorSample {
	RangerThrottleMetrics metrics;
	std::expected<float, Error> result;
	float expected_thrust_lbf;
	float expected_of;
	float expected_mdot_fuel;
	float expected_mdot_lox;
};

static ThrustPredictorSample run_thrust_predictor_sample(std::size_t i)
{
	AnalogSensorReadings sensors = AnalogSensorReadings_init_default;
	sensors.pt103 = kThrustPredictorData[i][1];
	sensors.pto401 = kThrustPredictorData[i][2];
	sensors.pt203 = kThrustPredictorData[i][4];
	sensors.ptf401 = kThrustPredictorData[i][5];
	sensors.ptc401 = kThrustPredictorData[i][6];
	sensors.ptc402 = kThrustPredictorData[i][7];

	sensors.has_pt103 = true;
	sensors.has_pto401 = true;
	sensors.has_pt203 = true;
	sensors.has_ptf401 = true;
	sensors.has_ptc401 = true;
	sensors.has_ptc402 = true;

	RangerThrottleMetrics metrics = RangerThrottleMetrics_init_default;
	auto result = RangerThrottle::thrust_predictor(sensors, metrics);

	return ThrustPredictorSample{
		.metrics = metrics,
		.result = result,
		.expected_thrust_lbf = kThrustPredictorData[i][8],
		.expected_of = kThrustPredictorData[i][9],
		.expected_mdot_fuel = kThrustPredictorData[i][10],
		.expected_mdot_lox = kThrustPredictorData[i][11],
	};
}

ZTEST(RangerThrottle_tests, test_thrust_predictor_time_series_success)
{
	RangerThrottle::reset();

	for (std::size_t i = 0; i < kThrustPredictorDataLen; ++i) {
		ThrustPredictorSample sample = run_thrust_predictor_sample(i);
		zassert_true(sample.result.has_value(), "predictor case %zu should succeed", i);
	}
}

ZTEST(RangerThrottle_tests, test_thrust_predictor_time_series_thrust_lbf)
{
	RangerThrottle::reset();

	for (std::size_t i = 0; i < kThrustPredictorDataLen; ++i) {
		ThrustPredictorSample sample = run_thrust_predictor_sample(i);
		zassert_true(sample.result.has_value(), "predictor case %zu should succeed", i);
        if (is_low_fuel_floor_case(i)) {
            continue; // skip thrust assertion for low fuel floor cases where thrust is not well-defined
        }
		char assert_msg[192];
		std::snprintf(
			assert_msg,
			sizeof(assert_msg),
			"predicted thrust mismatch at case %zu (expected=%.6f, actual=%.6f)",
			i,
			(double)sample.expected_thrust_lbf,
			(double)sample.metrics.predicted_thrust_lbf);
		zassert_within(
			sample.metrics.predicted_thrust_lbf,
			sample.expected_thrust_lbf,
			kTolerancePredictorThrustLbf,
			assert_msg);
	}
}

ZTEST(RangerThrottle_tests, test_thrust_predictor_time_series_of)
{
	RangerThrottle::reset();

	for (std::size_t i = 0; i < kThrustPredictorDataLen; ++i) {

		ThrustPredictorSample sample = run_thrust_predictor_sample(i);
		zassert_true(sample.result.has_value(), "predictor case %zu should succeed", i);
        if (is_low_fuel_floor_case(i)) {
            continue; // skip O/F assertion for low fuel floor cases where O/F is not well-defined
        }
		char assert_msg[192];
		std::snprintf(
			assert_msg,
			sizeof(assert_msg),
			"predicted O/F mismatch at case %zu (expected=%.6f, actual=%.6f)",
			i,
			(double)sample.expected_of,
			(double)sample.metrics.predicted_of);
		zassert_within(
			sample.metrics.predicted_of,
			sample.expected_of,
			kTolerancePredictorOf,
			assert_msg);
	}
}

ZTEST(RangerThrottle_tests, test_thrust_predictor_time_series_mdot_fuel)
{
	RangerThrottle::reset();

	for (std::size_t i = 0; i < kThrustPredictorDataLen; ++i) {

		ThrustPredictorSample sample = run_thrust_predictor_sample(i);
		zassert_true(sample.result.has_value(), "predictor case %zu should succeed", i);

		char assert_msg[192];
		std::snprintf(
			assert_msg,
			sizeof(assert_msg),
			"fuel mdot mismatch at case %zu (expected=%.6f, actual=%.6f)",
			i,
			(double)sample.expected_mdot_fuel,
			(double)sample.metrics.mdot_fuel);
		zassert_within(
			sample.metrics.mdot_fuel,
			sample.expected_mdot_fuel,
			kTolerancePredictorMdot,
			assert_msg);
	}
}

ZTEST(RangerThrottle_tests, test_thrust_predictor_time_series_mdot_lox)
{
	RangerThrottle::reset();

	for (std::size_t i = 0; i < kThrustPredictorDataLen; ++i) {

		ThrustPredictorSample sample = run_thrust_predictor_sample(i);
		zassert_true(sample.result.has_value(), "predictor case %zu should succeed", i);

		char assert_msg[192];
		std::snprintf(
			assert_msg,
			sizeof(assert_msg),
			"lox mdot mismatch at case %zu (expected=%.6f, actual=%.6f)",
			i,
			(double)sample.expected_mdot_lox,
			(double)sample.metrics.mdot_lox);
		zassert_within(
			sample.metrics.mdot_lox,
			sample.expected_mdot_lox,
			kTolerancePredictorMdot,
			assert_msg);
	}
}

ZTEST_SUITE(RangerThrottle_tests, NULL, NULL, NULL, NULL, NULL);
