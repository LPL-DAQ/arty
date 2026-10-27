#include "../../../../clover/src/sensors/HallCurrent.h"
#include <zephyr/ztest.h>

static constexpr float TOL_A = 0.05f;

ZTEST(HallCurrent_tests, test_edf_zero_and_rated)
{
    zassert_within(edf_current_amps(2.495f), 0.0f, TOL_A);
    zassert_within(edf_current_amps(3.120f), 400.0f, TOL_A);
    zassert_within(edf_current_amps(1.870f), -400.0f, TOL_A);
}

ZTEST(HallCurrent_tests, test_bus_zero_and_rated)
{
    zassert_within(bus_current_amps(2.495f), 0.0f, TOL_A);
    zassert_within(bus_current_amps(3.120f), 200.0f, TOL_A);
    zassert_within(bus_current_amps(1.870f), -200.0f, TOL_A);
}

ZTEST(HallCurrent_tests, test_divider_scales_adc_volts_before_conversion)
{
    zassert_within(hall_adc_volts_to_amps(1.2475f, 2.0f, 2.495f, 1.5625f), 0.0f, TOL_A);
    zassert_within(hall_adc_volts_to_amps(1.560f, 2.0f, 2.495f, 1.5625f), 400.0f, TOL_A);
    zassert_within(hall_adc_volts_to_amps(1.560f, 2.0f, 2.495f, 3.125f), 200.0f, TOL_A);
    zassert_within(hall_adc_volts_to_amps(3.120f, 1.0f, 2.495f, 1.5625f), edf_current_amps(3.120f), 1e-4f);
}

ZTEST_SUITE(HallCurrent_tests, NULL, NULL, NULL, NULL, NULL);
