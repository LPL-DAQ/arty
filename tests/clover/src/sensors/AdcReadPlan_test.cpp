#include "sensors/AdcReadPlan.h"
#include <zephyr/ztest.h>

// Two 16-channel ADCs, like Ranger 1. Input channels are listed out of channel order on purpose.
constexpr std::array<int, 5> INPUT_ADC = {0, 0, 0, 1, 1};
constexpr std::array<int, 5> INPUT_CHANNEL_ID = {6, 1, 10, 3, 0};

using Plan = AdcReadPlan<2, 16>;

ZTEST(AdcReadPlan_tests, test_nothing_configured_reads_nothing)
{
    Plan plan = build_adc_read_plan<2, 16>(INPUT_ADC, INPUT_CHANNEL_ID, std::array<bool, 5>{});

    zassert_equal(plan.masks[0], 0u);
    zassert_equal(plan.masks[1], 0u);
    zassert_equal(plan.num_readings(0), 0);
    zassert_equal(plan.reading_to_input_channel[0][0], -1);
    zassert_equal(plan.reading_to_input_channel[1][0], -1);
}

ZTEST(AdcReadPlan_tests, test_only_configured_channels_are_read)
{
    // Inputs 0 (adc0 ch6) and 4 (adc1 ch0) configured
    Plan plan = build_adc_read_plan<2, 16>(INPUT_ADC, INPUT_CHANNEL_ID, std::array<bool, 5>{true, false, false, false, true});

    zassert_equal(plan.masks[0], 1u << 6);
    zassert_equal(plan.masks[1], 1u << 0);
    zassert_equal(plan.reading_to_input_channel[0][0], 0);
    zassert_equal(plan.reading_to_input_channel[0][1], -1);
    zassert_equal(plan.reading_to_input_channel[1][0], 4);
    zassert_equal(plan.reading_to_input_channel[1][1], -1);
}

ZTEST(AdcReadPlan_tests, test_readings_follow_adc_channel_order)
{
    // ADC returns ch1, ch6, ch10 in that order, which are inputs 1, 0, 2
    Plan plan = build_adc_read_plan<2, 16>(INPUT_ADC, INPUT_CHANNEL_ID, std::array<bool, 5>{true, true, true, false, false});

    zassert_equal(plan.masks[0], (1u << 1) | (1u << 6) | (1u << 10));
    zassert_equal(plan.num_readings(0), 3);
    zassert_equal(plan.reading_to_input_channel[0][0], 1);
    zassert_equal(plan.reading_to_input_channel[0][1], 0);
    zassert_equal(plan.reading_to_input_channel[0][2], 2);
    zassert_equal(plan.reading_to_input_channel[0][3], -1);
    zassert_equal(plan.masks[1], 0u);
}

ZTEST_SUITE(AdcReadPlan_tests, NULL, NULL, NULL, NULL, NULL);
