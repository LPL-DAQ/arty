#pragma once

#include <array>
#include <bit>
#include <cstddef>
#include <cstdint>

/// Which channels each ADC reads, and which input channel each reading belongs to.
template <int NUM_ADCS, int MAX_CHANNELS_PER_ADC>
struct AdcReadPlan {
    /// Channel bitmask passed to each ADC's sequence. 0 means the ADC is not read.
    std::array<uint32_t, NUM_ADCS> masks{};
    /// Maps each ADC's reading index to an input channel, -1 past the last reading.
    std::array<std::array<int, MAX_CHANNELS_PER_ADC>, NUM_ADCS> reading_to_input_channel{};

    constexpr int num_readings(int adc) const
    {
        return std::popcount(masks[adc]);
    }
};

/// Builds a read plan covering only configured input channels. Input channel i lives on ADC input_adc[i] at
/// channel input_channel_id[i]. ADCs return readings in ascending channel order, so the map follows that order
/// rather than the input channel order.
template <int NUM_ADCS, int MAX_CHANNELS_PER_ADC, size_t NUM_INPUTS>
constexpr AdcReadPlan<NUM_ADCS, MAX_CHANNELS_PER_ADC> build_adc_read_plan(const std::array<int, NUM_INPUTS>& input_adc,
                                                                         const std::array<int, NUM_INPUTS>& input_channel_id,
                                                                         const std::array<bool, NUM_INPUTS>& configured)
{
    AdcReadPlan<NUM_ADCS, MAX_CHANNELS_PER_ADC> plan;
    for (int adc = 0; adc < NUM_ADCS; ++adc) {
        int reading_index = 0;
        for (int channel_id = 0; channel_id < MAX_CHANNELS_PER_ADC; ++channel_id) {
            for (size_t input = 0; input < NUM_INPUTS; ++input) {
                if (configured[input] && input_adc[input] == adc && input_channel_id[input] == channel_id) {
                    plan.masks[adc] |= (1u << channel_id);
                    plan.reading_to_input_channel[adc][reading_index++] = static_cast<int>(input);
                    break;
                }
            }
        }
        for (; reading_index < MAX_CHANNELS_PER_ADC; ++reading_index) {
            plan.reading_to_input_channel[adc][reading_index] = -1;
        }
    }
    return plan;
}
