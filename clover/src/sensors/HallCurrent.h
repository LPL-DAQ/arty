#pragma once

// Tamura L37S...S05 Hall-effect current sensors, read through a raw (volts) analog channel.
// Output is 2.495 V typical at 0 A and moves 0.625 V at rated current.
// Hardware: each sensor needs a 10k load from Vout to GND, the zero offset should be measured per
// sensor at 0 A, and a fault or large overcurrent can drive the output above the 3.3 V ADC limit.
// The divider ratio is sensor volts per ADC volt, for a divider between Vout and the ADC pin.
constexpr float EDF_CURRENT_ZERO_OFFSET_V = 2.495f;  // L37S400S05M, 400 A rated
constexpr float EDF_CURRENT_MV_PER_A = 1.5625f;
constexpr float EDF_CURRENT_DIVIDER_RATIO = 1.0f;
constexpr float BUS_CURRENT_ZERO_OFFSET_V = 2.495f;  // L37S200S05M, 200 A rated
constexpr float BUS_CURRENT_MV_PER_A = 3.125f;
constexpr float BUS_CURRENT_DIVIDER_RATIO = 1.0f;

constexpr float hall_volts_to_amps(float volts, float zero_offset_v, float mv_per_a)
{
    return (volts - zero_offset_v) / (mv_per_a * 1e-3f);
}

constexpr float hall_adc_volts_to_amps(float adc_volts, float divider_ratio, float zero_offset_v, float mv_per_a)
{
    return hall_volts_to_amps(adc_volts * divider_ratio, zero_offset_v, mv_per_a);
}

constexpr float edf_current_amps(float adc_volts)
{
    return hall_adc_volts_to_amps(adc_volts, EDF_CURRENT_DIVIDER_RATIO, EDF_CURRENT_ZERO_OFFSET_V, EDF_CURRENT_MV_PER_A);
}

constexpr float bus_current_amps(float adc_volts)
{
    return hall_adc_volts_to_amps(adc_volts, BUS_CURRENT_DIVIDER_RATIO, BUS_CURRENT_ZERO_OFFSET_V, BUS_CURRENT_MV_PER_A);
}
