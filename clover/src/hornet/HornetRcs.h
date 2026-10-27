#pragma once

#include "../Error.h"
#include "clover.pb.h"
#include <cstdint>
#include <expected>
#include <tuple>

namespace HornetRcs {
void reset();
/// Open-loop bench command: throttle percent (0-100) to pulse, capped at HORNET_RCS_BENCH_MAX_THROTTLE_PERCENT.
uint32_t bench_throttle_percent_to_pulse_us(float throttle_percent);
std::expected<std::tuple<float, float, HornetRcsMetrics>, Error> tick(EstimatedState state, float roll_command_deg);

#if CONFIG_TEST
// Overrides the clock used to compute dt. reset() clears the override.
void set_now_us_for_testing(int64_t us);
#endif
}  // namespace HornetRcs
