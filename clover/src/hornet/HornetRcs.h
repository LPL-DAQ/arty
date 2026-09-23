#pragma once

#include "../Error.h"
#include "clover.pb.h"
#include <expected>
#include <tuple>

namespace HornetRcs {
void reset();
std::expected<std::tuple<float, float, HornetRcsMetrics>, Error> tick(EstimatedState state, float roll_command_deg);

#if CONFIG_TEST
// Overrides the uptime used to compute dt. reset() clears the override.
// previous_timestamp is currently never updated, so the override value becomes dt
// directly rather than acting as a clock. That changes once previous_timestamp is
// fixed to track the previous call.
void set_now_ms_for_testing(int64_t ms);
#endif
}  // namespace HornetRcs
