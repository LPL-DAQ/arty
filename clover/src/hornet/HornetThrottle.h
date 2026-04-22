#pragma once

#include "../Error.h"
#include "clover.pb.h"
#include <expected>
#include <tuple>

namespace HornetThrottle {
void reset();
/// Takes vertical acceleration command (m/s²) and returns (pulse_us, metrics).
/// Thrust and lbf conversion are computed internally.
std::expected<std::tuple<float, HornetThrottleMetrics>, Error> tick(float z_accel_m_s2);
}  // namespace HornetThrottle
