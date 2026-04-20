#pragma once

#include "Error.h"
#include "clover.pb.h"
#include <expected>
#include <tuple>

namespace HornetThrottle {
void reset();
std::expected<std::tuple<float, HornetThrottleMetrics>, Error> tick(float thrust_command_);
}  // namespace HornetThrottle
