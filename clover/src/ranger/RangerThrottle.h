#pragma once

#include "Error.h"
#include "clover.pb.h"
#include <expected>
#include <tuple>

namespace RangerThrottle {
void reset();
std::expected<std::tuple<ThrottleValveCommand, ThrottleValveCommand, RangerThrottleMetrics>, Error> tick(AnalogSensorReadings& analog_sensors, float thrust_command_lbf);
}  // namespace RangerThrottle
