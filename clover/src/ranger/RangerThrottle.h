#pragma once

#include "Error.h"
#include "clover.pb.h"
#include <cstdint>
#include <expected>
#include <tuple>

namespace RangerThrottle {
void reset();
std::expected<std::tuple<ThrottleValveCommand, ThrottleValveCommand, RangerThrottleMetrics>, Error> tick(AnalogSensorReadings& analog_sensors, float thrust_command_lbf);
std::expected<ThrottleValveCommand, Error>
calibration_tick(ThrottleValveType valve, uint32_t timestamp, float valve_pos, float valve_pos_enc);
void calibration_reset(ThrottleValveType valve, float valve_pos, float valve_pos_enc);
}  // namespace RangerThrottle
