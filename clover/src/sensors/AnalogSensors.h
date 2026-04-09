#pragma once

#include "../Error.h"
#include "clover.pb.h"
#include <expected>
#include <optional>
#include <tuple>

namespace AnalogSensors {

std::expected<void, Error> init();

std::expected<void, Error> handle_configure_analog_sensors(const ConfigureAnalogSensorsRequest& req);

void start_sense();
std::optional<std::pair<AnalogSensorReadings, float>> read();

}  // namespace AnalogSensors
