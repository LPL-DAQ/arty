#pragma once

#include "Error.h"
#include <expected>
#include <optional>
#include <tuple>

namespace AnalogSensors {

std::expected<void, Error> init();

void configure();

void start_sense();
std::optional<std::tuple<PTs, TCs, float>> read();

}  // namespace AnalogSensors
