#pragma once

#include "Error.h"
#include "clover.pb.h"
#include <expected>
#include <tuple>

namespace RangerTvc {
void reset();
std::expected<std::tuple<TvcActuatorCommand, TvcActuatorCommand, RangerTvcMetrics>, Error> tick(float pitch_command_deg, float yaw_command_deg);
}  // namespace RangerTvc
