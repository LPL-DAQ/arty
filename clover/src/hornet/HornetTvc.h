#pragma once

#include "Error.h"
#include "clover.pb.h"
#include <expected>
#include <tuple>

namespace HornetTvc {
void reset();
std::expected<std::tuple<float, float, HornetTvcMetrics>, Error> tick(float pitch_command_deg, float yaw_command_deg);
}  // namespace HornetTvc
