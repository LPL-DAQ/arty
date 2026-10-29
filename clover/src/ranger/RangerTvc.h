#pragma once

#include "Error.h"
#include "clover.pb.h"
#include <expected>
#include <tuple>

namespace RangerTvc {
void reset();
std::expected<std::tuple<TvcActuatorCommand, TvcActuatorCommand, RangerTvcMetrics>, Error> tick(float position);
}  // namespace RangerTvc
