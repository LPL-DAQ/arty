#pragma once

#include "Error.h"
#include "clover.pb.h"
#include <expected>
#include <tuple>

namespace RangerRcs {
void reset();
std::expected<std::tuple<bool, bool, RangerRcsMetrics>, Error> tick(float roll_command_deg);
}  // namespace RangerRcs
