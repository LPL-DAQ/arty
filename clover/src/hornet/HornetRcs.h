#pragma once

#include "Error.h"
#include "clover.pb.h"
#include <expected>
#include <tuple>

namespace HornetRcs {
void reset();
std::expected<std::tuple<bool, bool, HornetRcsMetrics>, Error> tick(float roll_command_deg);
}  // namespace HornetRcs
