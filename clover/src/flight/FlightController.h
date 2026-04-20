#pragma once

#include "Error.h"
#include "clover.pb.h"
#include <expected>
#include <tuple>

namespace FlightController {
void reset();
std::expected<std::tuple<float, float, float, FlightControllerMetrics>, Error>
tick(float x_command_m, float y_command_m, float z_command_m);

std::expected<void, Error> handle_configure_gains(const ConfigureFlightControllerGainsRequest& req);
}  // namespace FlightController
