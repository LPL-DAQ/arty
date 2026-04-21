#pragma once

#include "Error.h"
#include <expected>

namespace Valves {
std::expected<void, Error> init();

std::expected<ValveStates, Error> get_valve_states();

std::expected<void, Error> handle_configure_valves_request(const ConfigureValvesRequest& req);
std::expected<void, Error> handle_actuate_valve_request(const ActuateValveRequest& req);
}  // namespace Valves
