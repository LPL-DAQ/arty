#pragma once

#include "Error.h"
#include <expected>

#ifdef CONFIG_VALVES

namespace Valves {
std::expected<void, Error> init();

std::expected<ValveStates, Error> get_valve_states();

std::expected<void, Error> handle_configure_valves(const ConfigureValvesRequest& req);
std::expected<void, Error> handle_actuate_valve(const ActuateValveRequest& req);
}  // namespace Valves

#endif  // CONFIG_VALVES
