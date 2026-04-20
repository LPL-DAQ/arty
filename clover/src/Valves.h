#pragma once

#include <expected>
#include "Error.h"

namespace Valves {
    std::expected<void, Error> init();
    std::expected<void, Error> handle_configure_valves_request(const ConfigureValvesRequest& req);
    std::expected<void, Error> handle_actuate_valve_request(const ActuateValveRequest& req);
}
