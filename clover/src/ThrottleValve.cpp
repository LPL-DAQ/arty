#include "ThrottleValve.h"

LOG_MODULE_REGISTER(ThrottleValve);

std::expected<void, Error> handle_throttle_reset_valve_position(const ThrottleResetValvePositionRequest& req)
{
    if(req.valve == ThrottleValveType_FUEL) {
        FuelValve::reset_pos(req.new_pos_deg);
    }
    else if(req.valve == ThrottleValveType_LOX) {
        LoxValve::reset_pos(req.new_pos_deg);
    }
    else {
        return std::unexpected(Error::from_cause("Unknown valve type: %d", req.valve));
    }

    return {};
}
