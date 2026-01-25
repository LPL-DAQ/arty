#include "ThrottleValve.h"

LOG_MODULE_REGISTER(throttle_valve);

std::expected<void, Error> handle_reset_valve_position(const ResetValvePositionRequest& req)
{
    switch (req.valve) {
    case Valve_FUEL:
        FuelValve::reset_pos(req.new_pos_deg);
        break;
    case Valve_LOX:
        LoxValve::reset_pos(req.new_pos_deg);
        break;
    default:
        return std::unexpected(Error::from_cause("invalid valve enum (must be fuel or lox) `%d`", req.valve));
    }

    return {};
}
