#include "RCSStateFlight.h"
#include "flight/FlightController.h"

void RCSStateFlight::init() {
    // Minimal flight state initialization.
}

std::pair<RCSStateOutput, RCSFlightData> RCSStateFlight::tick() {
    RCSStateOutput out{};
    RCSFlightData data{};

    // TODO: do something with target_roll
    float target_roll = FlightController::get_roll_position();
    out.next_state = RCSState_RCS_STATE_FLIGHT;

    return std::make_pair(out, data);
}
