#include "RCSStateFlight.h"
#include "flight/FlightController.h"

void RCSStateFlight::init() {
    // Minimal flight state initialization.
}

std::pair<RCSStateOutput, RCSFlightData> RCSStateFlight::tick() {
    RCSStateOutput out{};
    RCSFlightData data{};

    float target_roll = FlightController::get_roll_position();
    // RCS Control -> CW or CW or Neither
    out.CW = false;
    out.CCW = false;
    out.next_state = RCSState_RCS_STATE_FLIGHT;

    return std::make_pair(out, data);
}
