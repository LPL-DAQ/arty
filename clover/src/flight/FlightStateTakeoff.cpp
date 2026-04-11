#include "FlightStateTakeoff.h"

void FlightStateTakeoff::init() {
    // Minimal takeoff initialization.
}

std::pair<FlightStateOutput, FlightTakeoffData> FlightStateTakeoff::tick(int64_t current_time, int64_t start_time) {
    FlightStateOutput out{};
    FlightTakeoffData data{};

    if (current_time - start_time > 1000) {
        out.next_state = FlightState_FLIGHT_STATE_FLIGHT_SEQ;
    } else {
        out.next_state = FlightState_FLIGHT_STATE_TAKEOFF;
    }

    return std::make_pair(out, data);
}
