#include "FlightStateAbort.h"

void FlightStateAbort::init() {
    // Controller handles actuation behavior for abort now.
}

std::pair<FlightStateOutput, FlightAbortData> FlightStateAbort::tick(int64_t current_time, int64_t entry_time) {
    FlightStateOutput out{};
    FlightAbortData data{};

    if (current_time - entry_time > 500) {
        out.next_state = FlightState_FLIGHT_STATE_IDLE;
    } else {
        out.next_state = FlightState_FLIGHT_STATE_ABORT;
    }

    return std::make_pair(out, data);
}
