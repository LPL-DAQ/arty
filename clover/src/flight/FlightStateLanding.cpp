#include "FlightStateLanding.h"

void FlightStateLanding::init() {
    // Minimal landing initialization.
}

std::pair<FlightStateOutput, FlightLandingData> FlightStateLanding::tick(int64_t current_time, int64_t start_time) {
    FlightStateOutput out{};
    FlightLandingData data{};

    if (current_time - start_time > 1000) {
        out.next_state = FlightState_FLIGHT_STATE_IDLE;
    } else {
        out.next_state = FlightState_FLIGHT_STATE_LANDING;
    }

    return std::make_pair(out, data);
}
