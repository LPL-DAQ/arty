#include "FlightStateIdle.h"

void FlightStateIdle::init() {
    // Ready for a new flight sequence or system idle.
}

std::pair<FlightStateOutput, FlightIdleData> FlightStateIdle::tick() {
    FlightStateOutput out{};
    FlightIdleData data{};
    out.next_state = FlightState_FLIGHT_STATE_IDLE;
    return std::make_pair(out, data);
}
