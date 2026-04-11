#include "FlightStateOff.h"

void FlightStateOff::init() {
    // Flight controller is powered off.
}

std::pair<FlightStateOutput, FlightOffData> FlightStateOff::tick() {
    FlightStateOutput out{};
    FlightOffData data{};
    out.next_state = FlightState_FLIGHT_STATE_OFF;
    return std::make_pair(out, data);
}
