#include "ThrottleStateFlight.h"
#include "../FlightController.h"

void ThrottleStateFlight::init() {
    // Minimal flight state initialization.
}

std::pair<ThrottleStateOutput, ThrottleFlightData> ThrottleStateFlight::tick(const AnalogSensorReadings& analog_sensors) {
    ThrottleStateOutput out{};
    ThrottleFlightData data{};

    // TODO: math to turn accel to thrust
    float target_thrust = FlightController::get_z_acceleration();
    out.has_thrust = true;
    out.thrust = target_thrust;
    out.power_on = true;
    out.next_state = ThrottleState_THROTTLE_STATE_FLIGHT;

    return std::make_pair(out, data);
}
