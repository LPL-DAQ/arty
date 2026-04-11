#include "TVCStateFlight.h"
#include "flight/FlightController.h"

void TVCStateFlight::init() {
    // Minimal flight state initialization.
}

std::pair<TVCStateOutput, TVCFlightData> TVCStateFlight::tick(const AnalogSensorReadings& analog_sensors) {
    TVCStateOutput out{};
    TVCFlightData data{};

    // TODO: math to turn accel to target angles
    float target_x = FlightController::get_x_angular_acceleration();
    float target_y = FlightController::get_y_angular_acceleration();
    out.next_state = TVCState_TVC_STATE_FLIGHT;
    // TODO: actually output the target angles
    return std::make_pair(out, data);
}
