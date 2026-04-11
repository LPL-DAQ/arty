#ifndef APP_STATE_THROTTLE_FLIGHT_H
#define APP_STATE_THROTTLE_FLIGHT_H

#include "ThrottleController.h"
#include "clover.pb.h"
#include <utility>

namespace ThrottleStateFlight {
    void init();
    std::pair<ThrottleStateOutput, ThrottleFlightData> tick(const AnalogSensorReadings& analog_sensors);
}

#endif  // APP_STATE_THROTTLE_FLIGHT_H
