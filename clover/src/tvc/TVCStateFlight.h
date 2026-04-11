#ifndef APP_STATE_TVC_FLIGHT_H
#define APP_STATE_TVC_FLIGHT_H

#include "TVCController.h"
#include "clover.pb.h"
#include <utility>

namespace TVCStateFlight {
    void init();
    std::pair<TVCStateOutput, TVCFlightData> tick(const AnalogSensorReadings& analog_sensors);
}

#endif  // APP_STATE_TVC_FLIGHT_H
