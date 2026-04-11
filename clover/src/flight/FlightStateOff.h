#ifndef APP_FLIGHT_STATE_OFF_H
#define APP_FLIGHT_STATE_OFF_H

#include "clover.pb.h"
#include <utility>

namespace FlightStateOff {
    void init();
    std::pair<FlightStateOutput, FlightOffData> tick();
}

#endif  // APP_FLIGHT_STATE_OFF_H
