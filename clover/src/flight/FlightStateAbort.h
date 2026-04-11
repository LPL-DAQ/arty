#ifndef APP_FLIGHT_STATE_ABORT_H
#define APP_FLIGHT_STATE_ABORT_H

#include "clover.pb.h"
#include <utility>

namespace FlightStateAbort {
    void init();
    std::pair<FlightStateOutput, FlightAbortData> tick(int64_t current_time, int64_t entry_time);
}

#endif  // APP_FLIGHT_STATE_ABORT_H
