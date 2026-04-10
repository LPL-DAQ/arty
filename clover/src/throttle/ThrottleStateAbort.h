#ifndef APP_THROTTLE_STATE_ABORT_H
#define APP_THROTTLE_STATE_ABORT_H

#include "ThrottleController.h"

namespace ThrottleStateAbort {
    void init();
    std::pair<ThrottleStateOutput, ThrottleAbortData> tick(uint32_t current_time, uint32_t entry_time, float default_fuel, float default_lox);
}

#endif // APP_THROTTLE_STATE_ABORT_H
