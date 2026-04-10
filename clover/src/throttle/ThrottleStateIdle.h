#ifndef APP_THROTTLE_STATE_IDLE_H
#define APP_THROTTLE_STATE_IDLE_H

#include "ThrottleController.h"

namespace ThrottleStateIdle {
    void init();
    std::pair<ThrottleStateOutput, ThrottleIdleData> tick();
}

#endif // APP_THROTTLE_STATE_IDLE_H
