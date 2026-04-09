#ifndef APP_THROTTLE_STATE_IDLE_H
#define APP_THROTTLE_STATE_IDLE_H

#include "ThrottleController.h"

namespace ThrottleStateIdle {
    void init();
    std::pair<ThrottleControllerOutput, ThrottleIdleData> tick();
}

#endif // APP_THROTTLE_STATE_IDLE_H
