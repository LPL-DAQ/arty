#ifndef APP_STATE_IDLE_H
#define APP_STATE_IDLE_H

#include "ThrottleController.h"

namespace StateIdle {
    void init();
    std::pair<ThrottleControllerOutput, IdleData> tick();
}

#endif // APP_STATE_IDLE_H
