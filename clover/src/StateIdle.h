#ifndef APP_STATE_IDLE_H
#define APP_STATE_IDLE_H

#include "Controller.h"

namespace IdleState {
    void init();
    std::pair<ControllerOutput, IdleData> tick();
}

#endif // APP_STATE_IDLE_H
