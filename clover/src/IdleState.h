#ifndef APP_IDLE_STATE_H
#define APP_IDLE_STATE_H

#include "Controller.h"

namespace IdleState {
    void init();
    ControllerOutput tick();
}

#endif // APP_IDLE_STATE_H
