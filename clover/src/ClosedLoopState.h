#ifndef APP_CLOSED_LOOP_STATE_H
#define APP_CLOSED_LOOP_STATE_H

#include "Controller.h"

namespace ClosedLoopState {
    void init();
    ControllerOutput tick(bool has_ptc, float ptc_pressure);
}

#endif // APP_CLOSED_LOOP_STATE_H
