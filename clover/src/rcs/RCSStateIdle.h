#ifndef APP_RCS_STATE_IDLE_H
#define APP_RCS_STATE_IDLE_H

#include "RCSController.h"

namespace RCSStateIdle {
    void init();
    std::pair<RCSStateOutput, RCSIdleData> tick();
}

#endif // APP_RCS_STATE_IDLE_H
