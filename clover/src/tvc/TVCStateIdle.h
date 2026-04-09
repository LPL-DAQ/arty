#ifndef APP_TVC_STATE_IDLE_H
#define APP_TVC_STATE_IDLE_H

#include "TVCController.h"

namespace TVCStateIdle {
    void init();
    std::pair<TVCControllerOutput, TVCIdleData> tick();
}

#endif // APP_TVC_STATE_IDLE_H
