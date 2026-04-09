#ifndef APP_TVC_STATE_ABORT_H
#define APP_TVC_STATE_ABORT_H

#include "TVCController.h"

namespace TVCStateAbort {
    void init();
    std::pair<TVCControllerOutput, TVCAbortData> tick(uint32_t current_time, uint32_t entry_time);
}

#endif // APP_TVC_STATE_ABORT_H
