#ifndef APP_RCS_STATE_ABORT_H
#define APP_RCS_STATE_ABORT_H

#include "RCSController.h"

namespace RCSStateAbort {
    void init();
    std::pair<RCSControllerOutput, RCSAbortData> tick(uint32_t current_time, uint32_t entry_time);
}

#endif // APP_RCS_STATE_ABORT_H
