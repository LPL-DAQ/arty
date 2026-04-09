#ifndef APP_STATE_ABORT_H
#define APP_STATE_ABORT_H

#include "ThrottleController.h"

namespace StateAbort {
    void init();
    std::pair<ThrottleControllerOutput, AbortData> tick(uint32_t current_time, uint32_t entry_time, float default_fuel, float default_lox);
}

#endif // APP_STATE_ABORT_H
