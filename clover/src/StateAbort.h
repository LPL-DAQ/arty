#ifndef APP_STATE_ABORT_H
#define APP_STATE_ABORT_H

#include "Controller.h"

namespace StateAbort {
    void init();
    std::pair<ControllerOutput, AbortData> tick(uint32_t current_time, uint32_t entry_time, float default_fuel, float default_lox);
}

#endif // APP_STATE_ABORT_H
