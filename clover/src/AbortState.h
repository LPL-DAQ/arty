#ifndef APP_ABORT_STATE_H
#define APP_ABORT_STATE_H

#include "Controller.h"

namespace AbortState {
    void init();
    ControllerOutput tick(uint32_t current_time, uint32_t entry_time, float default_fuel, float default_lox);
}

#endif // APP_ABORT_STATE_H
