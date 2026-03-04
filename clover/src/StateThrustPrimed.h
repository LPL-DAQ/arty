#ifndef APP_STATE_THRUST_PRIMED_H
#define APP_STATE_THRUST_PRIMED_H

#include "Controller.h"

namespace StateThrustPrimed {
    void init();
    std::pair<ControllerOutput, IdleData> tick(uint32_t current_time, uint32_t entry_time, float default_fuel, float default_lox);
}

#endif // APP_STATE_THRUST_PRIMED_H
