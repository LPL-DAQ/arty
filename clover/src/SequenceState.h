#ifndef APP_SEQUENCE_STATE_H
#define APP_SEQUENCE_STATE_H

#include "Controller.h"
#include "Trace.h"

namespace SequenceState {
    void init();
    ControllerOutput tick(uint32_t current_time, uint32_t start_time, Trace& fuel_trace, Trace& lox_trace);
}

#endif // APP_SEQUENCE_STATE_H
