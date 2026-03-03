#ifndef APP_STATE_VALVE_SEQ_H
#define APP_STATE_VALVE_SEQ_H

#include "Controller.h"
#include "Trace.h"

namespace StateValveSeq {
    void init();
     std::pair<ControllerOutput, ValveSequenceData> tick(uint32_t current_time, uint32_t start_time, Trace& fuel_trace, Trace& lox_trace);
}

#endif // APP_STATE_VALVE_SEQ_H
