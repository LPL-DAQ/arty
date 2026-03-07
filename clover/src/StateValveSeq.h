#ifndef APP_STATE_VALVE_SEQ_H
#define APP_STATE_VALVE_SEQ_H

#include "Controller.h"
#include "Trace.h"

namespace StateValveSeq {
    void init(bool has_lox_trace, bool has_fuel_trace, float fuel_total_time, float lox_total_time);
    std::pair<ControllerOutput, ValveSequenceData> tick(uint32_t current_time, uint32_t start_time);

    Trace& get_fuel_trace();
    Trace& get_lox_trace();
}

#endif // APP_STATE_VALVE_SEQ_H
