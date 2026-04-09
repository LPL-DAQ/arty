#ifndef APP_STATE_VALVE_SEQ_H
#define APP_STATE_VALVE_SEQ_H

#include "ThrottleController.h"
#include "Trace.h"

namespace StateValveSeq {
    void init(bool has_lox_trace, bool has_fuel_trace, float fuel_total_time_ms, float lox_total_time_ms);
    std::pair<ThrottleControllerOutput, ValveSequenceData> tick(int64_t current_time, int64_t start_time);

    Trace& get_fuel_trace();
    Trace& get_lox_trace();
}

#endif // APP_STATE_VALVE_SEQ_H
