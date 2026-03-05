#ifndef APP_STATE_VALVE_SEQ_H
#define APP_STATE_VALVE_SEQ_H

#include "Controller.h"
#include "Trace.h"

namespace StateValveSeq {
    static inline Trace fuel_trace;
    static inline Trace lox_trace;
    static bool has_fuel;
    static bool has_lox;
    static float fuel_total_time;
    static float lox_total_time;

    void init(bool has_lox_trace, bool has_fuel_trace, float fuel_total_time, float lox_total_time);
    std::pair<ControllerOutput, ValveSequenceData> tick(uint32_t current_time, uint32_t start_time);

    Trace& get_fuel_trace();
    Trace& get_lox_trace();
}

#endif // APP_STATE_VALVE_SEQ_H
