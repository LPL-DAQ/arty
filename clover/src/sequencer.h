#ifndef ARTY_SEQUENCER_H
#define ARTY_SEQUENCER_H

#include <vector>
#include <expected>
#include <zephyr/kernel.h>
#include "clover.pb.h"
#include "Error.h"

// Shared variables physically defined in sequencer.cpp
extern std::vector<float> fuel_breakpoints, lox_breakpoints;
extern volatile int step_count, count_to;
extern uint64_t start_clock;

// FIX: Wrap the Zephyr queue in extern "C" to match the macro definition
extern "C" {
    extern struct k_msgq control_data_msgq;
}

int sequencer_load_from_proto(const LoadMotorSequenceRequest& req);
float get_trace_val(const ControlTrace& trace, uint32_t ms);

#endif