#include "sequencer.h"
#include "Trace.h"
#include <algorithm>

static Trace fuel_trace_internal;
static Trace lox_trace_internal;
static uint32_t current_total_steps = 0;

void sequencer_load_from_proto(const LoadMotorSequenceRequest& req) {
    sequencer_reset();

    // Load traces using the Trace class logic for validation and sampling
    if (req.has_fuel_trace) {
        fuel_trace_internal.load(req.fuel_trace);
    }
    if (req.has_lox_trace) {
        lox_trace_internal.load(req.lox_trace);
    }

    // Determine the total duration of the sequence in milliseconds
    current_total_steps = std::max(
        req.has_fuel_trace ? req.fuel_trace.total_time_ms : 0u,
        req.has_lox_trace ? req.lox_trace.total_time_ms : 0u
    );
}

uint32_t sequencer_get_total_steps() {
    return current_total_steps;
}

float sequencer_get_fuel_at(uint32_t step) {
    // Sample using Trace class which handles linear/sine math and clamping
    auto result = fuel_trace_internal.sample(static_cast<float>(step));
    return result.value_or(0.0f);
}

float sequencer_get_lox_at(uint32_t step) {
    auto result = lox_trace_internal.sample(static_cast<float>(step));
    return result.value_or(0.0f);
}

void sequencer_reset() {
    // Re-initialize trace objects to clear state
    fuel_trace_internal = Trace();
    lox_trace_internal = Trace();
    current_total_steps = 0;
}
