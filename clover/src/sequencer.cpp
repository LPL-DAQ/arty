#include "sequencer.h"
#include <cmath>
#include <algorithm>

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

std::vector<float> fuel_breakpoints, lox_breakpoints;
volatile int step_count = 0;
volatile int count_to = 0;
uint64_t start_clock = 0;

float get_trace_val(const ControlTrace& trace, uint32_t ms) {
    for (size_t i = 0; i < (int)trace.segments_count; i++) {
        const auto& s = trace.segments[i];
        if (ms >= s.start_ms && ms < (s.start_ms + s.length_ms)) {
            float p = static_cast<float>(ms - s.start_ms) / static_cast<float>(s.length_ms);
            if (s.which_type == Segment_linear_tag) {
                return s.type.linear.start_val + (s.type.linear.end_val - s.type.linear.start_val) * p;
            } else if (s.which_type == Segment_sine_tag) {
                float t = static_cast<float>(ms) / 1000.0f;
                float freq = 1.0f / (s.type.sine.period_ms / 1000.0f);
                // Use explicit float math to avoid double-promotion warnings
                return s.type.sine.offset + s.type.sine.amplitude * sinf(2.0f * static_cast<float>(M_PI) * freq * t + 
                       (s.type.sine.phase_deg * static_cast<float>(M_PI) / 180.0f));
            }
        }
    }
    return 0.0f;
}

int sequencer_load_from_proto(const LoadMotorSequenceRequest& req) {
    fuel_breakpoints.clear();
    lox_breakpoints.clear();
    if (req.has_fuel_trace) {
        count_to = req.fuel_trace.total_time_ms;
        for (uint32_t t = 0; t <= static_cast<uint32_t>(count_to); t++) {
            fuel_breakpoints.push_back(get_trace_val(req.fuel_trace, t));
            lox_breakpoints.push_back(req.has_lox_trace ? get_trace_val(req.lox_trace, t) : 0.0f);
        }
    }
    return 0;
}