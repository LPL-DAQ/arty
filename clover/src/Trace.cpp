#include "Trace.h"

#include <algorithm>
#include <cmath>
#include <numbers>

/// The transition between two segments must be within EPSILON of each other to be considered valid.
static constexpr float EPSILON = 0.00001f;

static float sample_segment(const Segment& segment, const float segment_time_ms);

Trace::Trace() : has_valid_trace {false}, last_used_segment_index {0}
{
    control_trace = ControlTrace_init_default;
}

/// Attempt to load a trace by checking that it's valid, then copying it into Trace. A valid trace is one where each
/// segment follows right after the next, and where there are no sudden jumps in its values.
std::expected<void, Error> Trace::load(const ControlTrace& trace)
{
    // Validate trace.
    if (trace.total_time_ms == 0) {
        return std::unexpected(Error::from_cause("total time must be greater than zero"));
    }
    if (trace.segments_count == 0) {
        return std::unexpected(Error::from_cause("no segments specified"));
    }

    uint32_t prev_end = 0;
    float left_segment_sample = 0.0f;
    for (int i = 0; i < trace.segments_count; i++) {
        const Segment& seg = trace.segments[i];
        switch (seg.which_type) {
        case Segment_sine_tag:
            if (seg.type.sine.amplitude < 0.0f) {
                return std::unexpected(Error::from_cause("amplitude for sine segment %d must be non-negative, got %f", i+1, static_cast<double>(seg.type.sine.amplitude)));
            }
            if (seg.type.sine.period <= 0.0f) {
                return std::unexpected(Error::from_cause("period for sine segment %d must be positive, got %f", i+1, static_cast<double>(seg.type.sine.period)));
            }
            break;
        case Segment_linear_tag:
            break;
        default:
            return std::unexpected(Error::from_cause("segment %d has invalid type, got %d", i+1, seg.which_type));
        }

        if (seg.start_ms != prev_end) {
            return std::unexpected(Error::from_cause("segment %d's start time is not continuous with previous segment (expected %d ms)", i+1, prev_end));
        }
        if (seg.length_ms == 0) {
            return std::unexpected(Error::from_cause("segment %d's length may not be zero", i+1));
        }

        // Ensure trace continuity.
        if (i > 0) {
            float right_segment_sample = sample_segment(seg, 0);
            if (std::abs(right_segment_sample - left_segment_sample) > EPSILON) {
                return std::unexpected(Error::from_cause("segment %d's start is discontinuous, previous ended at %f but this one started at %f", i+1, static_cast<double>(left_segment_sample), static_cast<double>(right_segment_sample)));
            }
        }
        left_segment_sample = sample_segment(seg, seg.length_ms);

        prev_end += seg.length_ms;
    }
    if (trace.total_time_ms != prev_end) {
        return std::unexpected(Error::from_cause("total time is incorrect, expected %d ms", prev_end));
    }

    control_trace = trace;
    has_valid_trace = true;
    last_used_segment_index = 0;

    return {};
}

/// Sample from the current trace. Errors only if no valid trace was loaded beforehand. If time is out of the bounds,
/// the input will be clamped to the nearest valid value. In other words, negative values will return as though we
/// sampled with time=0, and overly large values will return as though we sampled with
/// time_ms=control_trace.total_time_ms.
std::expected<float, Error> Trace::sample(float time_ms)
{
    if (!has_valid_trace) {
        return std::unexpected(Error::from_cause("valid trace not loaded"));
    }

    // Let's avoid floating point rounding hell by picking what segment we're in based on the truncated time.
    auto time_ms_trunc = std::clamp(static_cast<uint32_t>(std::floor(time_ms)), 0u, control_trace.total_time_ms-1);

    // We are almost certainly sampling forwards in time, so we should search for the matching segment starting from the
    // last used segment.
    for (int i = 0; i < control_trace.segments_count; i++) {
        int index = (last_used_segment_index + i) % control_trace.segments_count;
        const Segment& seg = control_trace.segments[index];
        if (time_ms_trunc >= seg.start_ms && time_ms_trunc < seg.start_ms + seg.length_ms) {
            last_used_segment_index = index;
            return sample_segment(seg, time_ms - static_cast<float>(seg.start_ms));
        }
    }

    // This should be unreachable if the trace is truly valid.
    return std::unexpected(Error::from_cause("failed to find matching segment"));
}

/// Samples within a segment. segment must be valid. segment_time must be the time from the start of the segment, not
/// necessarily the time starting from the beginning of the trace.
static float sample_segment(const Segment& segment, const float segment_time_ms)
{
    switch (segment.which_type) {
        case Segment_linear_tag: {
            const LinearSegment& lin = segment.type.linear;
            return lin.start_val + (lin.end_val - lin.start_val) * (segment_time_ms / static_cast<float>(segment.length_ms));
        }

        case Segment_sine_tag: {
            constexpr float TAU = 2.0f * std::numbers::pi_v<float>;

            const SineSegment& sine = segment.type.sine;
            return sine.offset + sine.amplitude * std::sin(segment_time_ms / sine.period * TAU + sine.phase_deg / 360.0f * TAU);
        }

        default: {
            // Should be unreachable if segment is valid.
            return 0.0f;
        }
    }
}
