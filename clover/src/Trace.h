#ifndef APP_TRACE_H
#define APP_TRACE_H

#include "Error.h"
#include "clover.pb.h"

#include <expected>

class Trace {
private:
    ControlTrace control_trace;
    bool has_valid_trace;
    int last_used_segment_index;

public:
    Trace();

    std::expected<void, Error> load(const ControlTrace& trace);
    std::expected<float, Error> sample(float time);
};

#endif  // APP_TRACE_H
