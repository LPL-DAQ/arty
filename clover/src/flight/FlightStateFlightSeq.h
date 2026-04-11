#ifndef APP_FLIGHT_STATE_FLIGHTSEQ_H
#define APP_FLIGHT_STATE_FLIGHTSEQ_H

#include "clover.pb.h"
#include "Trace.h"
#include <utility>

namespace FlightStateFlightSeq {
    static inline Trace trace;
    void init(float total_time_ms);
    std::pair<FlightStateOutput, FlightSequenceData> tick(const AnalogSensorReadings& analog_sensors, int64_t current_time, int64_t start_time);
    Trace& get_trace();
}

#endif  // APP_FLIGHT_STATE_FLIGHTSEQ_H
