#ifndef APP_FLIGHT_STATE_FLIGHTSEQ_H
#define APP_FLIGHT_STATE_FLIGHTSEQ_H

#include "clover.pb.h"
#include "Trace.h"
#include <utility>

namespace FlightStateFlightSeq {


    void init(float total_time_ms);
    std::pair<FlightStateOutput, FlightSequenceData> tick(const AnalogSensorReadings& analog_sensors, int64_t current_time, int64_t start_time);
    Trace& get_x_trace();
    Trace& get_y_trace();
    Trace& get_z_trace();
    Trace& get_roll_trace();
}

#endif  // APP_FLIGHT_STATE_FLIGHTSEQ_H
