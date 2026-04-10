#ifndef APP_STATE_THRUST_SEQ_H
#define APP_STATE_THRUST_SEQ_H

#include "ThrottleController.h"

namespace StateThrustSeq {
    static inline Trace trace;

    void init(float total_time_ms);
    std::pair<ThrottleStateOutput, ThrottleThrustSequenceData> tick(const AnalogSensorReadings& analog_sensors, int64_t current_time, int64_t start_time);


    Trace& get_trace();

}

#endif // APP_STATE_THRUST_SEQ_H
