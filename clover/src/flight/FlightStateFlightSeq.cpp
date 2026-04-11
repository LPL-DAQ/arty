#include "FlightStateFlightSeq.h"
#include <zephyr/logging/log.h>

LOG_MODULE_DECLARE(FlightController);

static float total_time_ms = 0.0f;

void FlightStateFlightSeq::init(float new_total_time_ms) {
    total_time_ms = new_total_time_ms;
    trace = Trace();
}

std::pair<FlightStateOutput, FlightSequenceData> FlightStateFlightSeq::tick(const AnalogSensorReadings& analog_sensors, int64_t current_time, int64_t start_time) {
    FlightStateOutput out{};
    FlightSequenceData data{};

    float elapsed_time = static_cast<float>(current_time - start_time);
    if (total_time_ms > 0.0f && elapsed_time > total_time_ms) {
        out.next_state = FlightState_FLIGHT_STATE_LANDING;
        return std::make_pair(out, data);
    }

    auto target_result = trace.sample(elapsed_time);
    if (!target_result) {
        LOG_ERR("Flight sequence trace sampling failed: %s", target_result.error().build_message().c_str());
        out.next_state = FlightState_FLIGHT_STATE_ABORT;
        return std::make_pair(out, data);
    }

    out.next_state = FlightState_FLIGHT_STATE_FLIGHT_SEQ;
    return std::make_pair(out, data);
}

Trace& FlightStateFlightSeq::get_trace() {
    return trace;
}
