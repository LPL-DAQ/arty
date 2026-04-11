#include "FlightStateFlightSeq.h"
#include <zephyr/logging/log.h>

LOG_MODULE_DECLARE(FlightController);

static float total_time_ms = 0.0f;
static bool has_trace = false;

static Trace x_trace;
static Trace y_trace;
static Trace z_trace;
static Trace roll_trace;

void FlightStateFlightSeq::init(float new_total_time_ms) {
    total_time_ms = new_total_time_ms;
    has_trace = (new_total_time_ms > 0.0f);

}

std::pair<FlightStateOutput, FlightSequenceData> FlightStateFlightSeq::tick(const AnalogSensorReadings& analog_sensors, int64_t current_time, int64_t start_time) {
    FlightStateOutput out{};
    FlightSequenceData data{};

    float elapsed_time = static_cast<float>(current_time - start_time);
    if (total_time_ms > 0.0f && elapsed_time > total_time_ms) {
        out.next_state = FlightState_FLIGHT_STATE_LANDING;
        return std::make_pair(out, data);
    }

    if (has_trace) {
        auto x_target = x_trace.sample(elapsed_time);
        if (!x_target) {
            LOG_ERR("Flight sequence X trace sampling failed: %s", x_target.error().build_message().c_str());
            out.next_state = FlightState_FLIGHT_STATE_ABORT;
            return std::make_pair(out, data);
        }

        auto y_target = y_trace.sample(elapsed_time);
        if (!y_target) {
            LOG_ERR("Flight sequence Y trace sampling failed: %s", y_target.error().build_message().c_str());
            out.next_state = FlightState_FLIGHT_STATE_ABORT;
            return std::make_pair(out, data);
        }

        auto z_target = z_trace.sample(elapsed_time);
        if (!z_target) {
            LOG_ERR("Flight sequence Z trace sampling failed: %s", z_target.error().build_message().c_str());
            out.next_state = FlightState_FLIGHT_STATE_ABORT;
            return std::make_pair(out, data);
        }

        auto roll_target = roll_trace.sample(elapsed_time);
        if (!roll_target) {
            LOG_ERR("Flight sequence roll trace sampling failed: %s", roll_target.error().build_message().c_str());
            out.next_state = FlightState_FLIGHT_STATE_ABORT;
            return std::make_pair(out, data);
        }

        out.z_acceleration = *z_target;
        out.x_angular_acceleration = *x_target;
        out.y_angular_acceleration = *y_target;
        out.roll_position = *roll_target;


    } else{
        // TODO: Error!!!
    }

    out.next_state = FlightState_FLIGHT_STATE_FLIGHT_SEQ;
    return std::make_pair(out, data);
}

Trace& FlightStateFlightSeq::get_x_trace() {
    return x_trace;
}

Trace& FlightStateFlightSeq::get_y_trace() {
    return y_trace;
}

Trace& FlightStateFlightSeq::get_z_trace() {
    return z_trace;
}

Trace& FlightStateFlightSeq::get_roll_trace() {
    return roll_trace;
}
