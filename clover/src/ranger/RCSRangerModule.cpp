#include "RCSRangerModule.h"
#include "../sensors/AnalogSensors.h"
#include "../ControllerConfig.h"
#include "../server.h"

#include "../config.h"
#include <zephyr/kernel.h>
#include <zephyr/kernel/thread_stack.h>
#include <zephyr/logging/log.h>
#include "RCSRangerActuator.h"

LOG_MODULE_REGISTER(RCSModule, LOG_LEVEL_INF);

namespace {
    Trace valve_trace;
    bool valve_has_trace = false;
    float valve_total_time = 0.0f;

    Trace roll_trace;
    bool roll_has_trace = false;
    float roll_total_time = 0.0f;
}


std::expected<void, Error> RCSModule::init()
{
    change_state(RCSState_RCS_STATE_IDLE);

    return {};
}

RCSRangerStateOutput RCSModule::step_control_loop(DataPacket& data )
{
    int64_t current_time = k_uptime_get();

    RCSStateOutput out{};

    // --- PROCEDURAL LOGIC DISPATCHER ---
    switch (current_state) {
    case RCSState_RCS_STATE_IDLE: {
        auto [idle_out, idle_data] = idle_tick();
        data.which_rcs_state_data = DataPacket_rcs_idle_data_tag;
        data.rcs_state_data.rcs_idle_data = idle_data;
        out = idle_out;
        break;
    }

    case RCSState_RCS_STATE_TRACE_PRIMED: {
        auto [primed_out, primed_data] = idle_tick();
        primed_out.next_state = RCSState_RCS_STATE_TRACE_PRIMED;
        data.which_rcs_state_data = DataPacket_rcs_idle_data_tag;
        data.rcs_state_data.rcs_idle_data = primed_data;
        out = primed_out;
        break;
    }
    case RCSState_RCS_STATE_FLIGHT: {
        auto [flight_out, flight_data] = flight_tick(data.analog_sensors);
        data.which_rcs_state_data = DataPacket_rcs_flight_data_tag;
        data.rcs_state_data.rcs_flight_data = flight_data;
        out = flight_out;
        break;
    }
    case RCSState_RCS_STATE_TRACE: {
        auto [seq_out, seq_data] = sequence_tick(current_time, sequence_start_time);
        data.which_rcs_state_data = DataPacket_rcs_sequence_data_tag;
        data.rcs_state_data.rcs_sequence_data = seq_data;
        out = seq_out;
        break;
    }
    case RCSState_RCS_STATE_ABORT: {
        auto [abort_out, abort_data] = abort_tick(current_time, abort_entry_time);
        data.which_rcs_state_data = DataPacket_rcs_abort_data_tag;
        data.rcs_state_data.rcs_abort_data = abort_data;
        out = abort_out;
        break;
    }
    default: {
        auto [idle_out, idle_data] = idle_tick();
        data.which_rcs_state_data = DataPacket_rcs_idle_data_tag;
        data.rcs_state_data.rcs_idle_data = idle_data;
        out = idle_out;
        break;
    }
    }

    // how to pass this upward? it shouldnt change its own state, controller should
    change_state(out.next_state);

    data.which_rcs_actuator_data = DataPacket_rcs_ranger_data_tag;

    data.rcs_state_output = out;
    data.rcs_state = current_state;
}



std::expected<void, Error> RCSModule::change_state(RCSState new_state)
{
    if (current_state == new_state)
        return {};

    current_state = new_state;
    LOG_INF("Changed RCS State to %s", get_state_name(current_state));
    return {};
}

std::pair<RCSStateOutput, RCSIdleData> RCSModule::idle_tick()
{
    RCSStateOutput out{};
    RCSIdleData data{};
    out.next_state = RCSState_RCS_STATE_IDLE;
    return {out, data};
}

std::pair<RCSStateOutput, RCSFlightData> RCSModule::flight_tick(const AnalogSensorReadings& analog_sensors)
{
    RCSStateOutput out{};
    RCSFlightData data{};
    float target_x = FlightController::get_x_angular_acceleration();
    float target_y = FlightController::get_y_angular_acceleration();
    out.next_state = RCSState_RCS_STATE_FLIGHT;
    out.target_x = target_x;
    out.target_y = target_y;
    return {out, data};
}


std::pair<RCSStateOutput, RCSSequenceData> RCSModule::sequence_tick(int64_t current_time, int64_t start_time)
{
    RCSStateOutput out{};
    RCSSequenceData data{};
    out.next_state = RCSState_RCS_STATE_TRACE;

    float dt = current_time - start_time;

    if (sequence_has_trace) {
        auto x_target = x_trace.sample(dt);
        if (!x_target) {
            LOG_ERR("Failed to sample X trace: %s", x_target.error().build_message().c_str());
            out.next_state = RCSState_RCS_STATE_IDLE;
            return {out, data};
        }

        auto y_target = y_trace.sample(dt);
        if (!y_target) {
            LOG_ERR("Failed to sample Y trace: %s", y_target.error().build_message().c_str());
            out.next_state = RCSState_RCS_STATE_IDLE;
            return {out, data};
        }

        out.target_x = *x_target;
        out.target_y = *y_target;
        data.x_angle_deg = *x_target;
        data.y_angle_deg = *y_target;
    }

    if (sequence_has_trace && dt >= sequence_total_time) {
        LOG_INF("Done RCS trace sequence, dt was %f", static_cast<double>(dt));
        out.next_state = RCSState_RCS_STATE_IDLE;
    }

    return {out, data};
}

std::pair<RCSStateOutput, RCSAbortData> RCSModule::abort_tick(uint32_t current_time, uint32_t entry_time)
{
    RCSStateOutput out{};
    RCSAbortData data{};

    if (current_time - entry_time > 500) {
        out.next_state = RCSState_RCS_STATE_IDLE;
    } else {
        out.next_state = RCSState_RCS_STATE_ABORT;
    }

    return {out, data};
}


std::expected<void, Error> RCSModule::load_sequence()
{
    LOG_INF("Received load sequence request");

    auto result_x = x_trace.load(req.x_angle_trace_deg);
    if (!result_x)
        return std::unexpected(result_x.error().context("%s", "Invalid X trace"));

    auto result_y = y_trace.load(req.y_angle_trace_deg);
    if (!result_y)
        return std::unexpected(result_y.error().context("%s", "Invalid Y trace"));

    sequence_has_trace = true;
    sequence_total_time = req.x_angle_trace_deg.total_time_ms;

    change_state(RCSState_RCS_STATE_TRACE_PRIMED);

    return {};
}

std::expected<void, Error> RCSModule::start_sequence()
{
    sequence_start_time = k_uptime_get();
    change_state(RCSState_RCS_STATE_TRACE);
    return {};
}

const char* RCSModule::get_state_name(RCSState state)
{
    if (state == RCSState_RCS_STATE_IDLE)
        return "Idle";

    if (state == RCSState_RCS_STATE_TRACE_PRIMED)
        return "Trace Primed";
    if (state == RCSState_RCS_STATE_TRACE)
        return "Trace";
    if (state == RCSState_RCS_STATE_OFF)
        return "Off";
    if (state == RCSState_RCS_STATE_FLIGHT)
        return "Flight";
    if (state == RCSState_RCS_STATE_ABORT)
        return "Abort";
    return "Unknown State";  // Unknown state
}
