#include "RCSRangerModule.h"
#include "../sensors/AnalogSensors.h"
#include "../ControllerConfig.h"
#include "../server.h"
#include "../FlightController.h"

#include "../config.h"
#include <zephyr/kernel.h>
#include <zephyr/kernel/thread_stack.h>
#include <zephyr/logging/log.h>
#include "RCSRangerActuator.h"

LOG_MODULE_REGISTER(RCSRangerModule, LOG_LEVEL_INF);

namespace {
    Trace motor_trace;
    bool motor_has_trace = false;
    float motor_total_time = 0.0f;

    Trace roll_trace;
    bool roll_has_trace = false;
    float roll_total_time = 0.0f;
}


RCSRangerStateOutput RCSRangerModule::step_control_loop(DataPacket& data )
{
    int64_t current_time = k_uptime_get();

    RCSRangerStateOutput out{};

    // --- PROCEDURAL LOGIC DISPATCHER ---
    switch (current_state) {
    case RCSState_RCS_STATE_IDLE: {
        auto [idle_out, idle_data] = idle_tick();
        data.which_rcs_state_data = DataPacket_rcs_idle_data_tag;
        data.rcs_state_data.rcs_idle_data = idle_data;
        out = idle_out;
        break;
    }

    case RCSState_RCS_STATE_VALVE_PRIMED: {
        auto [motor_primed_out, motor_primed_data] = idle_tick();
        motor_primed_out.next_state = RCSState_RCS_STATE_VALVE_PRIMED;
        data.which_rcs_state_data = DataPacket_rcs_idle_data_tag;
        data.rcs_state_data.rcs_idle_data = motor_primed_data;
        out = motor_primed_out;
        break;
    }
    case RCSState_RCS_STATE_VALVE_SEQ: {
        auto [seq_out, seq_data] = motor_sequence_tick(current_time, sequence_start_time);
        data.which_rcs_state_data = DataPacket_rcs_valve_sequence_data_tag;
        data.rcs_state_data.rcs_valve_sequence_data = seq_data;
        out = seq_out;
        break;
    }
    case RCSState_RCS_STATE_ROLL_PRIMED: {
        auto [roll_primed_out, roll_primed_data] = idle_tick();
        roll_primed_out.next_state = RCSState_RCS_STATE_ROLL_PRIMED;
        data.which_rcs_state_data = DataPacket_rcs_idle_data_tag;
        data.rcs_state_data.rcs_idle_data = roll_primed_data;
        out = roll_primed_out;
        break;
    }
    case RCSState_RCS_STATE_ROLL_SEQ: {
        auto [roll_out, roll_data] = roll_sequence_tick(data.analog_sensors, current_time, sequence_start_time);
        data.which_rcs_state_data = DataPacket_rcs_roll_sequence_data_tag;
        data.rcs_state_data.rcs_roll_sequence_data = roll_data;
        out = roll_out;
        break;
    }
    case RCSState_RCS_STATE_FLIGHT: {
        auto [flight_out, flight_data] = flight_tick(data.analog_sensors);
        data.which_rcs_state_data = DataPacket_rcs_flight_data_tag;
        data.rcs_state_data.rcs_flight_data = flight_data;
        out = flight_out;
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

    data.which_rcs_state_output = DataPacket_rcs_ranger_state_output_tag;
    data.rcs_state_output.rcs_ranger_state_output = out;
    data.which_rcs_actuator_data = DataPacket_rcs_ranger_data_tag;
    data.rcs_state = current_state;
    return out;
}



std::expected<void, Error> RCSRangerModule::change_state(RCSState new_state)
{
    if (current_state == new_state)
        return {};
    else if (new_state == RCSState_RCS_STATE_ABORT){
        abort_entry_time = k_uptime_get();
    }

    current_state = new_state;
    LOG_INF("Changed RCS State to %s", get_state_name(current_state));
    return {};
}

std::pair<RCSRangerStateOutput, RCSIdleData> RCSRangerModule::idle_tick()
{
    RCSRangerStateOutput out{};
    RCSIdleData data{};
    out.CW = false;
    out.CCW = false;
    out.next_state = RCSState_RCS_STATE_IDLE;
    return {out, data};
}

std::pair<RCSRangerStateOutput, RCSFlightData> RCSRangerModule::flight_tick(const AnalogSensorReadings& analog_sensors)
{
    RCSRangerStateOutput out{};
    RCSFlightData data{};
    out.next_state = RCSState_RCS_STATE_FLIGHT;
    return {out, data};
}


std::pair<RCSRangerStateOutput, RCSRollSequenceData> RCSRangerModule::roll_sequence_tick(const AnalogSensorReadings& analog_sensors, int64_t current_time, int64_t start_time)
{
    RCSRangerStateOutput out{};
    RCSRollSequenceData data{};
    float dt = current_time - start_time;

    if (roll_has_trace) {
        auto target = roll_trace.sample(dt);
        if (!target) {
            LOG_ERR("Failed to sample roll trace: %s", target.error().build_message().c_str());
            out.next_state = RCSState_RCS_STATE_IDLE;
            out.CW = false;
            out.CCW = false;
            return {out, data};
        }

        float value = *target;
        if (value > 0.0f) {
            out.CW = true;
            out.CCW = false;
        } else if (value < 0.0f) {
            out.CW = false;
            out.CCW = true;
        } else {
            out.CW = false;
            out.CCW = false;
        }
    } else {
        out.CW = false;
        out.CCW = false;
    }

    out.next_state = RCSState_RCS_STATE_ROLL_SEQ;
    if (roll_has_trace && dt >= roll_total_time) {
        LOG_INF("Done RCS roll trace sequence, dt was %f", static_cast<double>(dt));
        out.next_state = RCSState_RCS_STATE_IDLE;
    }

    return {out, data};
}
std::pair<RCSRangerStateOutput, RCSValveSequenceData> RCSRangerModule::motor_sequence_tick(int64_t current_time, int64_t start_time)
{
    RCSRangerStateOutput out{};
    RCSValveSequenceData data{};
    out.CW = false;
    out.CCW = false;
    out.next_state = RCSState_RCS_STATE_VALVE_SEQ;

    float dt = current_time - start_time;

    if (motor_has_trace) {
        auto target = motor_trace.sample(dt);
        if (!target) {
            LOG_ERR("Failed to sample motor trace: %s", target.error().build_message().c_str());
            out.next_state = RCSState_RCS_STATE_IDLE;
            return {out, data};
        }

        float value = *target;
        if (value > 0.0f) {
            out.CW = true;
            out.CCW = false;
        } else if (value < 0.0f) {
            out.CW = false;
            out.CCW = true;
        }
    }

    if (motor_has_trace && dt >= motor_total_time) {
        LOG_INF("Done RCS motor trace sequence, dt was %f", static_cast<double>(dt));
        out.next_state = RCSState_RCS_STATE_IDLE;
    }

    return {out, data};
}


std::pair<RCSRangerStateOutput, RCSAbortData> RCSRangerModule::abort_tick(uint32_t current_time, uint32_t entry_time)
{
    RCSRangerStateOutput out{};
    RCSAbortData data{};

    if (current_time - entry_time > 500) {
        out.next_state = RCSState_RCS_STATE_IDLE;
    } else {
        out.next_state = RCSState_RCS_STATE_ABORT;
    }

    return {out, data};
}


std::expected<void, Error> RCSRangerModule::load_motor_sequence(const RCSLoadValveSequenceRequest& req)
{
    LOG_INF("Received load sequence request");

    auto result = motor_trace.load(req.trace_lbf);
    if (!result)
        return std::unexpected(result.error().context("%s", "Invalid trace"));

    motor_has_trace = true;
    motor_total_time = req.trace_lbf.total_time_ms;

    change_state(RCSState_RCS_STATE_VALVE_PRIMED);

    return {};
}

std::expected<void, Error> RCSRangerModule::start_motor_sequence()
{
    sequence_start_time = k_uptime_get();
    change_state(RCSState_RCS_STATE_VALVE_SEQ);
    return {};
}

std::expected<void, Error> RCSRangerModule::load_roll_sequence(const RCSLoadRollSequenceRequest& req)
{
    LOG_INF("Received load sequence request");

    auto result = roll_trace.load(req.trace_deg);
    if (!result)
        return std::unexpected(result.error().context("%s", "Invalid trace"));

    roll_has_trace = true;
    roll_total_time = req.trace_deg.total_time_ms;

    change_state(RCSState_RCS_STATE_ROLL_PRIMED);

    return {};
}

std::expected<void, Error> RCSRangerModule::start_roll_sequence()
{
    sequence_start_time = k_uptime_get();
    change_state(RCSState_RCS_STATE_ROLL_SEQ);
    return {};
}
const char* RCSRangerModule::get_state_name(RCSState state)
{
    if (state == RCSState_RCS_STATE_IDLE)
        return "Idle";

    if (state == RCSState_RCS_STATE_VALVE_PRIMED)
        return "Valve Primed";
    if (state == RCSState_RCS_STATE_ROLL_PRIMED)
        return "Roll Primed";
    if (state == RCSState_RCS_STATE_VALVE_SEQ)
        return "Valve Sequence";
    if (state == RCSState_RCS_STATE_ROLL_SEQ)
        return "Roll Sequence";
    if (state == RCSState_RCS_STATE_FLIGHT)
        return "Flight";
    if (state == RCSState_RCS_STATE_ABORT)
        return "Abort";
    return "Unknown State";  // Unknown state
}
