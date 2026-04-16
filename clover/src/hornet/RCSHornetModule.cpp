#include "RCSHornetModule.h"
#include "MutexGuard.h"
#include "../sensors/AnalogSensors.h"
#include "../ControllerConfig.h"
#include "../server.h"

#include "../config.h"
#include <zephyr/kernel.h>
#include <zephyr/kernel/thread_stack.h>
#include <zephyr/logging/log.h>
#include "RCSHornetActuator.h"

LOG_MODULE_REGISTER(RCSHornetModule, LOG_LEVEL_INF);

K_MUTEX_DEFINE(rcs_hornet_module_lock);

namespace {
    Trace valve_trace;
    bool valve_has_trace = false;
    float valve_total_time = 0.0f;

    Trace roll_trace;
    bool roll_has_trace = false;
    float roll_total_time = 0.0f;
}

void RCSHornetModule::step_control_loop(DataPacket& data )
{
    int64_t current_time = k_uptime_get();

    RCSState local_state;
    uint32_t local_sequence_start_time;
    uint32_t local_abort_entry_time;
    {
        MutexGuard guard{&rcs_hornet_module_lock};
        local_state = current_state;
        local_sequence_start_time = sequence_start_time;
        local_abort_entry_time = abort_entry_time;
    }

    RCSHornetStateOutput out{};

    // --- PROCEDURAL LOGIC DISPATCHER ---
    switch (local_state) {
    case RCSState_RCS_STATE_IDLE: {
        auto [idle_out, idle_data] = idle_tick();
        data.which_rcs_state_data = DataPacket_rcs_idle_data_tag;
        data.rcs_state_data.rcs_idle_data = idle_data;
        out = idle_out;
        break;
    }

    case RCSState_RCS_STATE_VALVE_PRIMED: {
        auto [valve_primed_out, valve_primed_data] = idle_tick();
        valve_primed_out.next_state = RCSState_RCS_STATE_VALVE_PRIMED;
        data.which_rcs_state_data = DataPacket_rcs_idle_data_tag;
        data.rcs_state_data.rcs_idle_data = valve_primed_data;
        out = valve_primed_out;
        break;
    }
    case RCSState_RCS_STATE_VALVE_SEQ: {
        auto [seq_out, seq_data] = valve_sequence_tick(current_time, local_sequence_start_time);
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
        auto [roll_out, roll_data] = roll_sequence_tick(data.analog_sensors, current_time, local_sequence_start_time);
        data.which_rcs_state_data = DataPacket_rcs_roll_sequence_data_tag;
        data.rcs_state_data.rcs_roll_sequence_data = roll_data;
        out = roll_out;
        break;
    }
    case RCSState_RCS_STATE_FLIGHT: {
        auto [flight_out, flight_data] = flight_tick(data.analog_sensors, data.flight_state_output.roll_position);
        data.which_rcs_state_data = DataPacket_rcs_flight_data_tag;
        data.rcs_state_data.rcs_flight_data = flight_data;
        out = flight_out;
        break;
    }
    case RCSState_RCS_STATE_ABORT: {
        auto [abort_out, abort_data] = abort_tick(current_time, local_abort_entry_time);
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

    data.which_rcs_state_output = DataPacket_rcs_hornet_state_output_tag;
    data.rcs_state_output.rcs_hornet_state_output = out;
    data.which_rcs_actuator_data = DataPacket_rcs_hornet_data_tag;
    data.rcs_state = state();
}



std::expected<void, Error> RCSHornetModule::change_state(RCSState new_state)
{
    MutexGuard guard{&rcs_hornet_module_lock};
    if (current_state == new_state)
        return {};
    else if (new_state == RCSState_RCS_STATE_ABORT){
        abort_entry_time = k_uptime_get();
    }

    current_state = new_state;
    LOG_INF("Changed RCS State to %s", get_state_name(current_state));
    return {};
}

RCSState RCSHornetModule::state()
{
    MutexGuard guard{&rcs_hornet_module_lock};
    return current_state;
}

std::pair<RCSHornetStateOutput, RCSIdleData> RCSHornetModule::idle_tick()
{
    RCSHornetStateOutput out{};
    RCSIdleData data{};
    out.CW = false;
    out.CCW = false;
    out.next_state = RCSState_RCS_STATE_IDLE;
    return {out, data};
}

std::pair<RCSHornetStateOutput, RCSFlightData> RCSHornetModule::flight_tick(const AnalogSensorReadings& analog_sensors, float roll_position)
{
    RCSHornetStateOutput out{};
    RCSFlightData data{};
    // Roll control based on position goes here
    out.CW = false;
    out.CCW = false;
    out.next_state = RCSState_RCS_STATE_FLIGHT;
    return {out, data};
}

// so roll control would need to be in here which means it's duped between hornet/ranger, but that's okay i guess
std::pair<RCSHornetStateOutput, RCSRollSequenceData> RCSHornetModule::roll_sequence_tick(const AnalogSensorReadings& analog_sensors, int64_t current_time, int64_t start_time)
{
    RCSHornetStateOutput out{};
    RCSRollSequenceData data{};
    float dt = current_time - start_time;
    bool local_roll_has_trace;
    float local_roll_total_time;
    {
        MutexGuard guard{&rcs_hornet_module_lock};
        local_roll_has_trace = roll_has_trace;
        local_roll_total_time = roll_total_time;
    }

    if (local_roll_has_trace) {
        auto target = roll_trace.sample(dt);
        if (!target) {
            LOG_ERR("Failed to sample roll trace: %s", target.error().build_message().c_str());
            out.next_state = RCSState_RCS_STATE_IDLE;
            out.CW = false;
            out.CCW = false;
            return {out, data};
        }

        float roll_position = *target;
        auto [control_out, control_data] = flight_tick(analog_sensors, roll_position);
        out.CW = control_out.CW;
        out.CCW = control_out.CCW;

    } else {
        out.CW = false;
        out.CCW = false;
    }

    out.next_state = RCSState_RCS_STATE_ROLL_SEQ;
    if (local_roll_has_trace && dt >= local_roll_total_time) {
        LOG_INF("Done RCS roll trace sequence, dt was %f", static_cast<double>(dt));
        out.next_state = RCSState_RCS_STATE_IDLE;
    }

    return {out, data};
}
std::pair<RCSHornetStateOutput, RCSValveSequenceData> RCSHornetModule::valve_sequence_tick(int64_t current_time, int64_t start_time)
{
    RCSHornetStateOutput out{};
    RCSValveSequenceData data{};
    out.CW = false;
    out.CCW = false;
    out.next_state = RCSState_RCS_STATE_VALVE_SEQ;

    float dt = current_time - start_time;

    bool local_valve_has_trace;
    float local_valve_total_time;
    {
        MutexGuard guard{&rcs_hornet_module_lock};
        local_valve_has_trace = valve_has_trace;
        local_valve_total_time = valve_total_time;
    }

    if (local_valve_has_trace) {
        auto target = valve_trace.sample(dt);
        if (!target) {
            LOG_ERR("Failed to sample valve trace: %s", target.error().build_message().c_str());
            out.next_state = RCSState_RCS_STATE_IDLE;
            return {out, data};
        }

        // + for CW, - for CCW, 0 for none
        float value = *target;
        if (value > 0.0f) {
            out.CW = true;
            out.CCW = false;
        } else if (value < 0.0f) {
            out.CW = false;
            out.CCW = true;
        }else {
            out.CW = false;
            out.CCW = false;
        }
    } else {
        out.CW = false;
        out.CCW = false;
    }

    if (local_valve_has_trace && dt >= local_valve_total_time) {
        LOG_INF("Done RCS valve trace sequence, dt was %f", static_cast<double>(dt));
        out.next_state = RCSState_RCS_STATE_IDLE;
    }

    return {out, data};
}


std::pair<RCSHornetStateOutput, RCSAbortData> RCSHornetModule::abort_tick(uint32_t current_time, uint32_t entry_time)
{
    RCSHornetStateOutput out{};
    RCSAbortData data{};

    if (current_time - entry_time > 500) {
        out.next_state = RCSState_RCS_STATE_IDLE;
    } else {
        out.next_state = RCSState_RCS_STATE_ABORT;
    }

    return {out, data};
}


std::expected<void, Error> RCSHornetModule::load_valve_sequence(const RCSLoadValveSequenceRequest& req)
{
    LOG_INF("Received load sequence request");

    auto result = valve_trace.load(req.trace_lbf);
    if (!result)
        return std::unexpected(result.error().context("%s", "Invalid trace"));

    {
        MutexGuard guard{&rcs_hornet_module_lock};
        valve_has_trace = true;
        valve_total_time = req.trace_lbf.total_time_ms;
    }

    change_state(RCSState_RCS_STATE_VALVE_PRIMED);

    return {};
}

std::expected<void, Error> RCSHornetModule::start_valve_sequence()
{
    {
        MutexGuard guard{&rcs_hornet_module_lock};
        sequence_start_time = k_uptime_get();
    }
    change_state(RCSState_RCS_STATE_VALVE_SEQ);
    return {};
}

std::expected<void, Error> RCSHornetModule::load_roll_sequence(const RCSLoadRollSequenceRequest& req)
{
    LOG_INF("Received load sequence request");

    auto result = roll_trace.load(req.trace_deg);
    if (!result)
        return std::unexpected(result.error().context("%s", "Invalid trace"));

    {
        MutexGuard guard{&rcs_hornet_module_lock};
        roll_has_trace = true;
        roll_total_time = req.trace_deg.total_time_ms;
    }

    change_state(RCSState_RCS_STATE_ROLL_PRIMED);

    return {};
}

std::expected<void, Error> RCSHornetModule::start_roll_sequence()
{
    {
        MutexGuard guard{&rcs_hornet_module_lock};
        sequence_start_time = k_uptime_get();
    }
    change_state(RCSState_RCS_STATE_ROLL_SEQ);
    return {};
}
const char* RCSHornetModule::get_state_name(RCSState state)
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
