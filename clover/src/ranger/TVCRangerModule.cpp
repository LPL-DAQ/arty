#include "TVCRangerModule.h"
#include "MutexGuard.h"
#include "../sensors/AnalogSensors.h"
#include "../ControllerConfig.h"
#include "../server.h"

#include "../config.h"
#include <zephyr/kernel.h>
#include <zephyr/kernel/thread_stack.h>
#include <zephyr/logging/log.h>
#include "TVCRangerActuator.h"

LOG_MODULE_REGISTER(TVCRangerModule, LOG_LEVEL_INF);

K_MUTEX_DEFINE(tvc_ranger_module_lock);

namespace {
    Trace pitch_trace;
    Trace yaw_trace;
    bool sequence_has_trace = false;
    float sequence_total_time = 0.0f;
}



void TVCRangerModule::step_control_loop(DataPacket& data )
{
    int64_t current_time = k_uptime_get();

    TVCRangerStateOutput out{};

    TVCState local_state;
    uint32_t local_sequence_start_time;
    uint32_t local_abort_entry_time;
    {
        MutexGuard guard{&tvc_ranger_module_lock};
        local_state = current_state;
        local_sequence_start_time = sequence_start_time;
        local_abort_entry_time = abort_entry_time;
    }

    // --- PROCEDURAL LOGIC DISPATCHER ---
    switch (local_state) {
    case TVCState_TVC_STATE_IDLE: {
        auto [idle_out, idle_data] = idle_tick();
        data.which_tvc_state_data = DataPacket_tvc_idle_data_tag;
        data.tvc_state_data.tvc_idle_data = idle_data;
        out = idle_out;
        break;
    }

    case TVCState_TVC_STATE_TRACE_PRIMED: {
        auto [primed_out, primed_data] = idle_tick();
        primed_out.next_state = TVCState_TVC_STATE_TRACE_PRIMED;
        data.which_tvc_state_data = DataPacket_tvc_idle_data_tag;
        data.tvc_state_data.tvc_idle_data = primed_data;
        out = primed_out;
        break;
    }
    case TVCState_TVC_STATE_FLIGHT: {
        auto [flight_out, flight_data] = flight_tick(data.analog_sensors, data.flight_state_output);
        data.which_tvc_state_data = DataPacket_tvc_flight_data_tag;
        data.tvc_state_data.tvc_flight_data = flight_data;
        out = flight_out;
        break;
    }
    case TVCState_TVC_STATE_TRACE: {
        auto [seq_out, seq_data] = sequence_tick(current_time, local_sequence_start_time);
        data.which_tvc_state_data = DataPacket_tvc_sequence_data_tag;
        data.tvc_state_data.tvc_sequence_data = seq_data;
        out = seq_out;
        break;
    }
    case TVCState_TVC_STATE_ABORT: {
        auto [abort_out, abort_data] = abort_tick(current_time, local_abort_entry_time);
        data.which_tvc_state_data = DataPacket_tvc_abort_data_tag;
        data.tvc_state_data.tvc_abort_data = abort_data;
        out = abort_out;
        break;
    }
    default: {
        auto [idle_out, idle_data] = idle_tick();
        data.which_tvc_state_data = DataPacket_tvc_idle_data_tag;
        data.tvc_state_data.tvc_idle_data = idle_data;
        out = idle_out;
        break;
    }
    }

    // how to pass this upward? it shouldnt change its own state, controller should
    change_state(out.next_state);

    data.which_tvc_state_output = DataPacket_tvc_ranger_state_output_tag;
    data.tvc_state_output.tvc_ranger_state_output = out;
    data.which_tvc_actuator_data = DataPacket_tvc_ranger_data_tag;
    data.tvc_state = state();
}

std::expected<void, Error> TVCRangerModule::change_state(TVCState new_state)
{
    MutexGuard guard{&tvc_ranger_module_lock};
    if (current_state == new_state)
        return {};
    else if (new_state == TVCState_TVC_STATE_ABORT){
        abort_entry_time = k_uptime_get();
    }

    current_state = new_state;
    LOG_INF("Changed TVC State to %s", get_state_name(current_state));
    return {};
}

std::pair<TVCRangerStateOutput, TVCIdleData> TVCRangerModule::idle_tick()
{
    TVCRangerStateOutput out{};
    TVCIdleData data{};
    out.next_state = TVCState_TVC_STATE_IDLE;
    return {out, data};
}

std::pair<TVCRangerStateOutput, TVCFlightData> TVCRangerModule::flight_tick(const AnalogSensorReadings& analog_sensors, FlightStateOutput& flight_output)
{
    TVCRangerStateOutput out{};
    TVCFlightData data{};
    float target_pitch = flight_output.pitch_angular_acceleration;
    float target_yaw = flight_output.yaw_angular_acceleration;
    out.next_state = TVCState_TVC_STATE_FLIGHT;
    out.target_yaw = target_yaw;
    out.target_pitch = target_pitch;
    return {out, data};
}


std::pair<TVCRangerStateOutput, TVCSequenceData> TVCRangerModule::sequence_tick(int64_t current_time, int64_t start_time)
{
    TVCRangerStateOutput out{};
    TVCSequenceData data{};
    out.next_state = TVCState_TVC_STATE_TRACE;

    float dt = current_time - start_time;

    if (sequence_has_trace) {
        auto pitch_target = pitch_trace.sample(dt);
        if (!pitch_target) {
            LOG_ERR("Failed to sample X trace: %s", pitch_target.error().build_message().c_str());
            out.next_state = TVCState_TVC_STATE_IDLE;
            return {out, data};
        }

        auto yaw_target = yaw_trace.sample(dt);
        if (!yaw_target) {
            LOG_ERR("Failed to sample Y trace: %s", yaw_target.error().build_message().c_str());
            out.next_state = TVCState_TVC_STATE_IDLE;
            return {out, data};
        }

        out.target_pitch = *pitch_target;
        out.target_yaw = *yaw_target;
        data.pitch_angle_deg = *pitch_target;
        data.yaw_angle_deg = *yaw_target;
    }

    if (sequence_has_trace && dt >= sequence_total_time) {
        LOG_INF("Done TVC trace sequence, dt was %f", static_cast<double>(dt));
        out.next_state = TVCState_TVC_STATE_IDLE;
    }

    return {out, data};
}

std::pair<TVCRangerStateOutput, TVCAbortData> TVCRangerModule::abort_tick(uint32_t current_time, uint32_t entry_time)
{
    TVCRangerStateOutput out{};
    TVCAbortData data{};

    if (current_time - entry_time > 500) {
        out.next_state = TVCState_TVC_STATE_IDLE;
    } else {
        out.next_state = TVCState_TVC_STATE_ABORT;
    }

    return {out, data};
}


std::expected<void, Error> TVCRangerModule::load_sequence(const TVCLoadSequenceRequest& req)
{
    LOG_INF("Received load sequence request");

    auto result_pitch = pitch_trace.load(req.pitch_angle_trace_deg);
    if (!result_pitch)
        return std::unexpected(result_pitch.error().context("%s", "Invalid Pitch trace"));

    auto result_yaw = yaw_trace.load(req.yaw_angle_trace_deg);
    if (!result_yaw)
        return std::unexpected(result_yaw.error().context("%s", "Invalid Yaw trace"));

    {
        MutexGuard guard{&tvc_ranger_module_lock};
        sequence_has_trace = true;
        sequence_total_time = req.pitch_angle_trace_deg.total_time_ms;
    }

    change_state(TVCState_TVC_STATE_TRACE_PRIMED);

    return {};
}

std::expected<void, Error> TVCRangerModule::start_sequence()
{
    {
        MutexGuard guard{&tvc_ranger_module_lock};
        sequence_start_time = k_uptime_get();
    }
    change_state(TVCState_TVC_STATE_TRACE);
    return {};
}

TVCState TVCRangerModule::state()
{
    MutexGuard guard{&tvc_ranger_module_lock};
    return current_state;
}

const char* TVCRangerModule::get_state_name(TVCState state)
{
    if (state == TVCState_TVC_STATE_IDLE)
        return "Idle";

    if (state == TVCState_TVC_STATE_TRACE_PRIMED)
        return "Trace Primed";
    if (state == TVCState_TVC_STATE_TRACE)
        return "Trace";
    if (state == TVCState_TVC_STATE_FLIGHT)
        return "Flight";
    if (state == TVCState_TVC_STATE_ABORT)
        return "Abort";
    return "Unknown State";  // Unknown state
}
