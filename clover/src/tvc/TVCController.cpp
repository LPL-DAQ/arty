#include "TVCController.h"
#include "../sensors/AnalogSensors.h"
#include "../ControllerConfig.h"
#include "TVCStateAbort.h"
#include "TVCStateIdle.h"
#include "TVCStateValveSeq.h"
#include "../server.h"

#include "../config.h"
#include <zephyr/kernel.h>
#include <zephyr/kernel/thread_stack.h>
#include <zephyr/logging/log.h>

LOG_MODULE_REGISTER(TVCController, LOG_LEVEL_INF);


std::expected<void, Error> TVCController::change_state(TVCState new_state)
{
    if (current_state == new_state)
        return {};

    switch (new_state) {
    case TVCState_TVC_STATE_IDLE:
        current_state = new_state;
        TVCStateIdle::init();

        break;

    case TVCState_TVC_STATE_VALVE_PRIMED:
        if (current_state != TVCState_TVC_STATE_IDLE) {
            return std::unexpected(Error::from_cause("Cannot switch from %s to Valve Primed, must be in Idle", get_state_name(current_state)));
        }
        TVCStateIdle::init();
        current_state = new_state;
        break;

    case TVCState_TVC_STATE_VALVE_SEQ:
        if (current_state != TVCState_TVC_STATE_VALVE_PRIMED) {
            return std::unexpected(Error::from_cause("Cannot switch from %s to Valve Seq, must be in Valve Primed", get_state_name(current_state)));
        }

        // INIT IS IN THE HANDLER
        current_state = new_state;
        break;

    case TVCState_TVC_STATE_PITCH_PRIMED:
        if (current_state != TVCState_TVC_STATE_IDLE) {
            return std::unexpected(Error::from_cause("Cannot switch from %s to Pitch Primed, must be in Idle", get_state_name(current_state)));
        }
        TVCStateIdle::init();
        current_state = new_state;
        break;

    case TVCState_TVC_STATE_PITCH_SEQ:
        if (current_state != TVCState_TVC_STATE_PITCH_PRIMED) {
            return std::unexpected(Error::from_cause("Cannot switch from %s to Pitch Seq, must be in Pitch Primed", get_state_name(current_state)));
        }
        // INIT IS IN THE HANDLER
        current_state = new_state;
        break;

    case TVCState_TVC_STATE_ABORT:
        if (current_state != TVCState_TVC_STATE_PITCH_SEQ && current_state != TVCState_TVC_STATE_VALVE_SEQ) {
            return std::unexpected(Error::from_cause("Cannot switch from %s to Abort, must be in Valve Seq or Pitch Seq", get_state_name(current_state)));
        }
        TVCStateAbort::init();
        current_state = new_state;
        break;
    default:
        break;
    }
    LOG_INF("Changed State to %s", get_state_name(current_state));

    return {};
}



std::expected<void, Error> TVCController::init()
{
    LOG_INF("Triggering initial sensor readings");
    k_sched_lock();
    AnalogSensors::start_sense();
    // Other sensors here...
    k_sched_unlock();

    LOG_INF("Pausing for initial sensor readings to complete");
    k_sleep(K_MSEC(500));

    auto ret = change_state(TVCState_TVC_STATE_IDLE);
    if (!ret.has_value()) {
        return std::unexpected(ret.error().context("Failed to change state to idle"));
    }
    return {};
}

static int step_control_loop_debounce_warn_count = 0;

void TVCController::step_control_loop(std::optional<std::pair<AnalogSensorReadings, float>> analog_sensors_readings )
{
    int64_t current_time = k_uptime_get();
    DataPacket data = DataPacket_init_default;

    TVCControllerOutput out;

    // --- PROCEDURAL LOGIC DISPATCHER ---
    switch (current_state) {
    case TVCState_TVC_STATE_IDLE: {
        auto [idle_out, idle_data] = TVCStateIdle::tick();
        data.which_rcs_state_data = DataPacket_rcs_idle_data_tag;
        data.rcs_state_data.rcs_idle_data = idle_data;
        out = idle_out;
        break;
    }

    case TVCState_TVC_STATE_VALVE_PRIMED: {
        auto [primed_out, primed_data] = TVCStateIdle::tick();
        primed_out.next_state = TVCState_TVC_STATE_VALVE_PRIMED;
        data.which_rcs_state_data = DataPacket_rcs_idle_data_tag;
        data.rcs_state_data.rcs_idle_data = primed_data;
        out = primed_out;
        break;
    }
    case TVCState_TVC_STATE_VALVE_SEQ: {
        auto [seq_out, seq_data] = TVCStateValveSeq::tick(current_time, sequence_start_time);
        data.which_rcs_state_data = DataPacket_rcs_valve_sequence_data_tag;
        data.rcs_state_data.rcs_valve_sequence_data = seq_data;
        out = seq_out;
        break;
    }
    case TVCState_TVC_STATE_PITCH_PRIMED: {
        auto [roll_primed_out, roll_primed_data] = TVCStateIdle::tick();
        roll_primed_out.next_state = TVCState_TVC_STATE_PITCH_PRIMED;
        data.which_rcs_state_data = DataPacket_rcs_idle_data_tag;
        data.rcs_state_data.rcs_idle_data = roll_primed_data;
        out = roll_primed_out;
        break;
    }
    case TVCState_TVC_STATE_PITCH_SEQ: {
        auto [roll_out, roll_data] = TVCStateValveSeq::tick(current_time, sequence_start_time);
        data.which_rcs_state_data = DataPacket_rcs_valve_sequence_data_tag;
        data.rcs_state_data.rcs_valve_sequence_data = roll_data;
        out = roll_out;
        break;
    }
    case TVCState_TVC_STATE_ABORT: {
        auto [abort_out, abort_data] = TVCStateAbort::tick(current_time, abort_entry_time);
        data.which_rcs_state_data = DataPacket_rcs_abort_data_tag;
        data.rcs_state_data.rcs_abort_data = abort_data;
        out = abort_out;
        break;
    }
    default: {
        auto [idle_out, idle_data] = TVCStateIdle::tick();
        data.which_rcs_state_data = DataPacket_rcs_idle_data_tag;
        data.rcs_state_data.rcs_idle_data = idle_data;
        out = idle_out;
        break;
    }
    }

    auto ret = change_state(out.next_state);
    if (!ret.has_value()) {
        LOG_ERR("Error while changing state: %s", ret.error().build_message().c_str());
    }

    // telemetry
    data.rcs_state = current_state;
}

std::expected<void, Error> TVCController::handle_abort(const AbortRequest& req)
{
    LOG_INF("Received abort request");
    abort_entry_time = k_uptime_get();
    auto ret = change_state(RCSState_RCS_STATE_ABORT);
    if (!ret.has_value()) {
        return std::unexpected(ret.error().context("Failed to change state to abort"));
    }
    return {};
}

std::expected<void, Error> TVCController::handle_unprime(const RCSUnprimeRequest& req)
{
    LOG_INF("Received unprime request");
    auto ret = change_state(RCSState_RCS_STATE_IDLE);
    if (!ret.has_value()) {
        return std::unexpected(ret.error().context("Failed to change state to idle"));
    }
    return {};
}

std::expected<void, Error> TVCController::handle_load_pitch_sequence(const RCSLoadRollSequenceRequest& req)
{
    LOG_INF("Received load roll sequence request");


    return {};
}

std::expected<void, Error> TVCController::handle_start_pitch_sequence(const RCSStartRollSequenceRequest& req)
{
    LOG_INF("Received start roll sequence request");
    sequence_start_time = k_uptime_get();
    change_state(RCSState_RCS_STATE_ROLL_SEQ);
    auto ret = change_state(RCSState_RCS_STATE_ROLL_SEQ);
    if (!ret.has_value()) {
        return std::unexpected(ret.error().context("Failed to change state to roll seq"));
    }
    return {};
}

std::expected<void, Error> TVCController::handle_load_valve_sequence(const RCSLoadValveSequenceRequest& req)
{

    LOG_INF("Received open loop valve sequence request");

    auto result = RCSStateValveSeq::get_trace().load(req.trace_deg);
    if (!result)
        return std::unexpected(result.error().context("%s", "Invalid  trace"));
    RCSStateValveSeq::init(true, req.trace_deg.total_time_ms);



    auto ret = change_state(RCSState_RCS_STATE_VALVE_PRIMED);
    if (!ret.has_value()) {
        return std::unexpected(ret.error().context("Failed to change state to roll primed"));
    }

    return {};
}

std::expected<void, Error> RCSController::handle_start_valve_sequence(const RCSStartValveSequenceRequest& req)
{

    LOG_INF("Received start valve sequence request");
    sequence_start_time = k_uptime_get();
    auto ret = change_state(RCSState_RCS_STATE_VALVE_SEQ);
    if (!ret.has_value()) {
        return std::unexpected(ret.error().context("Failed to change state to valve seq"));
    }
    return {};
}

std::expected<void, Error> RCSController::handle_halt(const RCSHaltRequest& req)
{
    LOG_INF("Received halt request");
    auto ret = change_state(RCSState_RCS_STATE_IDLE);
    if (!ret.has_value()) {
        return std::unexpected(ret.error().context("Failed to change state to idle"));
    }
    return {};
}

const char* TVCController::get_state_name(TVCState state)
{
    if (state == TVCState_TVC_STATE_IDLE)
        return "Idle";

    if (state == TVCState_TVC_STATE_VALVE_PRIMED)
        return "Valve Primed";
    if (state == TVCState_TVC_STATE_VALVE_SEQ)
        return "Valve Seq";
    if (state == TVCState_TVC_STATE_PITCH_PRIMED)
        return "Pitch Primed";
    if (state == TVCState_TVC_STATE_PITCH_SEQ)
        return "Pitch Seq";
    if (state == TVCState_TVC_STATE_ABORT)
        return "Abort";
    return "Unknown State";  // Unknown state
}
