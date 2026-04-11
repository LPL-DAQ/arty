#include "TVCController.h"
#include "../sensors/AnalogSensors.h"
#include "../ControllerConfig.h"
#include "TVCStateAbort.h"
#include "TVCStateIdle.h"
#include "TVCStateSeq.h"
#include "../server.h"

#include "../config.h"
#include <zephyr/kernel.h>
#include <zephyr/kernel/thread_stack.h>
#include <zephyr/logging/log.h>
#if defined(CONFIG_RANGER)
#include "ranger/TVCRanger.h"
namespace TVCImpl = TVCRanger;
#elif defined(CONFIG_HORNET)
#include "hornet/TVCHornet.h"
namespace TVCImpl = TVCHornet;
#else
#error "Select either CONFIG_RANGER or CONFIG_HORNET"
#endif
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

    case TVCState_TVC_STATE_TRACE_PRIMED:
        if (current_state != TVCState_TVC_STATE_IDLE) {
            return std::unexpected(Error::from_cause("Cannot switch from %s to Trace Primed, must be in Idle", get_state_name(current_state)));
        }
        TVCStateIdle::init();
        current_state = new_state;
        break;

    case TVCState_TVC_STATE_TRACE:
        if (current_state != TVCState_TVC_STATE_TRACE_PRIMED) {
            return std::unexpected(Error::from_cause("Cannot switch from %s to Trace, must be in Trace Primed", get_state_name(current_state)));
        }

        // INIT IS IN THE HANDLER
        current_state = new_state;
        break;

    case TVCState_TVC_STATE_ABORT:
        if (current_state != TVCState_TVC_STATE_TRACE) {
            return std::unexpected(Error::from_cause("Cannot switch from %s to Abort, must be in Trace", get_state_name(current_state)));
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

void TVCController::step_control_loop(DataPacket& data, std::optional<std::pair<AnalogSensorReadings, float>> analog_sensors_readings )
{
    int64_t current_time = k_uptime_get();

    TVCStateOutput out{};

    // --- PROCEDURAL LOGIC DISPATCHER ---
    switch (current_state) {
    case TVCState_TVC_STATE_IDLE: {
        auto [idle_out, idle_data] = TVCStateIdle::tick();
        data.which_tvc_state_data = DataPacket_tvc_idle_data_tag;
        data.tvc_state_data.tvc_idle_data = idle_data;
        out = idle_out;
        break;
    }

    case TVCState_TVC_STATE_TRACE_PRIMED: {
        auto [primed_out, primed_data] = TVCStateIdle::tick();
        primed_out.next_state = TVCState_TVC_STATE_TRACE_PRIMED;
        data.which_tvc_state_data = DataPacket_tvc_idle_data_tag;
        data.tvc_state_data.tvc_idle_data = primed_data;
        out = primed_out;
        break;
    }
    case TVCState_TVC_STATE_TRACE: {
        auto [seq_out, seq_data] = TVCStateSeq::tick(current_time, sequence_start_time);
        data.which_tvc_state_data = DataPacket_tvc_sequence_data_tag;
        data.tvc_state_data.tvc_sequence_data = seq_data;
        out = seq_out;
        break;
    }
    case TVCState_TVC_STATE_ABORT: {
        auto [abort_out, abort_data] = TVCStateAbort::tick(current_time, abort_entry_time);
        data.which_tvc_state_data = DataPacket_tvc_abort_data_tag;
        data.tvc_state_data.tvc_abort_data = abort_data;
        out = abort_out;
        break;
    }
    default: {
        auto [idle_out, idle_data] = TVCStateIdle::tick();
        data.which_tvc_state_data = DataPacket_tvc_idle_data_tag;
        data.tvc_state_data.tvc_idle_data = idle_data;
        out = idle_out;
        break;
    }
    }

    auto ret = change_state(out.next_state);
    if (!ret.has_value()) {
        LOG_ERR("Error while changing state: %s", ret.error().build_message().c_str());
    }

    auto impl_ret = TVCImpl::tick(out, data, data.analog_sensors);
    if (!impl_ret.has_value()) {
        LOG_ERR("TVCImpl error: %s", impl_ret.error().build_message().c_str());
        out.next_state = TVCState_TVC_STATE_ABORT;
    }

    data.tvc_state_output = out;

    // telemetry
    data.tvc_state = current_state;
}

std::expected<void, Error> TVCController::handle_abort(const AbortRequest& req)
{
    LOG_INF("Received abort request");
    abort_entry_time = k_uptime_get();
    auto ret = change_state(TVCState_TVC_STATE_ABORT);
    if (!ret.has_value()) {
        return std::unexpected(ret.error().context("Failed to change state to abort"));
    }
    return {};
}

std::expected<void, Error> TVCController::handle_unprime(const TVCUnprimeRequest& req)
{
    LOG_INF("Received unprime request");
    auto ret = change_state(TVCState_TVC_STATE_IDLE);
    if (!ret.has_value()) {
        return std::unexpected(ret.error().context("Failed to change state to idle"));
    }
    return {};
}

std::expected<void, Error> TVCController::handle_load_sequence(const TVCLoadSequenceRequest& req)
{
    LOG_INF("Received load sequence request");

    auto result_x = TVCStateSeq::get_x_trace().load(req.x_angle_trace_deg);
    if (!result_x)
        return std::unexpected(result_x.error().context("%s", "Invalid X trace"));

    auto result_y = TVCStateSeq::get_y_trace().load(req.y_angle_trace_deg);
    if (!result_y)
        return std::unexpected(result_y.error().context("%s", "Invalid Y trace"));

    TVCStateSeq::init(true, req.x_angle_trace_deg.total_time_ms);

    auto ret = change_state(TVCState_TVC_STATE_TRACE_PRIMED);
    if (!ret.has_value()) {
        return std::unexpected(ret.error().context("Failed to change state to trace primed"));
    }

    return {};
}

std::expected<void, Error> TVCController::handle_start_sequence(const TVCStartSequenceRequest& req)
{
    LOG_INF("Received start sequence request");
    sequence_start_time = k_uptime_get();
    auto ret = change_state(TVCState_TVC_STATE_TRACE);
    if (!ret.has_value()) {
        return std::unexpected(ret.error().context("Failed to change state to trace"));
    }
    return {};
}

std::expected<void, Error> TVCController::handle_halt(const TVCHaltRequest& req)
{
    LOG_INF("Received halt request");
    auto ret = change_state(TVCState_TVC_STATE_IDLE);
    if (!ret.has_value()) {
        return std::unexpected(ret.error().context("Failed to change state to idle"));
    }
    return {};
}

const char* TVCController::get_state_name(TVCState state)
{
    if (state == TVCState_TVC_STATE_IDLE)
        return "Idle";

    if (state == TVCState_TVC_STATE_TRACE_PRIMED)
        return "Trace Primed";
    if (state == TVCState_TVC_STATE_TRACE)
        return "Trace";
    if (state == TVCState_TVC_STATE_ABORT)
        return "Abort";
    return "Unknown State";  // Unknown state
}
