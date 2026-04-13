#include "TVCController.h"
#include "../sensors/AnalogSensors.h"
#include "../ControllerConfig.h"
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

namespace {
    Trace x_trace;
    Trace y_trace;
    bool sequence_has_trace = false;
    float sequence_total_time = 0.0f;
}


std::expected<void, Error> TVCController::change_state(TVCState new_state)
{
    if (current_state == new_state)
        return {};

    current_state = new_state;
    LOG_INF("Changed State to %s", get_state_name(current_state));
    return {};
}

std::pair<TVCStateOutput, TVCIdleData> TVCController::idle_tick()
{
    TVCStateOutput out{};
    TVCIdleData data{};
    out.next_state = TVCState_TVC_STATE_IDLE;
    return {out, data};
}

std::pair<TVCStateOutput, TVCIdleData> TVCController::trace_primed_tick()
{
    auto [out, data] = idle_tick();
    out.next_state = TVCState_TVC_STATE_TRACE_PRIMED;
    return {out, data};
}

std::pair<TVCStateOutput, TVCSequenceData> TVCController::sequence_tick(int64_t current_time, int64_t start_time)
{
    TVCStateOutput out{};
    TVCSequenceData data{};
    out.next_state = TVCState_TVC_STATE_TRACE;

    float dt = current_time - start_time;

    if (sequence_has_trace) {
        auto x_target = x_trace.sample(dt);
        if (!x_target) {
            LOG_ERR("Failed to sample X trace: %s", x_target.error().build_message().c_str());
            out.next_state = TVCState_TVC_STATE_IDLE;
            return {out, data};
        }

        auto y_target = y_trace.sample(dt);
        if (!y_target) {
            LOG_ERR("Failed to sample Y trace: %s", y_target.error().build_message().c_str());
            out.next_state = TVCState_TVC_STATE_IDLE;
            return {out, data};
        }

        out.target_x = *x_target;
        out.target_y = *y_target;
        data.x_angle_deg = *x_target;
        data.y_angle_deg = *y_target;
    }

    if (sequence_has_trace && dt >= sequence_total_time) {
        LOG_INF("Done TVC trace sequence, dt was %f", static_cast<double>(dt));
        out.next_state = TVCState_TVC_STATE_IDLE;
    }

    return {out, data};
}

std::pair<TVCStateOutput, TVCFlightData> TVCController::flight_tick(const AnalogSensorReadings& analog_sensors)
{
    TVCStateOutput out{};
    TVCFlightData data{};
    float target_x = FlightController::get_x_angular_acceleration();
    float target_y = FlightController::get_y_angular_acceleration();
    out.next_state = TVCState_TVC_STATE_FLIGHT;
    out.target_x = target_x;
    out.target_y = target_y;
    return {out, data};
}

std::pair<TVCStateOutput, TVCAbortData> TVCController::abort_tick(uint32_t current_time, uint32_t entry_time)
{
    TVCStateOutput out{};
    TVCAbortData data{};

    if (current_time - entry_time > 500) {
        out.next_state = TVCState_TVC_STATE_IDLE;
    } else {
        out.next_state = TVCState_TVC_STATE_ABORT;
    }

    return {out, data};
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

void TVCController::step_control_loop(DataPacket& data )
{
    int64_t current_time = k_uptime_get();

    TVCStateOutput out{};

    // --- PROCEDURAL LOGIC DISPATCHER ---
    switch (current_state) {
    case TVCState_TVC_STATE_IDLE: {
        auto [idle_out, idle_data] = idle_tick();
        data.which_tvc_state_data = DataPacket_tvc_idle_data_tag;
        data.tvc_state_data.tvc_idle_data = idle_data;
        out = idle_out;
        break;
    }

    case TVCState_TVC_STATE_TRACE_PRIMED: {
        auto [primed_out, primed_data] = trace_primed_tick();
        data.which_tvc_state_data = DataPacket_tvc_idle_data_tag;
        data.tvc_state_data.tvc_idle_data = primed_data;
        out = primed_out;
        break;
    }
    case TVCState_TVC_STATE_FLIGHT: {
        auto [flight_out, flight_data] = flight_tick(data.analog_sensors);
        data.which_tvc_state_data = DataPacket_tvc_flight_data_tag;
        data.tvc_state_data.tvc_flight_data = flight_data;
        out = flight_out;
        break;
    }
    case TVCState_TVC_STATE_TRACE: {
        auto [seq_out, seq_data] = sequence_tick(current_time, sequence_start_time);
        data.which_tvc_state_data = DataPacket_tvc_sequence_data_tag;
        data.tvc_state_data.tvc_sequence_data = seq_data;
        out = seq_out;
        break;
    }
    case TVCState_TVC_STATE_ABORT: {
        auto [abort_out, abort_data] = abort_tick(current_time, abort_entry_time);
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

    auto ret = change_state(out.next_state);
    if (!ret.has_value()) {
        LOG_ERR("Error while changing state: %s", ret.error().build_message().c_str());
    }

    auto impl_ret = TVCImpl::tick(out, data);
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

    auto result_x = x_trace.load(req.x_angle_trace_deg);
    if (!result_x)
        return std::unexpected(result_x.error().context("%s", "Invalid X trace"));

    auto result_y = y_trace.load(req.y_angle_trace_deg);
    if (!result_y)
        return std::unexpected(result_y.error().context("%s", "Invalid Y trace"));

    sequence_has_trace = true;
    sequence_total_time = req.x_angle_trace_deg.total_time_ms;

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
    if (state == TVCState_TVC_STATE_OFF)
        return "Off";
    if (state == TVCState_TVC_STATE_FLIGHT)
        return "Flight";
    if (state == TVCState_TVC_STATE_ABORT)
        return "Abort";
    return "Unknown State";  // Unknown state
}
