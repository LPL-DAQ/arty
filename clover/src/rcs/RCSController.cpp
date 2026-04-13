#include "RCSController.h"
#include "../sensors/AnalogSensors.h"
#include "../ControllerConfig.h"
#include "../server.h"

#include "../config.h"
#include <zephyr/kernel.h>
#include <zephyr/kernel/thread_stack.h>
#include <zephyr/logging/log.h>
#if defined(CONFIG_RANGER)
#include "ranger/RCSRanger.h"
namespace RCSImpl = RCSRanger;
#elif defined(CONFIG_HORNET)
#include "hornet/RCSHornet.h"
namespace RCSImpl = RCSHornet;
#else
#error "Select either CONFIG_RANGER or CONFIG_HORNET"
#endif
LOG_MODULE_REGISTER(RCSController, LOG_LEVEL_INF);

namespace {
    Trace valve_trace;
    bool valve_has_trace = false;
    float valve_total_time = 0.0f;

    Trace roll_trace;
    bool roll_has_trace = false;
    float roll_total_time = 0.0f;
}


std::expected<void, Error> RCSController::change_state(RCSState new_state)
{
    if (current_state == new_state)
        return {};

    current_state = new_state;
    LOG_INF("Changed State to %s", get_state_name(current_state));
    return {};
}

std::pair<RCSStateOutput, RCSIdleData> RCSController::idle_tick()
{
    RCSStateOutput out{};
    RCSIdleData data{};
    out.CW = false;
    out.CCW = false;
    out.next_state = RCSState_RCS_STATE_IDLE;
    return {out, data};
}

std::pair<RCSStateOutput, RCSIdleData> RCSController::valve_primed_tick()
{
    auto [out, data] = idle_tick();
    out.next_state = RCSState_RCS_STATE_VALVE_PRIMED;
    return {out, data};
}

std::pair<RCSStateOutput, RCSValveSequenceData> RCSController::valve_sequence_tick(int64_t current_time, int64_t start_time)
{
    RCSStateOutput out{};
    RCSValveSequenceData data{};
    out.CW = false;
    out.CCW = false;
    out.next_state = RCSState_RCS_STATE_VALVE_SEQ;

    float dt = current_time - start_time;

    if (valve_has_trace) {
        auto target = valve_trace.sample(dt);
        if (!target) {
            LOG_ERR("Failed to sample valve trace: %s", target.error().build_message().c_str());
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

    if (valve_has_trace && dt >= valve_total_time) {
        LOG_INF("Done RCS valve trace sequence, dt was %f", static_cast<double>(dt));
        out.next_state = RCSState_RCS_STATE_IDLE;
    }

    return {out, data};
}

std::pair<RCSStateOutput, RCSIdleData> RCSController::roll_primed_tick()
{
    auto [out, data] = idle_tick();
    out.next_state = RCSState_RCS_STATE_ROLL_PRIMED;
    return {out, data};
}

std::pair<RCSStateOutput, RCSRollSequenceData> RCSController::roll_sequence_tick(const AnalogSensorReadings& analog_sensors, int64_t current_time, int64_t start_time)
{
    RCSStateOutput out{};
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

std::pair<RCSStateOutput, RCSFlightData> RCSController::flight_tick()
{
    RCSStateOutput out{};
    RCSFlightData data{};

    float target_roll = FlightController::get_roll_position();
    out.CW = false;
    out.CCW = false;
    out.next_state = RCSState_RCS_STATE_FLIGHT;
    return {out, data};
}

std::pair<RCSStateOutput, RCSAbortData> RCSController::abort_tick(uint32_t current_time, uint32_t entry_time)
{
    RCSStateOutput out{};
    RCSAbortData data{};

    if (current_time - entry_time > 500) {
        out.next_state = RCSState_RCS_STATE_IDLE;
    } else {
        out.next_state = RCSState_RCS_STATE_ABORT;
    }

    out.CW = false;
    out.CCW = false;
    return {out, data};
}

std::expected<void, Error> RCSController::init(){
    LOG_INF("Triggering initial sensor readings");
    k_sched_lock();
    AnalogSensors::start_sense();
    // Other sensors here...
    k_sched_unlock();

    LOG_INF("Pausing for initial sensor readings to complete");
    k_sleep(K_MSEC(500));

    auto ret = change_state(RCSState_RCS_STATE_IDLE);
    if (!ret.has_value()) {
        return std::unexpected(ret.error().context("Failed to change state to idle"));
    }
    return {};
}

void RCSController::step_control_loop(DataPacket& data )
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

    case RCSState_RCS_STATE_VALVE_PRIMED: {
        auto [primed_out, primed_data] = valve_primed_tick();
        data.which_rcs_state_data = DataPacket_rcs_idle_data_tag;
        data.rcs_state_data.rcs_idle_data = primed_data;
        out = primed_out;
        break;
    }
    case RCSState_RCS_STATE_VALVE_SEQ: {
        auto [seq_out, seq_data] = valve_sequence_tick(current_time, sequence_start_time);
        data.which_rcs_state_data = DataPacket_rcs_valve_sequence_data_tag;
        data.rcs_state_data.rcs_valve_sequence_data = seq_data;
        out = seq_out;
        break;
    }
    case RCSState_RCS_STATE_ROLL_PRIMED: {
        auto [roll_primed_out, roll_primed_data] = roll_primed_tick();
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
        auto [flight_out, flight_data] = flight_tick();
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

    auto ret = change_state(out.next_state);
    if (!ret.has_value()) {
        LOG_ERR("Error while changing state: %s", ret.error().build_message().c_str());
    }

    auto impl_ret = RCSImpl::tick(out, data);
    if (!impl_ret.has_value()) {
        LOG_ERR("RCSImpl error: %s", impl_ret.error().build_message().c_str());
        out.next_state = RCSState_RCS_STATE_ABORT;
    }

    data.rcs_state_output = out;

    // telemetry
    data.rcs_state = current_state;
}

std::expected<void, Error> RCSController::handle_abort(const AbortRequest& req)
{
    LOG_INF("Received abort request");
    abort_entry_time = k_uptime_get();
    auto ret = change_state(RCSState_RCS_STATE_ABORT);
    if (!ret.has_value()) {
        return std::unexpected(ret.error().context("Failed to change state to abort"));
    }
    return {};
}

std::expected<void, Error> RCSController::handle_unprime(const RCSUnprimeRequest& req)
{
    LOG_INF("Received unprime request");
    auto ret = change_state(RCSState_RCS_STATE_IDLE);
    if (!ret.has_value()) {
        return std::unexpected(ret.error().context("Failed to change state to idle"));
    }
    return {};
}

std::expected<void, Error> RCSController::handle_load_roll_sequence(const RCSLoadRollSequenceRequest& req)
{
    LOG_INF("Received load roll sequence request");

    auto result = roll_trace.load(req.Roll_trace_lbf);
    if (!result)
        return std::unexpected(result.error().context("%s", "Invalid roll trace"));

    roll_has_trace = true;
    roll_total_time = req.Roll_trace_lbf.total_time_ms;

    auto ret = change_state(RCSState_RCS_STATE_ROLL_PRIMED);
    if (!ret.has_value()) {
        return std::unexpected(ret.error().context("Failed to change state to roll primed"));
    }

    return {};
}

std::expected<void, Error> RCSController::handle_start_roll_sequence(const RCSStartRollSequenceRequest& req)
{
    LOG_INF("Received start roll sequence request");
    sequence_start_time = k_uptime_get();
    auto ret = change_state(RCSState_RCS_STATE_ROLL_SEQ);
    if (!ret.has_value()) {
        return std::unexpected(ret.error().context("Failed to change state to roll seq"));
    }
    return {};
}

std::expected<void, Error> RCSController::handle_load_valve_sequence(const RCSLoadValveSequenceRequest& req)
{
    LOG_INF("Received open loop valve sequence request");

    auto result = valve_trace.load(req.trace_deg);
    if (!result)
        return std::unexpected(result.error().context("%s", "Invalid trace"));

    valve_has_trace = true;
    valve_total_time = req.trace_deg.total_time_ms;

    auto ret = change_state(RCSState_RCS_STATE_VALVE_PRIMED);
    if (!ret.has_value()) {
        return std::unexpected(ret.error().context("Failed to change state to valve primed"));
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

const char* RCSController::get_state_name(RCSState state)
{
    if (state == RCSState_RCS_STATE_IDLE)
        return "Idle";

    if (state == RCSState_RCS_STATE_VALVE_PRIMED)
        return "Valve Primed";
    if (state == RCSState_RCS_STATE_VALVE_SEQ)
        return "Valve Seq";
    if (state == RCSState_RCS_STATE_ROLL_PRIMED)
        return "Roll Primed";
    if (state == RCSState_RCS_STATE_ROLL_SEQ)
        return "Roll Seq";
    if (state == RCSState_RCS_STATE_OFF)
        return "Off";
    if (state == RCSState_RCS_STATE_FLIGHT)
        return "Flight";
    if (state == RCSState_RCS_STATE_ABORT)
        return "Abort";
    return "Unknown State";  // Unknown state
}
