#include "ThrottleController.h"
#include "../sensors/AnalogSensors.h"
#include "../ControllerConfig.h"
#include "ThrottleStateAbort.h"
#include "ranger/ThrottleStateCalibrateValve.h"
#include "ThrottleStateIdle.h"
#include "ThrottleStateThrustSeq.h"
#include "ranger/ThrottleStateValveSeq.h"
#include "ranger/ThrottleRanger.h"
#include "../server.h"

#include "../config.h"
#include <zephyr/kernel.h>
#include <zephyr/kernel/thread_stack.h>
#include <zephyr/logging/log.h>

LOG_MODULE_REGISTER(ThrottleController, LOG_LEVEL_INF);


std::expected<void, Error> ThrottleController::change_state(ThrottleState new_state)
{
    if (current_state == new_state)
        return {};


    switch (new_state) {
    case ThrottleState_THROTTLE_STATE_IDLE:
        current_state = new_state;
        ThrottleStateIdle::init();

        break;

    case ThrottleState_THROTTLE_STATE_CALIBRATE_VALVE:
        if (current_state != ThrottleState_THROTTLE_STATE_IDLE) {
            return std::unexpected(Error::from_cause("Cannot switch from %s to Calibrate Valve, must be in Idle", get_state_name(current_state)));
        }
        StateCalibrateValve::init(ThrottleRanger::fuel_get_pos_internal(), ThrottleRanger::fuel_get_pos_encoder(), ThrottleRanger::lox_get_pos_internal(), ThrottleRanger::lox_get_pos_encoder());
        current_state = new_state;
        break;

    case ThrottleState_THROTTLE_STATE_VALVE_PRIMED:
        if (current_state != ThrottleState_THROTTLE_STATE_IDLE) {
            return std::unexpected(Error::from_cause("Cannot switch from %s to Valve Primed, must be in Idle", get_state_name(current_state)));
        }
        ThrottleStateIdle::init();
        current_state = new_state;
        break;

    case ThrottleState_THROTTLE_STATE_VALVE_SEQ:
        if (current_state != ThrottleState_THROTTLE_STATE_VALVE_PRIMED) {
            return std::unexpected(Error::from_cause("Cannot switch from %s to Valve Seq, must be in Valve Primed", get_state_name(current_state)));
        }

        // INIT IS IN THE HANDLER
        current_state = new_state;
        break;

    case ThrottleState_THROTTLE_STATE_THRUST_PRIMED:
        if (current_state != ThrottleState_THROTTLE_STATE_IDLE) {
            return std::unexpected(Error::from_cause("Cannot switch from %s to Thrust Primed, must be in Idle", get_state_name(current_state)));
        }
        ThrottleStateIdle::init();
        current_state = new_state;
        break;

    case ThrottleState_THROTTLE_STATE_THRUST_SEQ:
        if (current_state != ThrottleState_THROTTLE_STATE_THRUST_PRIMED) {
            return std::unexpected(Error::from_cause("Cannot switch from %s to Thrust Seq, must be in Thrust Primed", get_state_name(current_state)));
        }
        // INIT IS IN THE HANDLER
        current_state = new_state;
        break;

    case ThrottleState_THROTTLE_STATE_ABORT:
        if (current_state != ThrottleState_THROTTLE_STATE_THRUST_SEQ && current_state != ThrottleState_THROTTLE_STATE_VALVE_SEQ) {
            return std::unexpected(Error::from_cause("Cannot switch from %s to Abort, must be in Valve Seq or Thrust Seq", get_state_name(current_state)));
        }
        ThrottleStateAbort::init();
        current_state = new_state;
        break;
    default:
        break;
    }
    LOG_INF("Changed State to %s", get_state_name(current_state));

    return {};
}



std::expected<void, Error> ThrottleController::init()
{
    LOG_INF("Triggering initial sensor readings");
    k_sched_lock();
    AnalogSensors::start_sense();
    // Other sensors here...
    k_sched_unlock();

    LOG_INF("Pausing for initial sensor readings to complete");
    k_sleep(K_MSEC(500));

    auto ret = change_state(ThrottleState_THROTTLE_STATE_IDLE);
    if (!ret.has_value()) {
        return std::unexpected(ret.error().context("Failed to change state to idle"));
    }
    return {};
}


void ThrottleController::step_control_loop(DataPacket& data, std::optional<std::pair<AnalogSensorReadings, float>> analog_sensors_readings )
{
    int64_t current_time = k_uptime_get();
    uint64_t start_cycle = k_cycle_get_64();
    ThrottleStateOutput out{};

    // --- PROCEDURAL LOGIC DISPATCHER ---
    switch (current_state) {
    case ThrottleState_THROTTLE_STATE_IDLE: {
        auto [idle_out, idle_data] = ThrottleStateIdle::tick();
        data.which_throttle_state_data = DataPacket_throttle_idle_data_tag;
        data.throttle_state_data.throttle_idle_data = idle_data;
        out = idle_out;
        break;
    }
    case ThrottleState_THROTTLE_STATE_CALIBRATE_VALVE: {
        auto [cal_out, cal_data] = StateCalibrateValve::tick(
            current_time, ThrottleRanger::fuel_get_pos_internal(), ThrottleRanger::lox_get_pos_internal(), ThrottleRanger::fuel_get_pos_encoder(), ThrottleRanger::lox_get_pos_encoder());
        data.which_throttle_state_data = DataPacket_throttle_valve_calibration_data_tag;
        data.throttle_state_data.throttle_valve_calibration_data = cal_data;
        out = cal_out;
        break;
    }
    case ThrottleState_THROTTLE_STATE_VALVE_PRIMED: {
        auto [primed_out, primed_data] = ThrottleStateIdle::tick();
        primed_out.next_state = ThrottleState_THROTTLE_STATE_VALVE_PRIMED;
        data.which_throttle_state_data = DataPacket_throttle_idle_data_tag;
        data.throttle_state_data.throttle_idle_data = primed_data;
        out = primed_out;
        break;
    }
    case ThrottleState_THROTTLE_STATE_VALVE_SEQ: {
        auto [seq_out, seq_data] = ThrottleStateValveSeq::tick(current_time, sequence_start_time);
        data.which_throttle_state_data = DataPacket_throttle_valve_sequence_data_tag;
        data.throttle_state_data.throttle_valve_sequence_data = seq_data;
        out = seq_out;
        break;
    }
    case ThrottleState_THROTTLE_STATE_THRUST_PRIMED: {
        auto [thrust_primed_out, thrust_primed_data] = ThrottleStateIdle::tick();
        thrust_primed_out.next_state = ThrottleState_THROTTLE_STATE_THRUST_PRIMED;
        data.which_throttle_state_data = DataPacket_throttle_idle_data_tag;
        data.throttle_state_data.throttle_idle_data = thrust_primed_data;
        out = thrust_primed_out;
        break;
    }
    case ThrottleState_THROTTLE_STATE_THRUST_SEQ: {
        auto [thrust_out, thrust_data] = StateThrustSeq::tick(data.analog_sensors, current_time, sequence_start_time);
        data.which_throttle_state_data = DataPacket_throttle_thrust_sequence_data_tag;
        data.throttle_state_data.throttle_thrust_sequence_data = thrust_data;
        out = thrust_out;
        break;
    }
    case ThrottleState_THROTTLE_STATE_ABORT: {
        auto [abort_out, abort_data] = ThrottleStateAbort::tick(current_time, abort_entry_time, DEFAULT_FUEL_POS, DEFAULT_LOX_POS);
        data.which_throttle_state_data = DataPacket_throttle_abort_data_tag;
        data.throttle_state_data.throttle_abort_data = abort_data;
        out = abort_out;
        break;
    }
    default: {
        auto [idle_out, idle_data] = ThrottleStateIdle::tick();
        data.which_throttle_state_data = DataPacket_throttle_idle_data_tag;
        data.throttle_state_data.throttle_idle_data = idle_data;
        out = idle_out;
        break;
    }
    }

    // TODO: is this proper for analog sensor readings?
    AnalogSensorReadings analog_sensors = AnalogSensorReadings_init_default;
    if (analog_sensors_readings) {
        analog_sensors = analog_sensors_readings->first;
    }
    auto ranger_ret = ThrottleRanger::tick(out, data, analog_sensors);
    if (!ranger_ret.has_value()) {
        LOG_ERR("ThrottleRanger error: %s", ranger_ret.error().build_message().c_str());
        out.next_state = ThrottleState_THROTTLE_STATE_ABORT;
    }

    data.throttle_state_output = out;
    data.which_throttle_actuator_data = DataPacket_throttle_ranger_data_tag;

    auto ret = change_state(out.next_state);
    if (!ret.has_value()) {
        LOG_ERR("Error while changing state: %s", ret.error().build_message().c_str());
    }

    // TODO: Reset position?

    data.throttle_state = current_state;

    // TODO: send data to primary controller
}

std::expected<void, Error> ThrottleController::handle_abort(const AbortRequest& req)
{
    LOG_INF("Received abort request");
    abort_entry_time = k_uptime_get();
    auto ret = change_state(ThrottleState_THROTTLE_STATE_ABORT);
    if (!ret.has_value()) {
        return std::unexpected(ret.error().context("Failed to change state to abort"));
    }
    return {};
}

std::expected<void, Error> ThrottleController::handle_unprime(const ThrottleUnprimeRequest& req)
{
    LOG_INF("Received unprime request");
    auto ret = change_state(ThrottleState_THROTTLE_STATE_IDLE);
    if (!ret.has_value()) {
        return std::unexpected(ret.error().context("Failed to change state to idle"));
    }
    return {};
}

std::expected<void, Error> ThrottleController::handle_load_thrust_sequence(const ThrottleLoadThrustSequenceRequest& req)
{
    LOG_INF("Received load thrust sequence request");

    auto result = StateThrustSeq::get_trace().load(req.thrust_trace_lbf);
    if (!result)
        return std::unexpected(result.error().context("%s", "Invalid thrust trace"));
    StateThrustSeq::init(req.thrust_trace_lbf.total_time_ms);

    auto ret = change_state(ThrottleState_THROTTLE_STATE_THRUST_PRIMED);
    if (!ret.has_value()) {
        return std::unexpected(ret.error().context("Failed to change state to thrust primed"));
    }

    return {};
}

std::expected<void, Error> ThrottleController::handle_start_thrust_sequence(const ThrottleStartThrustSequenceRequest& req)
{
    LOG_INF("Received start thrust sequence request");
    sequence_start_time = k_uptime_get();
    auto ret = change_state(ThrottleState_THROTTLE_STATE_THRUST_SEQ);
    if (!ret.has_value()) {
        return std::unexpected(ret.error().context("Failed to change state to thrust seq"));
    }
    return {};
}

std::expected<void, Error> ThrottleController::handle_load_valve_sequence(const ThrottleLoadValveSequenceRequest& req)
{

    LOG_INF("Received open loop valve sequence request");
    bool has_fuel = req.has_fuel_trace_deg;
    bool has_lox = req.has_lox_trace_deg;
    if (!has_fuel && !has_lox) {
        return std::unexpected(Error::from_cause("No sequences provided in load request"));
    }

    if (has_fuel && has_lox) {

        auto result = ThrottleStateValveSeq::get_fuel_trace().load(req.fuel_trace_deg);
        if (!result)
            return std::unexpected(result.error().context("%s", "Invalid fuel trace"));
        result = ThrottleStateValveSeq::get_lox_trace().load(req.lox_trace_deg);
        if (!result)
            return std::unexpected(result.error().context("%s", "Invalid lox trace"));
        ThrottleStateValveSeq::init(true, true, req.fuel_trace_deg.total_time_ms, req.lox_trace_deg.total_time_ms);
    }
    else if (has_fuel) {
        auto result = ThrottleStateValveSeq::get_fuel_trace().load(req.fuel_trace_deg);
        if (!result)
            return std::unexpected(result.error().context("%s", "Invalid fuel trace"));
        ThrottleStateValveSeq::init(true, false, req.fuel_trace_deg.total_time_ms, -1.0f);
    }
    else if (has_lox) {
        auto result = ThrottleStateValveSeq::get_lox_trace().load(req.lox_trace_deg);
        if (!result)
            return std::unexpected(result.error().context("%s", "Invalid lox trace"));
        ThrottleStateValveSeq::init(false, true, -1.0f, req.lox_trace_deg.total_time_ms);
    }

    auto ret = change_state(ThrottleState_THROTTLE_STATE_VALVE_PRIMED);
    if (!ret.has_value()) {
        return std::unexpected(ret.error().context("Failed to change state to thrust primed"));
    }

    return {};
}

std::expected<void, Error> ThrottleController::handle_start_valve_sequence(const ThrottleStartValveSequenceRequest& req)
{

    LOG_INF("Received start valve sequence request");
    sequence_start_time = k_uptime_get();
    auto ret = change_state(ThrottleState_THROTTLE_STATE_VALVE_SEQ);
    if (!ret.has_value()) {
        return std::unexpected(ret.error().context("Failed to change state to valve seq"));
    }
    return {};
}

std::expected<void, Error> ThrottleController::handle_halt(const ThrottleHaltRequest& req)
{
    LOG_INF("Received halt request");
    auto ret = change_state(ThrottleState_THROTTLE_STATE_IDLE);
    if (!ret.has_value()) {
        return std::unexpected(ret.error().context("Failed to change state to idle"));
    }
    return {};
}

std::expected<void, Error> ThrottleController::handle_calibrate_valve(const ThrottleCalibrateValveRequest& req)
{
    LOG_INF("Received calibrate valve request");
    auto ret = change_state(ThrottleState_THROTTLE_STATE_CALIBRATE_VALVE);
    if (!ret.has_value()) {
        return std::unexpected(ret.error().context("Failed to change state to calibrate valve"));
    }
    return {};
}

std::expected<void, Error> ThrottleController::handle_reset_valve_position(const ThrottleResetValvePositionRequest& req)
{
    LOG_INF("Received reset valve request");

    if (current_state != ThrottleState_THROTTLE_STATE_IDLE) {
        return std::unexpected(Error::from_cause("Cannot reset valve position unless system is IDLE"));
    }

    switch (req.valve) {

    // this is giving a double -> float warning rn but deal w that later
    case Valve_FUEL:
        LOG_INF("Resetting fuel valve position to %f", (double)req.new_pos_deg);
        ThrottleRanger::fuel_reset_pos(req.new_pos_deg);
        break;
    case Valve_LOX:
        LOG_INF("Resetting lox valve position to %f", (double)req.new_pos_deg);
        ThrottleRanger::lox_reset_pos(req.new_pos_deg);
        break;
    default:
        return std::unexpected(Error::from_cause("Unknown valve identifier provided to reset command"));
    }

    return {};
}


// currently dysfunctional
std::expected<void, Error> ThrottleController::handle_power_on(const ThrottlePowerOnRequest& req)
{
    LOG_INF("Received power on valve request");

    if (current_state != ThrottleState_THROTTLE_STATE_IDLE) {
        return std::unexpected(Error::from_cause("Cannot turn valve on unless system is IDLE"));
    }

    switch (req.valve) {
    case Valve_FUEL:
        LOG_INF("Turning fuel valve on");
        fuel_powered = true;
        break;
    case Valve_LOX:
        LOG_INF("Turning lox valve on");
        lox_powered = true;
        break;
    default:
        return std::unexpected(Error::from_cause("Unknown valve identifier provided to power on command"));
    }
    return {};
}

std::expected<void, Error> ThrottleController::handle_power_off(const ThrottlePowerOffRequest& req)
{
    LOG_INF("Received power off valve request");

    if (current_state != ThrottleState_THROTTLE_STATE_IDLE) {
        return std::unexpected(Error::from_cause("Cannot turn valve off unless system is IDLE"));
    }

    switch (req.valve) {
    case Valve_FUEL:
        fuel_powered = false;
        LOG_INF("Turning fuel valve off");
        break;
    case Valve_LOX:
        lox_powered = false;
        LOG_INF("Turning lox valve off");
        break;
    default:
        return std::unexpected(Error::from_cause("Unknown valve identifier provided to power off command"));
    }
    return {};
}

const char* ThrottleController::get_state_name(ThrottleState state)
{
    if (state == ThrottleState_THROTTLE_STATE_IDLE)
        return "Idle";
    if (state == ThrottleState_THROTTLE_STATE_CALIBRATE_VALVE)
        return "Calibrate Valve";
    if (state == ThrottleState_THROTTLE_STATE_VALVE_PRIMED)
        return "Valve Primed";
    if (state == ThrottleState_THROTTLE_STATE_VALVE_SEQ)
        return "Valve Seq";
    if (state == ThrottleState_THROTTLE_STATE_THRUST_PRIMED)
        return "Thrust Primed";
    if (state == ThrottleState_THROTTLE_STATE_THRUST_SEQ)
        return "Thrust Seq";
    if (state == ThrottleState_THROTTLE_STATE_ABORT)
        return "Abort";
    return "Unknown State";  // Unknown state
}
