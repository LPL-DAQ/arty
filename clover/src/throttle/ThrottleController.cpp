#include "ThrottleController.h"
#include "../sensors/AnalogSensors.h"
#include "../ControllerConfig.h"
#include "../server.h"

#include "../config.h"
#include <zephyr/kernel.h>
#include <zephyr/kernel/thread_stack.h>
#include <zephyr/logging/log.h>
#include <utility>


#if defined(CONFIG_RANGER)
#include "ranger/ThrottleRanger.h"
namespace ThrottleImpl = ThrottleRanger;

#elif defined(CONFIG_HORNET)
#include "hornet/ThrottleHornet.h"
namespace ThrottleImpl = ThrottleHornet;
#else
#error "Select either CONFIG_RANGER or CONFIG_HORNET"
#endif

LOG_MODULE_REGISTER(ThrottleController, LOG_LEVEL_INF);

static Trace throttle_thrust_trace;

std::expected<void, Error> ThrottleController::change_state(ThrottleState new_state)
{
    if (current_state == new_state)
        return {};

    switch (new_state) {
    case ThrottleState_THROTTLE_STATE_IDLE:
        current_state = new_state;
        break;

    case ThrottleState_THROTTLE_STATE_CALIBRATE_VALVE:
        current_state = new_state;
        ThrottleImpl::init_calibrate_valve(
            ThrottleImpl::fuel_get_pos_internal(),
            ThrottleImpl::fuel_get_pos_encoder(),
            ThrottleImpl::lox_get_pos_internal(),
            ThrottleImpl::lox_get_pos_encoder());
        break;

    case ThrottleState_THROTTLE_STATE_VALVE_PRIMED:
        current_state = new_state;
        break;

    case ThrottleState_THROTTLE_STATE_VALVE_SEQ:
        current_state = new_state;
        ThrottleImpl::init_valve_sequence(
            valve_sequence_has_fuel,
            valve_sequence_has_lox,
            valve_sequence_fuel_total_time_ms,
            valve_sequence_lox_total_time_ms);
        break;

    case ThrottleState_THROTTLE_STATE_THRUST_PRIMED:
        current_state = new_state;
        ThrottleImpl::init_thrust_sequence(thrust_sequence_total_time_ms);
        break;

    case ThrottleState_THROTTLE_STATE_THRUST_SEQ:
        current_state = new_state;
        break;

    case ThrottleState_THROTTLE_STATE_FLIGHT:
        current_state = new_state;
        break;

    case ThrottleState_THROTTLE_STATE_ABORT:
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

// std::optional<std::pair<AnalogSensorReadings, float>> analog_sensors_readings
void ThrottleController::step_control_loop(DataPacket& data)
{
    int64_t current_time = k_uptime_get();
    uint64_t start_cycle = k_cycle_get_64();
    ThrottleStateOutput out{};

    // --- PROCEDURAL LOGIC DISPATCHER ---
    switch (current_state) {
    case ThrottleState_THROTTLE_STATE_IDLE: {
        auto [idle_out, idle_data] = idle_tick();
        data.which_throttle_state_data = DataPacket_throttle_idle_data_tag;
        data.throttle_state_data.throttle_idle_data = idle_data;
        out = idle_out;
        break;
    }
    case ThrottleState_THROTTLE_STATE_CALIBRATE_VALVE: {
        auto [cal_out, cal_data] = calibrate_valve_tick(current_time);
        data.which_throttle_state_data = DataPacket_throttle_valve_calibration_data_tag;
        data.throttle_state_data.throttle_valve_calibration_data = cal_data;
        out = cal_out;
        break;
    }
    case ThrottleState_THROTTLE_STATE_VALVE_PRIMED: {
        auto [primed_out, primed_data] = valve_primed_tick();
        data.which_throttle_state_data = DataPacket_throttle_idle_data_tag;
        data.throttle_state_data.throttle_idle_data = primed_data;
        out = primed_out;
        break;
    }
    case ThrottleState_THROTTLE_STATE_VALVE_SEQ: {
        auto [seq_out, seq_data] = valve_sequence_tick(current_time);
        data.which_throttle_state_data = DataPacket_throttle_valve_sequence_data_tag;
        data.throttle_state_data.throttle_valve_sequence_data = seq_data;
        out = seq_out;
        break;
    }
    case ThrottleState_THROTTLE_STATE_THRUST_PRIMED: {
        auto [thrust_primed_out, thrust_primed_data] = thrust_primed_tick();
        data.which_throttle_state_data = DataPacket_throttle_idle_data_tag;
        data.throttle_state_data.throttle_idle_data = thrust_primed_data;
        out = thrust_primed_out;
        break;
    }
    case ThrottleState_THROTTLE_STATE_THRUST_SEQ: {
        auto [thrust_out, thrust_data] = thrust_sequence_tick(data.analog_sensors, current_time);
        data.which_throttle_state_data = DataPacket_throttle_thrust_sequence_data_tag;
        data.throttle_state_data.throttle_thrust_sequence_data = thrust_data;
        out = thrust_out;
        break;
    }
    case ThrottleState_THROTTLE_STATE_FLIGHT: {
        auto [flight_out, flight_data] = flight_tick(data.analog_sensors);
        data.which_throttle_state_data = DataPacket_throttle_flight_data_tag;
        data.throttle_state_data.throttle_flight_data = flight_data;
        out = flight_out;
        break;
    }
    case ThrottleState_THROTTLE_STATE_ABORT: {
        auto [abort_out, abort_data] = abort_tick(current_time);
        data.which_throttle_state_data = DataPacket_throttle_abort_data_tag;
        data.throttle_state_data.throttle_abort_data = abort_data;
        out = abort_out;
        break;
    }
    default: {
        auto [idle_out, idle_data] = idle_tick();
        data.which_throttle_state_data = DataPacket_throttle_idle_data_tag;
        data.throttle_state_data.throttle_idle_data = idle_data;
        out = idle_out;
        break;
    }
    }

    #if defined(CONFIG_RANGER)
    auto ranger_ret = ThrottleImpl::tick(out, data);
    if (!ranger_ret.has_value()) {
        LOG_ERR("ThrottleImpl error: %s", ranger_ret.error().build_message().c_str());
        out.next_state = ThrottleState_THROTTLE_STATE_ABORT;
    }
    data.which_throttle_actuator_data = DataPacket_throttle_ranger_data_tag;
    #elif defined(CONFIG_HORNET)
    auto hornet_ret = ThrottleImpl::tick(out, data);
    if (!hornet_ret.has_value()) {
        LOG_ERR("ThrottleImpl error: %s", hornet_ret.error().build_message().c_str());
        out.next_state = ThrottleState_THROTTLE_STATE_ABORT;
    }
    data.which_throttle_actuator_data = DataPacket_throttle_hornet_data_tag;
     #endif
    data.throttle_state_output = out;

    auto ret = change_state(out.next_state);
    if (!ret.has_value()) {
        LOG_ERR("Error while changing state: %s", ret.error().build_message().c_str());
    }

    data.throttle_state = current_state;
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

    auto result = throttle_thrust_trace.load(req.thrust_trace_lbf);
    if (!result)
        return std::unexpected(result.error().context("%s", "Invalid thrust trace"));

    thrust_sequence_total_time_ms = req.thrust_trace_lbf.total_time_ms;

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

// TODO: fix this so it is done in a way that actually makes
std::expected<void, Error> ThrottleController::handle_load_valve_sequence(const ThrottleLoadValveSequenceRequest& req)
{
    #if defined(CONFIG_RANGER)
    LOG_INF("Received open loop valve sequence request");
    bool has_fuel = req.has_fuel_trace_deg;
    bool has_lox = req.has_lox_trace_deg;
    if (!has_fuel && !has_lox) {
        return std::unexpected(Error::from_cause("No sequences provided in load request"));
    }

    valve_sequence_has_fuel = has_fuel;
    valve_sequence_has_lox = has_lox;
    valve_sequence_fuel_total_time_ms = -1.0f;
    valve_sequence_lox_total_time_ms = -1.0f;

    if (has_fuel) {
        auto result = ThrottleImpl::get_fuel_trace().load(req.fuel_trace_deg);
        if (!result)
            return std::unexpected(result.error().context("%s", "Invalid fuel trace"));
        valve_sequence_fuel_total_time_ms = req.fuel_trace_deg.total_time_ms;
    }

    if (has_lox) {
        auto result = ThrottleImpl::get_lox_trace().load(req.lox_trace_deg);
        if (!result)
            return std::unexpected(result.error().context("%s", "Invalid lox trace"));
        valve_sequence_lox_total_time_ms = req.lox_trace_deg.total_time_ms;
    }

    auto ret = change_state(ThrottleState_THROTTLE_STATE_VALVE_PRIMED);
    if (!ret.has_value()) {
        return std::unexpected(ret.error().context("Failed to change state to valve primed"));
    }
    #endif
    return {};
}

std::expected<void, Error> ThrottleController::handle_start_valve_sequence(const ThrottleStartValveSequenceRequest& req)
{
    #if defined(CONFIG_RANGER)
    LOG_INF("Received start valve sequence request");
    sequence_start_time = k_uptime_get();
    auto ret = change_state(ThrottleState_THROTTLE_STATE_VALVE_SEQ);
    if (!ret.has_value()) {
        return std::unexpected(ret.error().context("Failed to change state to valve seq"));
    }
    #endif
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
    #if defined(CONFIG_RANGER)
    LOG_INF("Received calibrate valve request");
    auto ret = change_state(ThrottleState_THROTTLE_STATE_CALIBRATE_VALVE);
    if (!ret.has_value()) {
        return std::unexpected(ret.error().context("Failed to change state to calibrate valve"));
    }
    #endif
    return {};
}

std::expected<void, Error> ThrottleController::handle_reset_valve_position(const ThrottleResetValvePositionRequest& req)
{
    #if defined(CONFIG_RANGER)
    LOG_INF("Received reset valve request");

    if (current_state != ThrottleState_THROTTLE_STATE_IDLE) {
        return std::unexpected(Error::from_cause("Cannot reset valve position unless system is IDLE"));
    }

    switch (req.valve) {

    // this is giving a double -> float warning rn but deal w that later
    case Valve_FUEL:
        LOG_INF("Resetting fuel valve position to %f", (double)req.new_pos_deg);
        ThrottleImpl::fuel_reset_pos(req.new_pos_deg);
        break;
    case Valve_LOX:
        LOG_INF("Resetting lox valve position to %f", (double)req.new_pos_deg);
        ThrottleImpl::lox_reset_pos(req.new_pos_deg);
        break;
    default:
        return std::unexpected(Error::from_cause("Unknown valve identifier provided to reset command"));
    }
    #endif
    return {};
}


// TODO: currently completely dysfunctional idk who or why this was made like this
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
// TODO: also dysfunctional, see above
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

std::pair<ThrottleStateOutput, ThrottleIdleData> ThrottleController::idle_tick()
{
    ThrottleStateOutput out{};
    ThrottleIdleData data{};
    out.power_on = true;
    out.next_state = ThrottleState_THROTTLE_STATE_IDLE;
    return {out, data};
}

std::pair<ThrottleStateOutput, ThrottleValveCalibrationData> ThrottleController::calibrate_valve_tick(uint32_t current_time)
{
    return ThrottleImpl::calibrate_valve_tick(
        current_time,
        ThrottleImpl::fuel_get_pos_internal(),
        ThrottleImpl::lox_get_pos_internal(),
        ThrottleImpl::fuel_get_pos_encoder(),
        ThrottleImpl::lox_get_pos_encoder());
}

std::pair<ThrottleStateOutput, ThrottleIdleData> ThrottleController::valve_primed_tick()
{
    auto [out, data] = idle_tick();
    out.next_state = ThrottleState_THROTTLE_STATE_VALVE_PRIMED;
    return {out, data};
}

std::pair<ThrottleStateOutput, ThrottleValveSequenceData> ThrottleController::valve_sequence_tick(int64_t current_time)
{
    return ThrottleImpl::valve_sequence_tick(current_time, sequence_start_time);
}

std::pair<ThrottleStateOutput, ThrottleIdleData> ThrottleController::thrust_primed_tick()
{
    auto [out, data] = idle_tick();
    out.next_state = ThrottleState_THROTTLE_STATE_THRUST_PRIMED;
    return {out, data};
}

std::pair<ThrottleStateOutput, ThrottleThrustSequenceData> ThrottleController::thrust_sequence_tick(const AnalogSensorReadings& analog_sensors, int64_t current_time)
{
    ThrottleStateOutput out{};
    ThrottleThrustSequenceData data{};

    float elapsed_time = current_time - sequence_start_time;
    if (elapsed_time > thrust_sequence_total_time_ms) {
        out.next_state = ThrottleState_THROTTLE_STATE_IDLE;
        return {out, data};
    }

    auto target_result = throttle_thrust_trace.sample(elapsed_time);
    if (!target_result) {
        LOG_ERR("Failed to sample thrust_trace: %s", target_result.error().build_message().c_str());
        out.next_state = ThrottleState_THROTTLE_STATE_ABORT;
        return {out, data};
    }

    float target_thrust_lbf = *target_result;
    out.has_thrust = true;
    out.thrust = target_thrust_lbf;
    out.power_on = true;
    out.next_state = ThrottleState_THROTTLE_STATE_THRUST_SEQ;

    data.target_thrust = target_thrust_lbf;
    data.thrust_error = 0.0f;

    return {out, data};
}

std::pair<ThrottleStateOutput, ThrottleFlightData> ThrottleController::flight_tick(const AnalogSensorReadings& analog_sensors)
{
    ThrottleStateOutput out{};
    ThrottleFlightData data{};

    float target_thrust = FlightController::get_z_acceleration();
    out.has_thrust = true;
    out.thrust = target_thrust;
    out.power_on = true;
    out.next_state = ThrottleState_THROTTLE_STATE_FLIGHT;

    return {out, data};
}

std::pair<ThrottleStateOutput, ThrottleAbortData> ThrottleController::abort_tick(uint32_t current_time)
{
    ThrottleStateOutput out{};
    ThrottleAbortData data{};

    out.power_on = true;
    out.has_fuel_pos = true;
    out.fuel_pos = DEFAULT_FUEL_POS;
    out.has_lox_pos = true;
    out.lox_pos = DEFAULT_LOX_POS;

    if (current_time - abort_entry_time > 500) {
        out.next_state = ThrottleState_THROTTLE_STATE_IDLE;
    } else {
        out.next_state = ThrottleState_THROTTLE_STATE_ABORT;
    }

    return {out, data};
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
    if (state == ThrottleState_THROTTLE_STATE_OFF)
        return "Off";
    if (state == ThrottleState_THROTTLE_STATE_FLIGHT)
        return "Flight";
    if (state == ThrottleState_THROTTLE_STATE_ABORT)
        return "Abort";
    return "Unknown State";  // Unknown state
}
