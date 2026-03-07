#include "Controller.h"
#include "ControllerConfig.h"
#include "StateAbort.h"
#include "StateCalibrateValve.h"
#include "StateIdle.h"
#include "StateThrustSeq.h"
#include "StateValveSeq.h"
#include "ThrottleValve.h"
#include "pts.h"
#include "server.h"

#include <zephyr/kernel.h>
#include <zephyr/logging/log.h>

LOG_MODULE_REGISTER(Controller, LOG_LEVEL_INF);

K_MSGQ_DEFINE(telemetry_msgq, sizeof(DataPacket), 50, 1);

std::expected<void, Error> Controller::change_state(SystemState new_state)
{
    if (current_state == new_state)
        return {};

    switch (new_state) {
    case SystemState_STATE_IDLE:
        current_state = new_state;
        StateIdle::init();

        break;

    case SystemState_STATE_CALIBRATE_VALVE:
        if (current_state != SystemState_STATE_IDLE) {
            return std::unexpected(Error::from_cause("Cannot switch from %s to Calibrate Valve, must be in Idle", get_state_name(current_state)));
        }
        StateCalibrateValve::init(FuelValve::get_pos_internal(), FuelValve::get_pos_encoder(), LoxValve::get_pos_internal(), LoxValve::get_pos_encoder());
        current_state = new_state;
        break;

    case SystemState_STATE_VALVE_PRIMED:
        if (current_state != SystemState_STATE_IDLE) {
            return std::unexpected(Error::from_cause("Cannot switch from %s to Valve Primed, must be in Idle", get_state_name(current_state)));
        }
        StateIdle::init();
        current_state = new_state;
        break;

    case SystemState_STATE_VALVE_SEQ:
        if (current_state != SystemState_STATE_VALVE_PRIMED) {
            return std::unexpected(Error::from_cause("Cannot switch from %s to Valve Seq, must be in Valve Primed", get_state_name(current_state)));
        }

        // INIT IS IN THE HANDLER
        current_state = new_state;
        break;

    case SystemState_STATE_THRUST_PRIMED:
        if (current_state != SystemState_STATE_IDLE) {
            return std::unexpected(Error::from_cause("Cannot switch from %s to Thrust Primed, must be in Idle", get_state_name(current_state)));
        }
        StateIdle::init();
        current_state = new_state;
        break;

    case SystemState_STATE_THRUST_SEQ:
        if (current_state != SystemState_STATE_THRUST_PRIMED) {
            return std::unexpected(Error::from_cause("Cannot switch from %s to Thrust Seq, must be in Thrust Primed", get_state_name(current_state)));
        }
        // INIT IS IN THE HANDLER
        current_state = new_state;
        break;

    case SystemState_STATE_ABORT:
        if (current_state != SystemState_STATE_THRUST_SEQ && current_state != SystemState_STATE_VALVE_SEQ) {
            return std::unexpected(Error::from_cause("Cannot switch from %s to Abort, must be in Valve Seq or Thrust Seq", get_state_name(current_state)));
        }
        StateAbort::init();
        current_state = new_state;
        break;
    default:
        break;
    }
    LOG_INF("Changed State to %s", get_state_name(current_state));

    return {};
}

K_WORK_DEFINE(control_loop, Controller::step_control_loop);

// ISR that schedules a control iteration in the work queue.
static void control_loop_schedule(k_timer* timer)
{
    k_work_submit(&control_loop);
}

K_TIMER_DEFINE(control_loop_schedule_timer, control_loop_schedule, nullptr);

std::expected<void, Error> Controller::init()
{
    auto ret = change_state(SystemState_STATE_IDLE);
    if (!ret.has_value()) {
        return std::unexpected(ret.error().context("Failed to change state to idle"));
    }
    k_timer_start(&control_loop_schedule_timer, K_NSEC(NSEC_PER_CONTROL_TICK), K_NSEC(NSEC_PER_CONTROL_TICK));
    LOG_INF("Initializing Controller...");
    return {};
}

static int step_control_loop_debounce_warn_count = 0;

void Controller::step_control_loop(k_work*)
{
    float current_time = k_uptime_get();
    DataPacket packet = DataPacket_init_default;

    pt_readings raw_pts = pts_sample();
    AnalogSensors current_sensors = AnalogSensors_init_default;

    current_sensors.ptc401 = raw_pts.ptc401;
    current_sensors.pto401 = raw_pts.pto401;
    current_sensors.pt202 = raw_pts.pt202;
    current_sensors.pt102 = raw_pts.pt102;
    current_sensors.pt103 = raw_pts.pt103;
    current_sensors.ptf401 = raw_pts.ptf401;
    current_sensors.ptc402 = raw_pts.ptc402;
    current_sensors.pt203 = raw_pts.pt203;
    current_sensors.adc_read_time_ns = pts_get_adc_read_time_ns();
    daq_client_status daq_status = get_daq_client_status();

    ControllerOutput out;

    // --- PROCEDURAL LOGIC DISPATCHER ---
    switch (current_state) {
    case SystemState_STATE_IDLE: {
        auto [idle_out, idle_data] = StateIdle::tick();
        packet.which_state_data = DataPacket_idle_data_tag;
        packet.state_data.idle_data = idle_data;
        out = idle_out;
        break;
    }
    case SystemState_STATE_CALIBRATE_VALVE: {
        // Can make this work over protobuf later
        auto [cal_out, cal_data] = StateCalibrateValve::tick(
            current_time, FuelValve::get_pos_internal(), LoxValve::get_pos_internal(), FuelValve::get_pos_encoder(), LoxValve::get_pos_encoder());
        packet.which_state_data = DataPacket_valve_calibration_data_tag;
        packet.state_data.valve_calibration_data = cal_data;
        out = cal_out;
        break;
    }
    case SystemState_STATE_VALVE_PRIMED: {
        auto [primed_out, primed_data] = StateIdle::tick();
        primed_out.next_state = SystemState_STATE_VALVE_PRIMED;
        packet.which_state_data = DataPacket_idle_data_tag;
        packet.state_data.idle_data = primed_data;
        out = primed_out;
        break;
    }
    case SystemState_STATE_VALVE_SEQ: {
        auto [seq_out, seq_data] = StateValveSeq::tick(current_time, sequence_start_time);
        packet.which_state_data = DataPacket_valve_sequence_data_tag;
        packet.state_data.valve_sequence_data = seq_data;
        out = seq_out;
        break;
    }
    case SystemState_STATE_THRUST_PRIMED: {
        auto [thrust_primed_out, thrust_primed_data] = StateIdle::tick();
        thrust_primed_out.next_state = SystemState_STATE_THRUST_PRIMED;
        packet.which_state_data = DataPacket_idle_data_tag;
        packet.state_data.idle_data = thrust_primed_data;
        out = thrust_primed_out;
        break;
    }
    case SystemState_STATE_THRUST_SEQ: {
        auto [thrust_out, thrust_data] = StateThrustSeq::tick(current_sensors, current_time, sequence_start_time);
        packet.which_state_data = DataPacket_thrust_sequence_data_tag;
        packet.state_data.thrust_sequence_data = thrust_data;
        out = thrust_out;
        break;
    }
    case SystemState_STATE_ABORT: {
        auto [abort_out, abort_data] = StateAbort::tick(current_time, abort_entry_time, DEFAULT_FUEL_POS, DEFAULT_LOX_POS);
        packet.which_state_data = DataPacket_abort_data_tag;
        packet.state_data.abort_data = abort_data;
        out = abort_out;
        break;
    }
    default: {
        auto [idle_out, idle_data] = StateIdle::tick();
        packet.which_state_data = DataPacket_idle_data_tag;
        packet.state_data.idle_data = idle_data;
        out = idle_out;
        break;
    }
    }

    auto ret = change_state(out.next_state);
    if (!ret.has_value()) {
        LOG_ERR("Error while changing state: %s", ret.error().build_message().c_str());
    }

    if (out.reset_fuel) {
        FuelValve::reset_pos(out.reset_fuel_pos);
    }
    if (out.reset_lox) {
        LoxValve::reset_pos(out.reset_lox_pos);
    }

    FuelValve::tick(out.fuel_on && fuel_powered, out.set_fuel, out.fuel_pos);
    LoxValve::tick(out.lox_on && lox_powered, out.set_lox, out.lox_pos);

    // telemetry
    packet.time_ns = k_ticks_to_ns_near64(k_uptime_ticks());
    packet.state = current_state;
    packet.data_queue_size = k_msgq_num_used_get(&telemetry_msgq);
    packet.sequence_number = udp_sequence_number++;
    packet.gnc_connected = true;
    packet.gnc_last_pinged_ns = 0;

    packet.daq_connected = daq_status.connected;
    packet.daq_last_pinged_ns = static_cast<float>(daq_status.last_pinged_ms) * 1e6f;

    packet.analog_sensors = current_sensors;
    packet.analog_sensors.adc_read_time_ns = 9767577;
    packet.fuel_valve = {
        .target_pos_deg = out.fuel_pos,
        .driver_setpoint_pos_deg = FuelValve::get_pos_internal(),
        .encoder_pos_deg = FuelValve::get_pos_encoder(),
        .is_on = FuelValve::get_power_on(),
    };
    packet.lox_valve = {
        .target_pos_deg = out.lox_pos,
        .driver_setpoint_pos_deg = LoxValve::get_pos_internal(),
        .encoder_pos_deg = LoxValve::get_pos_encoder(),
        .is_on = LoxValve::get_power_on(),
    };

    if (k_msgq_put(&telemetry_msgq, &packet, K_NO_WAIT) != 0) {
        if (step_control_loop_debounce_warn_count < 5) {
            LOG_WRN("Telemetry queue full, packet dropped");
        }
        else if (step_control_loop_debounce_warn_count == 5) {
            LOG_WRN("Telemetry queue full, packet dropped (silencing further warnings)");
        }
        step_control_loop_debounce_warn_count++;
    }
    else {
        // Reset warning count
        step_control_loop_debounce_warn_count = 0;
    }
}

std::expected<void, Error> Controller::handle_abort(const AbortRequest& req)
{
    LOG_INF("Received abort request");
    abort_entry_time = k_uptime_get();
    auto ret = change_state(SystemState_STATE_ABORT);
    if (!ret.has_value()) {
        return std::unexpected(ret.error().context("Failed to change state to abort"));
    }
    return {};
}

std::expected<void, Error> Controller::handle_unprime(const UnprimeRequest& req)
{
    LOG_INF("Received unprime request");
    auto ret = change_state(SystemState_STATE_IDLE);
    if (!ret.has_value()) {
        return std::unexpected(ret.error().context("Failed to change state to idle"));
    }
    return {};
}

std::expected<void, Error> Controller::handle_load_thrust_sequence(const LoadThrustSequenceRequest& req)
{
    LOG_INF("Received load thrust sequence request");

    auto result = StateThrustSeq::get_trace().load(req.thrust_trace_lbf);
    if (!result)
        return std::unexpected(result.error().context("%s", "Invalid thrust trace"));
    StateThrustSeq::init(req.thrust_trace_lbf.total_time_ms);

    auto ret = change_state(SystemState_STATE_THRUST_PRIMED);
    if (!ret.has_value()) {
        return std::unexpected(ret.error().context("Failed to change state to thrust primed"));
    }

    return {};
}

std::expected<void, Error> Controller::handle_start_thrust_sequence(const StartThrustSequenceRequest& req)
{
    LOG_INF("Received start thrust sequence request");
    sequence_start_time = k_uptime_get();
    change_state(SystemState_STATE_THRUST_SEQ);
    auto ret = change_state(SystemState_STATE_THRUST_SEQ);
    if (!ret.has_value()) {
        return std::unexpected(ret.error().context("Failed to change state to thrust seq"));
    }
    return {};
}

std::expected<void, Error> Controller::handle_load_valve_sequence(const LoadValveSequenceRequest& req)
{

    LOG_INF("Received open loop valve sequence request");
    bool has_fuel = req.has_fuel_trace_deg;
    bool has_lox = req.has_lox_trace_deg;
    if (!has_fuel && !has_lox) {
        return std::unexpected(Error::from_cause("No sequences provided in load request"));
    }

    if (has_fuel && has_lox) {

        auto result = StateValveSeq::get_fuel_trace().load(req.fuel_trace_deg);
        if (!result)
            return std::unexpected(result.error().context("%s", "Invalid fuel trace"));
        result = StateValveSeq::get_lox_trace().load(req.lox_trace_deg);
        if (!result)
            return std::unexpected(result.error().context("%s", "Invalid lox trace"));
        StateValveSeq::init(true, true, req.fuel_trace_deg.total_time_ms, req.lox_trace_deg.total_time_ms);
    }
    else if (has_fuel) {
        auto result = StateValveSeq::get_fuel_trace().load(req.fuel_trace_deg);
        if (!result)
            return std::unexpected(result.error().context("%s", "Invalid fuel trace"));
        StateValveSeq::init(true, false, req.fuel_trace_deg.total_time_ms, -1.0f);
    }
    else if (has_lox) {
        auto result = StateValveSeq::get_lox_trace().load(req.lox_trace_deg);
        if (!result)
            return std::unexpected(result.error().context("%s", "Invalid lox trace"));
        StateValveSeq::init(false, true, -1.0f, req.lox_trace_deg.total_time_ms);
    }

    auto ret = change_state(SystemState_STATE_VALVE_PRIMED);
    if (!ret.has_value()) {
        return std::unexpected(ret.error().context("Failed to change state to thrust primed"));
    }

    return {};
}

std::expected<void, Error> Controller::handle_start_valve_sequence(const StartValveSequenceRequest& req)
{

    LOG_INF("Received start valve sequence request");
    sequence_start_time = k_uptime_get();
    auto ret = change_state(SystemState_STATE_VALVE_SEQ);
    if (!ret.has_value()) {
        return std::unexpected(ret.error().context("Failed to change state to valve seq"));
    }
    return {};
}

std::expected<void, Error> Controller::handle_halt(const HaltRequest& req)
{
    LOG_INF("Received halt request");
    auto ret = change_state(SystemState_STATE_IDLE);
    if (!ret.has_value()) {
        return std::unexpected(ret.error().context("Failed to change state to idle"));
    }
    return {};
}

std::expected<void, Error> Controller::handle_calibrate_valve(const CalibrateValveRequest& req)
{
    LOG_INF("Received calibrate valve request");
    auto ret = change_state(SystemState_STATE_CALIBRATE_VALVE);
    if (!ret.has_value()) {
        return std::unexpected(ret.error().context("Failed to change state to calibrate valve"));
    }
    return {};
}

std::expected<void, Error> Controller::handle_reset_valve_position(const ResetValvePositionRequest& req)
{
    LOG_INF("Received reset valve request");

    if (current_state != SystemState_STATE_IDLE) {
        return std::unexpected(Error::from_cause("Cannot reset valve position unless system is IDLE"));
    }

    switch (req.valve) {

    // this is giving a double -> float warning rn but deal w that later
    case Valve_FUEL:
        LOG_INF("Resetting fuel valve position to %f", (double)req.new_pos_deg);
        FuelValve::reset_pos(req.new_pos_deg);
        break;
    case Valve_LOX:
        LOG_INF("Resetting lox valve position to %f", (double)req.new_pos_deg);
        LoxValve::reset_pos(req.new_pos_deg);
        break;
    default:
        return std::unexpected(Error::from_cause("Unknown valve identifier provided to reset command"));
    }

    return {};
}

std::expected<void, Error> Controller::handle_power_on_valve(const PowerOnValveRequest& req)
{
    LOG_INF("Received power on valve request");

    if (current_state != SystemState_STATE_IDLE) {
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

std::expected<void, Error> Controller::handle_configure_analog_sensor_bias(const ConfigureAnalogSensorBiasRequest& req)
{
    LOG_INF("Received configure analog sensor bias request");

    // Maps AnalogSensor enum to pt_configs[] index (order from tvc_throttle_dev.dts pt-names)
    int i;
    switch (req.sensor) {
    case AnalogSensor_PTC401:
        i = 0;
        break;
    case AnalogSensor_PTO401:
        i = 1;
        break;
    case AnalogSensor_PT202:
        i = 2;
        break;
    case AnalogSensor_PT102:
        i = 3;
        break;
    case AnalogSensor_PT103:
        i = 4;
        break;
    case AnalogSensor_PTF401:
        i = 5;
        break;
    case AnalogSensor_PT203:
        i = 6;
        break;
    case AnalogSensor_PTC402:
        i = 7;
        break;
    case AnalogSensor_TC102:
    case AnalogSensor_TC102_5:
        return std::unexpected(Error::from_cause("TC sensors are not ADC-sourced and do not support bias configuration"));
    default:
        return std::unexpected(Error::from_cause("Unknown analog sensor identifier"));
    }

    pts_set_bias(i, req.bias);
    return {};
}

std::expected<void, Error> Controller::handle_power_off_valve(const PowerOffValveRequest& req)
{
    LOG_INF("Received power off valve request");

    if (current_state != SystemState_STATE_IDLE) {
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

const char* Controller::get_state_name(SystemState state)
{
    if (state == SystemState_STATE_IDLE)
        return "Idle";
    if (state == SystemState_STATE_CALIBRATE_VALVE)
        return "Calibrate Valve";
    if (state == SystemState_STATE_VALVE_PRIMED)
        return "Valve Primed";
    if (state == SystemState_STATE_VALVE_SEQ)
        return "Valve Seq";
    if (state == SystemState_STATE_THRUST_PRIMED)
        return "Thrust Primed";
    if (state == SystemState_STATE_THRUST_SEQ)
        return "Thrust Seq";
    if (state == SystemState_STATE_ABORT)
        return "Abort";
    return "Unknown State";  // Unknown state
}
