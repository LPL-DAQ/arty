#include "Controller.h"
#include "AnalogSensors.h"
#include "Lidar.h"
#include "ControllerConfig.h"
#include "StateAbort.h"
#include "StateCalibrateValve.h"
#include "StateIdle.h"
#include "StateThrustSeq.h"
#include "StateValveSeq.h"
#include "server.h"

#ifdef CONFIG_HORNET

#elif CONFIG_RANGER
#include "ThrottleValve.h"

#else
#error Either CONFIG_HORNET or CONFIG_RANGER must be set.
#endif

#include "config.h"
#include <zephyr/kernel.h>
#include <zephyr/kernel/thread_stack.h>
#include <zephyr/logging/log.h>

LOG_MODULE_REGISTER(Controller, LOG_LEVEL_INF);

K_MSGQ_DEFINE(telemetry_msgq, sizeof(DataPacket), 100, 1);

// Controller tick workqueue thread
K_THREAD_STACK_DEFINE(controller_step_thread_stack, 4096);
k_work_q controller_step_work_q;

/// Count of how many packets had to be dropped, used for logging. Owned by controller tick workqueue thread.
static int recent_packets_attempted = 0;
static int recent_packets_dropped = 0;

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

        // StateCalibrateValve::init(FuelValve::get_pos_internal(), FuelValve::get_pos_encoder(), LoxValve::get_pos_internal(), LoxValve::get_pos_encoder());
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
    LOG_INF("Triggering initial sensor readings");
    k_sched_lock();
    AnalogSensors::start_sense();
    Lidar1::start_sense();
    // Other sensors here...
    k_sched_unlock();

    LOG_INF("Pausing for initial sensor readings to complete");
    k_sleep(K_MSEC(500));

    // Set up workqueue
    LOG_INF("Initializing workqueue");
    k_work_queue_init(&controller_step_work_q);
    k_work_queue_start(
        &controller_step_work_q, controller_step_thread_stack, K_THREAD_STACK_SIZEOF(controller_step_thread_stack), CONTROLLER_STEP_WORK_Q_PRIORITY, nullptr);

    auto ret = change_state(SystemState_STATE_IDLE);
    if (!ret.has_value()) {
        return std::unexpected(ret.error().context("Failed to change state to idle"));
    }
    LOG_INF("Beginning controller ticks");
    k_timer_start(&control_loop_schedule_timer, K_NSEC(NSEC_PER_CONTROL_TICK), K_NSEC(NSEC_PER_CONTROL_TICK));
    return {};
}

void Controller::step_control_loop(k_work*)
{
    int64_t current_time = k_uptime_get();
    uint64_t start_cycle = k_cycle_get_64();
    DataPacket data = DataPacket_init_default;

    // Read sensors
    auto analog_sensors_readings = AnalogSensors::read();
    if (analog_sensors_readings) {
        std::tie(data.analog_sensors, data.controller_timing.analog_sensors_sense_time_ns) = *analog_sensors_readings;
    }
    else {
        // LOG_WRN("Analog sensor data is not yet ready, leaving defaults.");
    }

auto lidar1 = Lidar1::read();
if (lidar1) {
    LidarReading reading;
    float sense_time_ns = 0.0f;
    std::tie(reading, sense_time_ns) = *lidar1;
    // If you want to store these in data, uncomment and adjust as needed:
    // std::tie(data.lidar, data.lidar_sense_time_ns) = *lidar1;
    LOG_INF("LiDAR distance: %f m, signal: %f, sense time: %f ns", (double)reading.distance_m, (double)reading.strength, (double)sense_time_ns);
} else {
}

    daq_client_status daq_status = get_daq_client_status();

    ControllerOutput out;

    // --- PROCEDURAL LOGIC DISPATCHER ---
    switch (current_state) {
    case SystemState_STATE_IDLE: {
        auto [idle_out, idle_data] = StateIdle::tick();
        data.which_state_data = DataPacket_idle_data_tag;
        data.state_data.idle_data = idle_data;
        out = idle_out;
        break;
    }
    case SystemState_STATE_CALIBRATE_VALVE: {
        // Can make this work over protobuf later
        // TODO: FIXUP
        // auto [cal_out, cal_data] = StateCalibrateValve::tick(
        //     current_time, FuelValve::get_pos_internal(), LoxValve::get_pos_internal(), FuelValve::get_pos_encoder(), LoxValve::get_pos_encoder());
        // data.which_state_data = DataPacket_valve_calibration_data_tag;
        // data.state_data.valve_calibration_data = cal_data;
        // out = cal_out;
        break;
    }
    case SystemState_STATE_VALVE_PRIMED: {
        auto [primed_out, primed_data] = StateIdle::tick();
        primed_out.next_state = SystemState_STATE_VALVE_PRIMED;
        data.which_state_data = DataPacket_idle_data_tag;
        data.state_data.idle_data = primed_data;
        out = primed_out;
        break;
    }
    case SystemState_STATE_VALVE_SEQ: {
        auto [seq_out, seq_data] = StateValveSeq::tick(current_time, sequence_start_time);
        data.which_state_data = DataPacket_valve_sequence_data_tag;
        data.state_data.valve_sequence_data = seq_data;
        out = seq_out;
        break;
    }
    case SystemState_STATE_THRUST_PRIMED: {
        auto [thrust_primed_out, thrust_primed_data] = StateIdle::tick();
        thrust_primed_out.next_state = SystemState_STATE_THRUST_PRIMED;
        data.which_state_data = DataPacket_idle_data_tag;
        data.state_data.idle_data = thrust_primed_data;
        out = thrust_primed_out;
        break;
    }
    case SystemState_STATE_THRUST_SEQ: {
        auto [thrust_out, thrust_data] = StateThrustSeq::tick(data.analog_sensors, current_time, sequence_start_time);
        data.which_state_data = DataPacket_thrust_sequence_data_tag;
        data.state_data.thrust_sequence_data = thrust_data;
        out = thrust_out;
        break;
    }
    case SystemState_STATE_ABORT: {
        auto [abort_out, abort_data] = StateAbort::tick(current_time, abort_entry_time, DEFAULT_FUEL_POS, DEFAULT_LOX_POS);
        data.which_state_data = DataPacket_abort_data_tag;
        data.state_data.abort_data = abort_data;
        out = abort_out;
        break;
    }
    default: {
        auto [idle_out, idle_data] = StateIdle::tick();
        data.which_state_data = DataPacket_idle_data_tag;
        data.state_data.idle_data = idle_data;
        out = idle_out;
        break;
    }
    }

    auto ret = change_state(out.next_state);
    if (!ret.has_value()) {
        LOG_ERR("Error while changing state: %s", ret.error().build_message().c_str());
    }

    if (out.reset_fuel) {
        // TODO: FIXUP
        // FuelValve::reset_pos(out.reset_fuel_pos);
    }
    if (out.reset_lox) {
        // TODO: FIXUP
        // LoxValve::reset_pos(out.reset_lox_pos);
    }

    // TODO: FIXUP
    // FuelValve::tick(out.fuel_on && fuel_powered, out.set_fuel, out.fuel_pos);
    // LoxValve::tick(out.lox_on && lox_powered, out.set_lox, out.lox_pos);

    // telemetry
    data.time_ns = k_ticks_to_ns_near64(k_uptime_ticks());
    data.state = current_state;
    data.data_queue_size = k_msgq_num_used_get(&telemetry_msgq);
    data.sequence_number = udp_sequence_number++;
    data.gnc_connected = true;
    data.gnc_last_pinged_ns = 0;

    data.controller_timing.controller_tick_time_ns = static_cast<float>(k_cycle_get_64() - start_cycle) / sys_clock_hw_cycles_per_sec() * 1e9f;

    data.daq_connected = daq_status.connected;
    data.daq_last_pinged_ns = static_cast<float>(daq_status.last_pinged_ms) * 1e6f;

    data.fuel_valve = {
        .target_pos_deg = out.fuel_pos,
        // TODO: FIXUP
        // .driver_setpoint_pos_deg = FuelValve::get_pos_internal(),
        // .encoder_pos_deg = FuelValve::get_pos_encoder(),
        // .is_on = FuelValve::get_power_on(),
    };
    data.lox_valve = {
        .target_pos_deg = out.lox_pos,
        // TODO: FIXUP
        // .driver_setpoint_pos_deg = LoxValve::get_pos_internal(),
        // .encoder_pos_deg = LoxValve::get_pos_encoder(),
        // .is_on = LoxValve::get_power_on(),
    };

    // Log metrics on packet transmission success rates.
    recent_packets_attempted++;
    if (k_msgq_put(&telemetry_msgq, &data, K_NO_WAIT) != 0) {
        recent_packets_dropped++;
    }
    if (recent_packets_attempted == 10000 && recent_packets_dropped > 0) {
        float percent_packets_dropped = static_cast<float>(recent_packets_dropped) / recent_packets_attempted * 100.0f;
        LOG_WRN(
            "Packets were dropped in the past 10 sec: %f%% (%d dropped, %d attempted)",
            static_cast<double>(percent_packets_dropped),
            recent_packets_dropped,
            recent_packets_attempted);
        recent_packets_attempted = 0;
        recent_packets_dropped = 0;
    }

    // Trigger sensor readings for next tick.
    AnalogSensors::start_sense();
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
        // TODO: FIXUP
        // FuelValve::reset_pos(req.new_pos_deg);
        break;
    case Valve_LOX:
        // TODO: FIXUP
        LOG_INF("Resetting lox valve position to %f", (double)req.new_pos_deg);
        // LoxValve::reset_pos(req.new_pos_deg);
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
