#include "Controller.h"
#include "StateAbort.h"
#include "StateCalibrateValve.h"
#include "StateIdle.h"
#include "StateThrustPrimed.h"
#include "StateThrustSeq.h"
#include "StateValvePrimed.h"
#include "StateValveSeq.h"
#include "ThrottleValve.h"
#include "pts.h"
// #include "sntp_imp.h"

/**

  STATE_UNKNOWN = 0;
  STATE_IDLE = 1;
  STATE_CALIBRATE_VALVE = 2;
  STATE_VALVE_PRIMED = 3;
  STATE_VALVE_SEQ = 4;
  STATE_THRUST_PRIMED = 5;
  STATE_THRUST_SEQ = 6;
  STATE_ABORT = 7;

*/

#include <zephyr/kernel.h>
#include <zephyr/logging/log.h>

LOG_MODULE_REGISTER(Controller, LOG_LEVEL_INF);

K_MSGQ_DEFINE(telemetry_msgq, sizeof(DataPacket), 50, 1);

constexpr uint64_t NSEC_PER_CONTROL_TICK = 1'000'000;  // 1 ms

void Controller::change_state(SystemState new_state)
{
    if (current_state == new_state)
        return;

    current_state = new_state;
    switch (current_state) {
    case SystemState_STATE_IDLE:
        StateIdle::init();
        break;
    case SystemState_STATE_CALIBRATE_VALVE:
        StateCalibrateValve::init(FuelValve::get_pos_internal(), FuelValve::get_pos_encoder(), LoxValve::get_pos_internal(), LoxValve::get_pos_encoder());
        break;
    case SystemState_STATE_VALVE_PRIMED:
        StateValvePrimed::init();
        break;
    case SystemState_STATE_VALVE_SEQ:
        StateValveSeq::init();
        break;
    case SystemState_STATE_THRUST_PRIMED:
        StateThrustPrimed::init();
        break;
    case SystemState_STATE_THRUST_SEQ:
        StateThrustSeq::init();
        break;
    case SystemState_STATE_ABORT:
        StateAbort::init();
        break;
    default:
        break;
    }
}

K_WORK_DEFINE(control_loop, Controller::step_control_loop);

// ISR that schedules a control iteration in the work queue.
static void control_loop_schedule(k_timer* timer)
{
    k_work_submit(&control_loop);
}

K_TIMER_DEFINE(control_loop_schedule_timer, control_loop_schedule, nullptr);

int Controller::init()
{
    change_state(SystemState_STATE_IDLE);
    k_timer_start(&control_loop_schedule_timer, K_NSEC(NSEC_PER_CONTROL_TICK), K_NSEC(NSEC_PER_CONTROL_TICK));
    LOG_INF("Initializing Controller...");
    return 0;
}

int tick_count = 0;  // temp for testing
void Controller::step_control_loop(k_work*)
{
    DataPacket packet = DataPacket_init_default;

    tick_count++;
    if (tick_count % 2000 == 0) {
        LOG_INF("Controller tick: %d | State: %d   ", tick_count, get_state_id(current_state));
    }

    pt_readings raw_pts = pts_get_last_reading();
    AnalogSensors current_sensors = AnalogSensors_init_default;

    current_sensors.ptc401 = raw_pts.ptc401;
    current_sensors.pto401 = raw_pts.pto401;
    current_sensors.pt202 = raw_pts.pt202;
    current_sensors.pt102 = raw_pts.pt102;
    current_sensors.pt103 = raw_pts.pt103;
    current_sensors.ptf401 = raw_pts.ptf401;
    current_sensors.ptc402 = raw_pts.ptc402;
    current_sensors.pt203 = raw_pts.pt203;

    ControllerOutput out;

    // --- PROCEDURAL LOGIC DISPATCHER ---
    switch (current_state) {
    case SystemState_STATE_IDLE: {
        auto [idle_out, idle_data] = StateIdle::tick();
        packet.state_data.idle_data = idle_data;
        out = idle_out;
        break;
    }
    case SystemState_STATE_CALIBRATE_VALVE: {
        // Can make this work over protobuf later
        auto [cal_out, cal_data] = StateCalibrateValve::tick(
            k_uptime_get(), FuelValve::get_pos_internal(), LoxValve::get_pos_internal(), FuelValve::get_pos_encoder(), LoxValve::get_pos_encoder());
        packet.state_data.valve_calibration_data = cal_data;
        out = cal_out;
        break;
    }
    case SystemState_STATE_VALVE_PRIMED: {
        auto [primed_out, primed_data] = StateValvePrimed::tick(k_uptime_get(), sequence_start_time, DEFAULT_FUEL_POS, DEFAULT_LOX_POS);
        packet.state_data.idle_data = primed_data;
        out = primed_out;
        break;
    }
    case SystemState_STATE_VALVE_SEQ: {
        auto [seq_out, seq_data] = StateValveSeq::tick(k_uptime_get(), sequence_start_time, fuel_trace, lox_trace);
        packet.state_data.valve_sequence_data = seq_data;
        out = seq_out;
        break;
    }
    case SystemState_STATE_THRUST_PRIMED: {
        auto [thrust_primed_out, thrust_primed_data] = StateThrustPrimed::tick(k_uptime_get(), sequence_start_time, DEFAULT_FUEL_POS, DEFAULT_LOX_POS);
        packet.state_data.idle_data = thrust_primed_data;
        out = thrust_primed_out;
        break;
    }
    case SystemState_STATE_THRUST_SEQ: {
        auto [thrust_out, thrust_data] = StateThrustSeq::tick(true, current_sensors.ptc401);
        packet.state_data.thrust_sequence_data = thrust_data;
        out = thrust_out;
        break;
    }
    case SystemState_STATE_ABORT: {
        auto [abort_out, abort_data] = StateAbort::tick(k_uptime_get(), abort_entry_time, DEFAULT_FUEL_POS, DEFAULT_LOX_POS);
        packet.state_data.abort_data = abort_data;
        out = abort_out;
        break;
    }
    default: {
        auto [idle_out, idle_data] = StateIdle::tick();
        packet.state_data.idle_data = idle_data;
        out = idle_out;
        break;
    }
    }

    if (tick_count % 500 == 0) {
        LOG_INF("Controller output - cmd_pos: %f | pos_e %f | pos_i: %f ", out.lox_pos, LoxValve::get_pos_encoder(), LoxValve::get_pos_internal());
    }

    change_state(out.next_state);

    if (out.reset_fuel) {
        FuelValve::reset_pos(out.reset_fuel_pos);
    }
    if (out.reset_lox) {
        LoxValve::reset_pos(out.reset_lox_pos);
    }

    FuelValve::tick(out.fuel_on, out.set_fuel, out.fuel_pos);
    LoxValve::tick(out.lox_on, out.set_lox, out.lox_pos);

    // telementary
    packet.time_ns = k_uptime_ticks() / (float)CONFIG_SYS_CLOCK_TICKS_PER_SEC;  // units may be off
    packet.state = current_state;
    packet.data_queue_size = k_msgq_num_used_get(&telemetry_msgq);
    packet.sequence_number = udp_sequence_number++;
    packet.controller_tick_time_ns = 0;  // WRONG
    packet.gnc_connected = true;         // WRONG
    packet.gnc_last_pinged_ns = 0;       // WRONG
    packet.daq_connected = true;         // WRONG
    packet.daq_last_pinged_ns = 0;       // WRONG

    packet.analog_sensors = current_sensors;
    packet.fuel_valve = ValveStatus_init_default;  // BAD
    packet.lox_valve = ValveStatus_init_default;   // BAD

    if (k_msgq_put(&telemetry_msgq, &packet, K_NO_WAIT) != 0) {
        printk("ERROR: Telemetry message queue is full! Packet dropped.\n");
    }
}

std::expected<void, Error> Controller::handle_abort(const AbortRequest& req)
{
    abort_entry_time = k_uptime_get();
    change_state(SystemState_STATE_ABORT);
    LOG_INF("Received abort request");
    return {};
}

std::expected<void, Error> Controller::handle_unprime(const UnprimeRequest& req)
{
    change_state(SystemState_STATE_IDLE);
    LOG_INF("Received unprime request");
    return {};
}

std::expected<void, Error> Controller::handle_load_thrust_sequence(const LoadThrustSequenceRequest& req)
{
    change_state(SystemState_STATE_THRUST_PRIMED);
    LOG_INF("Received load thrust sequence request");
    return {};
}

std::expected<void, Error> Controller::handle_start_thrust_sequence(const StartThrustSequenceRequest& req)
{
    change_state(SystemState_STATE_THRUST_SEQ);
    LOG_INF("Received start thrust sequence request");
    return {};
}

std::expected<void, Error> Controller::handle_load_valve_sequence(const LoadValveSequenceRequest& req)
{
    LOG_INF("Received open loop valve sequence request");
    change_state(SystemState_STATE_VALVE_PRIMED);
    if (!req.has_fuel_trace_deg && !req.has_lox_trace_deg) {
        return std::unexpected(Error::from_cause("No sequences provided in load request"));
    }

    if (req.has_fuel_trace_deg) {
        auto result = fuel_trace.load(req.fuel_trace_deg);
        if (!result)
            return std::unexpected(result.error().context("%s", "Invalid fuel trace"));
    }
    if (req.has_lox_trace_deg) {
        auto result = lox_trace.load(req.lox_trace_deg);
        if (!result)
            return std::unexpected(result.error().context("%s", "Invalid lox trace"));
    }
    return {};
}

std::expected<void, Error> Controller::handle_start_valve_sequence(const StartValveSequenceRequest& req)
{
    LOG_INF("Received start valve sequence request");

    sequence_start_time = k_uptime_get();
    change_state(SystemState_STATE_VALVE_SEQ);
    return {};
}

std::expected<void, Error> Controller::handle_halt(const HaltRequest& req)
{
    LOG_INF("Received halt request");
    change_state(SystemState_STATE_IDLE);
    return {};
}
std::expected<void, Error> Controller::handle_calibrate_valve(const CalibrateValveRequest& req)
{
    LOG_INF("Received calibrate valve request");
    change_state(SystemState_STATE_CALIBRATE_VALVE);
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
        LOG_INF("Resetting fuel valve position to %f", req.new_pos_deg);
        FuelValve::reset_pos(req.new_pos_deg);
        break;
    case Valve_LOX:
        LOG_INF("Resetting lox valve position to %f", req.new_pos_deg);
        LoxValve::reset_pos(req.new_pos_deg);
        break;
    default:
        return std::unexpected(Error::from_cause("Unknown valve identifier provided to reset command"));
    }

    return {};
}

int Controller::get_state_id(SystemState state)
{
    if (state == SystemState_STATE_IDLE)
        return 1;
    if (state == SystemState_STATE_CALIBRATE_VALVE)
        return 2;
    if (state == SystemState_STATE_VALVE_PRIMED)
        return 3;
    if (state == SystemState_STATE_VALVE_SEQ)
        return 4;
    if (state == SystemState_STATE_THRUST_PRIMED)
        return 5;
    if (state == SystemState_STATE_THRUST_SEQ)
        return 6;
    if (state == SystemState_STATE_ABORT)
        return 7;
    return -1;  // Unknown state
}
