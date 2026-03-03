#include "Controller.h"
#include "StateIdle.h"
#include "StateCalibrateValve.h"
#include "StateAbort.h"
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

static void control_timer_expiry(struct k_timer *t);
K_TIMER_DEFINE(control_loop_timer, control_timer_expiry, NULL);

void Controller::change_state(SystemState new_state) {
    if (current_state == new_state) return;

    current_state = new_state;
    switch(current_state) {
        case SystemState_STATE_IDLE: StateIdle::init(); break;
        case SystemState_STATE_CALIBRATION: StateCalibrateValve::init(FuelValve::get_pos_internal(), FuelValve::get_pos_encoder(), LoxValve::get_pos_internal(), LoxValve::get_pos_encoder()); break;
        case SystemState_STATE_VALVE_PRIMED: StateValvePrimed::init(); break;
        case SystemState_STATE_VALVE_SEQ: StateValveSeq::init(); break;
        case SystemState_STATE_THRUST_PRIMED: StateThrustPrimed::init(); break;
        case SystemState_STATE_THRUST_SEQ: StateThrustSeq::init(); break;
        case SystemState_STATE_ABORT: StateAbort::init(); break;
        default: break;
    }
}

int Controller::controller_init() {
    change_state(SystemState_STATE_IDLE);
    k_timer_start(&control_loop_timer, K_MSEC(1), K_MSEC(1));
    LOG_INF("Initializing Controller...");
    return 0;
}

int tick_count = 0; // temp for testing
void Controller::tick() {
    DataPacket packet = DataPacket_init_default;

    // tick_count++;
    // if (tick_count % 2000 == 0) {
    // //     LOG_INF("Controller tick: %d | State: %d   ", tick_count, get_state_id(current_state));
    //     timespec t = get_system_time();
    //     LOG_INF("Time: %f", t.tv_sec + t.tv_nsec / 1e9);
    // }

    // pt_readings raw_pts = pts_get_last_reading();
    Sensors current_sensors = Sensors_init_default;

    // current_sensors.has_ptc401 = true; current_sensors.ptc401 = raw_pts.ptc401;
    // current_sensors.has_pto401 = true; current_sensors.pto401 = raw_pts.pto401;
    // current_sensors.has_pt202  = true; current_sensors.pt202  = raw_pts.pt202;
    // current_sensors.has_pt102  = true; current_sensors.pt102  = raw_pts.pt102;
    // current_sensors.has_pt103  = true; current_sensors.pt103  = raw_pts.pt103;
    // current_sensors.has_ptf401 = true; current_sensors.ptf401 = raw_pts.ptf401;
    // current_sensors.has_ptc402 = true; current_sensors.ptc402 = raw_pts.ptc402;
    // current_sensors.has_pt203  = true; current_sensors.pt203  = raw_pts.pt203;

    ControllerOutput out;

    // --- PROCEDURAL LOGIC DISPATCHER ---
    switch(current_state) {
        case SystemState_STATE_IDLE:{
            auto [idle_out, idle_data] = StateIdle::tick();
            packet.has_idle_data = true;
            packet.idle_data = idle_data;
            out = idle_out;
            break;
        }
        case SystemState_STATE_CALIBRATION: {
            // Can make this work over protobuf later
            auto [cal_out, cal_data] = StateCalibrateValve::tick(k_uptime_get(),FuelValve::get_pos_internal(), LoxValve::get_pos_internal(), FuelValve::get_pos_encoder(), LoxValve::get_pos_encoder(), FuelValve::get_encoder_velocity(), LoxValve::get_encoder_velocity());
            packet.has_calibration_data = true;
            packet.calibration_data = cal_data;
            out = cal_out;
            break;
        }
        case SystemState_STATE_VALVE_PRIMED:{
            auto [primed_out, primed_data] = StateValvePrimed::tick(k_uptime_get(), sequence_start_time, DEFAULT_FUEL_POS, DEFAULT_LOX_POS);
            packet.has_idle_data = true;
            packet.idle_data = primed_data;
            out = primed_out;
            break;
        }
        case SystemState_STATE_VALVE_SEQ:{
            auto [seq_out, seq_data] = StateValveSeq::tick(k_uptime_get(), sequence_start_time, fuel_trace, lox_trace);
            packet.has_valve_sequence_data = true;
            packet.valve_sequence_data = seq_data;
            out = seq_out;
            break;
        }
        case SystemState_STATE_THRUST_PRIMED:{
            auto [thrust_primed_out, thrust_primed_data] = StateThrustPrimed::tick(k_uptime_get(), sequence_start_time, DEFAULT_FUEL_POS, DEFAULT_LOX_POS);
            packet.has_idle_data = true;
            packet.idle_data = thrust_primed_data;
            out = thrust_primed_out;
            break;
        }
        case SystemState_STATE_THRUST_SEQ:{
            auto [thrust_out, thrust_data] = StateThrustSeq::tick(current_sensors.has_ptc401, current_sensors.ptc401);
            packet.has_thrust_sequence_data = true;
            packet.thrust_sequence_data = thrust_data;
            out = thrust_out;
            break;
        }
        case SystemState_STATE_ABORT:{
            auto [abort_out, abort_data] = StateAbort::tick(k_uptime_get(), abort_entry_time, DEFAULT_FUEL_POS, DEFAULT_LOX_POS);
            packet.has_abort_data = true;
            packet.abort_data = abort_data;
            out = abort_out;
            break;
        }
         default:{
            auto [idle_out, idle_data] = StateIdle::tick();
            packet.has_idle_data = true;
            packet.idle_data = idle_data;
            out = idle_out;
            break;
         }
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
    packet.time = k_uptime_ticks() / (float)CONFIG_SYS_CLOCK_TICKS_PER_SEC;
    packet.sensors = current_sensors;

    packet.state = current_state;
    packet.is_abort = (packet.state == SystemState_STATE_ABORT);
    packet.sequence_number = udp_sequence_number++;

    packet.data_queue_size = k_msgq_num_used_get(&telemetry_msgq);

    packet.fuel_valve.enabled = true;
    packet.fuel_valve.target_pos_deg = FuelValve::get_pos_internal();
    packet.fuel_valve.driver_setpoint_pos_deg = FuelValve::get_pos_internal();
    packet.fuel_valve.encoder_pos_deg = FuelValve::get_pos_encoder();

    packet.lox_valve.enabled = true;
    packet.lox_valve.target_pos_deg = LoxValve::get_pos_internal();
    packet.lox_valve.driver_setpoint_pos_deg = LoxValve::get_pos_internal();
    packet.lox_valve.encoder_pos_deg = LoxValve::get_pos_encoder();

    if (k_msgq_put(&telemetry_msgq, &packet, K_NO_WAIT) != 0) {
        printk("ERROR: Telemetry message queue is full! Packet dropped.\n");
    }
}


static void control_timer_expiry(struct k_timer *t) {
    Controller::tick();
}


std::expected<void, Error> Controller::handle_abort(const AbortRequest& req) {
    abort_entry_time = k_uptime_get();
    change_state(SystemState_STATE_ABORT);
    LOG_INF("Received abort request");

}
std::expected<void, Error> Controller::handle_unprime(const UnprimeRequest& req) {
    change_state(SystemState_STATE_IDLE);
    LOG_INF("Received unprime request");
}

std::expected<void, Error> Controller::handle_load_thrust_sequence(const LoadThrustSequenceRequest& req) {
    change_state(SystemState_STATE_THRUST_PRIMED);
    LOG_INF("Received load thrust sequence request");
}
std::expected<void, Error> Controller::handle_start_thrust_sequence(const StartThrustSequenceRequest& req) {
    change_state(SystemState_STATE_THRUST_SEQ);
    LOG_INF("Received start thrust sequence request");
}


std::expected<void, Error> Controller::handle_load_valve_sequence(const LoadValveSequenceRequest& req) {
    LOG_INF("Received open loop valve sequence request");
    change_state(SystemState_STATE_VALVE_PRIMED);
    if (!req.has_fuel_trace && !req.has_lox_trace) {
        return std::unexpected(Error::from_cause("No sequences provided in load request"));
    }

    if (req.has_fuel_trace) {
        auto result = fuel_trace.load(req.fuel_trace);
        if (!result) return std::unexpected(result.error().context("%s", "Invalid fuel trace"));
    }
    if (req.has_lox_trace) {
        auto result = lox_trace.load(req.lox_trace);
        if (!result) return std::unexpected(result.error().context("%s", "Invalid lox trace"));
    }
    return {};
}

std::expected<void, Error> Controller::handle_start_valve_sequence(const StartValveSequenceRequest& req) {
    LOG_INF("Received start valve sequence request");

    sequence_start_time = k_uptime_get();
    change_state(SystemState_STATE_VALVE_SEQ);
    return {};
}

std::expected<void, Error> Controller::handle_halt(const HaltRequest& req) {
    LOG_INF("Received halt request");
    change_state(SystemState_STATE_IDLE);
    return {};
}
std::expected<void, Error> Controller::handle_calibrate_valve(const CalibrateValveRequest& req) {
    LOG_INF("Received calibrate valve request");
    change_state(SystemState_STATE_CALIBRATION);
    return {};
}

std::expected<void, Error> Controller::handle_reset_valve_position(const ResetValvePositionRequest& req) {
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

std::expected<void, Error> Controller::handle_set_controller_state(const SetControllerStateRequest& req)
{
    LOG_INF("Received set controller state request");

    switch (req.state) {
        case SystemState_STATE_IDLE:
        case SystemState_STATE_CALIBRATION:
        case SystemState_STATE_VALVE_PRIMED:
        case SystemState_STATE_VALVE_SEQ:
        case SystemState_STATE_THRUST_PRIMED:
        case SystemState_STATE_THRUST_SEQ:
        case SystemState_STATE_ABORT:
            change_state(req.state);
            return {};

        default:
            return std::unexpected(
                Error::from_cause("Invalid controller state requested"));
    }
}

int Controller::get_state_id(SystemState state) {
        if (state == SystemState_STATE_IDLE) return 1;
        if (state == SystemState_STATE_CALIBRATION) return 2;
        if (state == SystemState_STATE_VALVE_PRIMED) return 3;
        if (state == SystemState_STATE_VALVE_SEQ) return 4;
        if (state == SystemState_STATE_THRUST_PRIMED) return 5;
        if (state == SystemState_STATE_THRUST_SEQ) return 6;
        if (state == SystemState_STATE_ABORT) return 7;
        return -1; // Unknown state
}


