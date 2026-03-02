#include "Controller.h"
#include "IdleState.h"
#include "SequenceState.h"
#include "ClosedLoopState.h"
#include "CalibrationState.h"
#include "AbortState.h"
#include "ThrottleValve.h"
#include "pts.h"
#include <zephyr/kernel.h>
#include <zephyr/logging/log.h>

LOG_MODULE_REGISTER(Controller, LOG_LEVEL_INF);

K_MSGQ_DEFINE(telemetry_msgq, sizeof(DataPacket), 50, 1);

// Forward declaration matching the definition at the bottom
static void control_timer_expiry(struct k_timer *t);
K_TIMER_DEFINE(control_loop_timer, control_timer_expiry, NULL);

void Controller::change_state(SystemState new_state) {
    if (current_state == new_state) return;

    current_state = new_state;
    switch(current_state) {
        case SystemState_STATE_IDLE: IdleState::init(); break;
        case SystemState_STATE_SEQUENCE: SequenceState::init(); break;
        case SystemState_STATE_ABORT: AbortState::init(); break;
        case SystemState_STATE_CLOSED_LOOP_THROTTLE: ClosedLoopState::init(); break;
        case SystemState_STATE_CALIBRATION: CalibrationState::init(FuelValve::get_pos_encoder(), LoxValve::get_pos_encoder()); break;
        default: break;
    }
}

std::expected<void, Error> Controller::controller_init() {
    LOG_INF("Initializing Controller...");
    change_state(SystemState_STATE_IDLE);
    k_timer_start(&control_loop_timer, K_MSEC(1), K_MSEC(1));
    return {};
}

int tick_count = 0;
void Controller::tick() {
    DataPacket packet = DataPacket_init_default;

    tick_count++;
    if (tick_count % 2000 == 0) {
        LOG_INF("Controller tick: %d | State: %d   ", tick_count, get_state_id(current_state));
    }

    Sensors current_sensors = Sensors_init_default;
    ControllerOutput out;

    switch(current_state) {
        case SystemState_STATE_IDLE:
            out = IdleState::tick();
            break;
        case SystemState_STATE_SEQUENCE:
            out = SequenceState::tick(k_uptime_get(), sequence_start_time, fuel_trace, lox_trace);
            break;
        case SystemState_STATE_ABORT:
            out = AbortState::tick(k_uptime_get(), abort_entry_time, DEFAULT_FUEL_POS, DEFAULT_LOX_POS);
            break;
        case SystemState_STATE_CLOSED_LOOP_THROTTLE:
            out = ClosedLoopState::tick(current_sensors.has_ptc401, current_sensors.ptc401);
            break;
        case SystemState_STATE_CALIBRATION: {
            auto [cal_out, cal_data] = CalibrationState::tick(
                k_uptime_get(),
                FuelValve::get_pos_internal(),
                LoxValve::get_pos_internal(),
                FuelValve::get_pos_encoder(),
                LoxValve::get_pos_encoder(),
                FuelValve::get_encoder_velocity(),
                LoxValve::get_encoder_velocity()
            );
            packet.has_calibration_data = true;
            packet.calibration_data = cal_data;
            out = cal_out;
            break;
        }
        default:
            out = IdleState::tick();
            break;
    }

    if (out.next_state != current_state) {
        change_state(out.next_state);
    }

   if (tick_count % 500 == 0) {
    LOG_INF("Controller output - cmd_pos: %f | pos_e %f | pos_i: %f ",
            static_cast<double>(out.fuel_pos),
            static_cast<double>(FuelValve::get_pos_encoder()),
            static_cast<double>(FuelValve::get_pos_internal()));
}

    FuelValve::tick(out.fuel_on, out.set_fuel, out.fuel_pos);
    LoxValve::tick(out.lox_on, out.set_lox, out.lox_pos);

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
        // Using LOG_WRN instead of printk for consistency
        LOG_WRN("Telemetry queue full, packet dropped");
    }
}

void Controller::trigger_abort() {
    abort_entry_time = k_uptime_get();
    change_state(SystemState_STATE_ABORT);
}

static void control_timer_expiry(struct k_timer *t) {
    Controller::tick();
}

std::expected<void, Error> Controller::handle_load_motor_sequence(const LoadMotorSequenceRequest& req) {
    if (!req.has_fuel_trace && !req.has_lox_trace) {
        return std::unexpected(Error::from_cause("No sequences provided in load request"));
    }

    if (req.has_fuel_trace) {
        auto result = fuel_trace.load(req.fuel_trace);
        if (!result) return std::unexpected(result.error().context("Invalid fuel trace"));
    }
    if (req.has_lox_trace) {
        auto result = lox_trace.load(req.lox_trace);
        if (!result) return std::unexpected(result.error().context("Invalid lox trace"));
    }
    return {};
}

std::expected<void, Error> Controller::handle_start_sequence(const StartSequenceRequest& req) {
    sequence_start_time = k_uptime_get();
    change_state(SystemState_STATE_SEQUENCE);
    return {};
}

std::expected<void, Error> Controller::handle_start_closed_loop(const StartThrottleClosedLoopRequest& req) {
    change_state(SystemState_STATE_CLOSED_LOOP_THROTTLE);
    return {};
}

std::expected<void, Error> Controller::handle_halt_sequence(const HaltSequenceRequest& req) {
    trigger_abort();
    return {};
}

std::expected<void, Error> Controller::handle_reset_valve_position(const ResetValvePositionRequest& req) {
    if (current_state != SystemState_STATE_IDLE) {
        return std::unexpected(Error::from_cause("Cannot reset valve position unless system is IDLE"));
    }

    switch (req.valve) {
        case Valve_FUEL:
            LOG_INF("Resetting fuel valve position to %f", static_cast<double>(req.new_pos_deg));
            FuelValve::reset_pos(req.new_pos_deg);
            break;
        case Valve_LOX:
            LOG_INF("Resetting lox valve position to %f", static_cast<double>(req.new_pos_deg));
            LoxValve::reset_pos(req.new_pos_deg);
            break;
        default:
            return std::unexpected(Error::from_cause("Unknown valve identifier"));
    }

    return {};
}

std::expected<void, Error> Controller::handle_set_controller_state(const SetControllerStateRequest& req)
{
    switch (req.state) {
        case SystemState_STATE_IDLE:
        case SystemState_STATE_SEQUENCE:
        case SystemState_STATE_CLOSED_LOOP_THROTTLE:
        case SystemState_STATE_ABORT:
        case SystemState_STATE_CALIBRATION:
            change_state(req.state);
            return {};
        default:
            return std::unexpected(Error::from_cause("Invalid controller state requested"));
    }
}

int Controller::get_state_id(SystemState state) {
    switch (state) {
        case SystemState_STATE_IDLE: return 0;
        case SystemState_STATE_SEQUENCE: return 1;
        case SystemState_STATE_CLOSED_LOOP_THROTTLE: return 2;
        case SystemState_STATE_ABORT: return 3;
        case SystemState_STATE_CALIBRATION: return 4;
        default: return -1;
    }
}
