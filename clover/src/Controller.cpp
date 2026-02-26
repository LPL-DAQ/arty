#include "Controller.h"
#include "SequenceState.h"
#include "ClosedLoopState.h" // NEW
#include "AbortState.h"
#include "ThrottleValve.h"
#include "pts.h"
#include <zephyr/kernel.h>
#include <zephyr/logging/log.h>

LOG_MODULE_REGISTER(Controller, LOG_LEVEL_INF);

K_MSGQ_DEFINE(telemetry_msgq, sizeof(DataPacket), 50, 1);

static void control_timer_expiry(struct k_timer *t);
K_TIMER_DEFINE(control_loop_timer, control_timer_expiry, NULL);

void Controller::init() {
    current_state = &IdleState::get();
    previous_state = current_state;
    current_state->init();

    k_timer_start(&control_loop_timer, K_MSEC(1), K_MSEC(1));
}

void Controller::tick() {
    pt_readings raw_pts = pts_get_last_reading();
    Sensors current_sensors = Sensors_init_default;

    current_sensors.has_ptc401 = true; current_sensors.ptc401 = raw_pts.ptc401;
    current_sensors.has_pto401 = true; current_sensors.pto401 = raw_pts.pto401;
    current_sensors.has_pt202  = true; current_sensors.pt202  = raw_pts.pt202;
    current_sensors.has_pt102  = true; current_sensors.pt102  = raw_pts.pt102;
    current_sensors.has_pt103  = true; current_sensors.pt103  = raw_pts.pt103;
    current_sensors.has_ptf401 = true; current_sensors.ptf401 = raw_pts.ptf401;
    current_sensors.has_ptc402 = true; current_sensors.ptc402 = raw_pts.ptc402;
    current_sensors.has_pt203  = true; current_sensors.pt203  = raw_pts.pt203;

    // --- NEW OOP STATE MACHINE LOGIC ---
    if (current_state != previous_state) {
        previous_state->end();
        current_state->init();
        previous_state = current_state;
    }

    current_state->run(current_sensors);
    // -----------------------------------

    stream_telemetry(current_sensors);
}

void Controller::trigger_abort() {
    abort_entry_time = k_uptime_get();
    change_state(&AbortState::get());
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
        if (!result) return std::unexpected(result.error().context("%s", "Invalid fuel trace"));
    }
    if (req.has_lox_trace) {
        auto result = lox_trace.load(req.lox_trace);
        if (!result) return std::unexpected(result.error().context("%s", "Invalid lox trace"));
    }
    return {};
}

std::expected<void, Error> Controller::handle_start_sequence(const StartSequenceRequest& req) {
    sequence_start_time = k_uptime_get();
    change_state(&SequenceState::get());
    return {};
}

// NEW: Closed Loop Handler
std::expected<void, Error> Controller::handle_start_closed_loop(const StartClosedLoopRequest& req) {
    change_state(&ClosedLoopState::get());
    return {};
}

std::expected<void, Error> Controller::handle_halt_sequence(const HaltSequenceRequest& req) {
    trigger_abort();
    return {};
}

std::expected<void, Error> Controller::handle_reset_valve_position(const ResetValvePositionRequest& req) {
    if (current_state->get_state_enum() != SystemState_STATE_IDLE) {
        return std::unexpected(Error::from_cause("Cannot reset valve position unless system is IDLE"));
    }

    switch (req.valve) {
        case Valve_FUEL:
            LOG_INF("Resetting fuel valve position to 0");
            FuelValve::reset_pos(0.0f);
            break;
        case Valve_LOX:
            LOG_INF("Resetting lox valve position to 0");
            LoxValve::reset_pos(0.0f);
            break;
        default:
            return std::unexpected(Error::from_cause("Unknown valve identifier provided to reset command"));
    }

    return {};
}

void Controller::stream_telemetry(const Sensors& sensors) {
    DataPacket packet = DataPacket_init_default;
    packet.time = k_uptime_ticks() / (float)CONFIG_SYS_CLOCK_TICKS_PER_SEC;
    packet.sensors = sensors;

    packet.state = current_state->get_state_enum();
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
