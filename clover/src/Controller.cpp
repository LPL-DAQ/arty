#include "Controller.h"
#include "ThrottleValve.h"
#include "pts.h"
#include <zephyr/kernel.h>

// Define the Thread-Safe Message Queue for Telemetry (10 packets deep)
K_MSGQ_DEFINE(telemetry_msgq, sizeof(DataPacket), 10, 4);

void Controller::init() {
    _state = SystemState_STATE_IDLE;
}

void Controller::tick() {
    // ADDED: Fetch the latest non-blocking ADC hardware readings
    pt_readings raw_pts = pts_get_last_reading();

    // ADDED: Map the C hardware struct to the Protobuf Sensors struct
    Sensors current_sensors = Sensors_init_default;
    // Map your specific PTs based on your devicetree
    current_sensors.pt202 = raw_pts.pt202;
    current_sensors.pt203 = raw_pts.pt203;

    switch (_state) {
        case SystemState_STATE_IDLE:                 run_idle(current_sensors);     break;
        case SystemState_STATE_SEQUENCE:             run_sequence(current_sensors); break;
        case SystemState_STATE_ABORT:                run_abort(current_sensors);    break;
        default:                                            trigger_abort();               break;
    }
    stream_telemetry(current_sensors);
}

void Controller::run_idle(const Sensors& sensors) {
    // Lead Requirement: Continuous sensor data collection (handled in stream_telemetry)
}

void Controller::run_sequence(const Sensors& sensors) {
    float current_time_ms = k_uptime_get() - sequence_start_time;

    // Sample the traces
    auto f_target = fuel_trace.sample(current_time_ms);
    auto l_target = lox_trace.sample(current_time_ms);

    // If sampling fails (e.g., past the end of the trace), sequence is over
    if (!f_target || !l_target) {
        _state = SystemState_STATE_IDLE;
        FuelValve::stop();
        LoxValve::stop();
        return;
    }

    FuelValve::tick(*f_target);
    LoxValve::tick(*l_target);
}

void Controller::run_abort(const Sensors& sensors) {
    // Lead Requirement: Drive valves to nominal safe positions quickly
    FuelValve::tick(81.0f);
    LoxValve::tick(74.0f);

    // Lead Requirement: Run for ~0.5s before allowing state transition
    if (k_uptime_get() - abort_entry_time > 500) {
        _state = SystemState_STATE_IDLE;
    }
}

void Controller::trigger_abort() {
    _state = SystemState_STATE_ABORT;
    abort_entry_time = k_uptime_get();
}

static void control_timer_expiry(struct k_timer *t) {
    Controller::tick();
}
K_TIMER_DEFINE(control_loop_timer, control_timer_expiry, NULL);

std::expected<void, Error> Controller::handle_load_motor_sequence(const LoadMotorSequenceRequest& req) {
    // Load and validate traces. Bubble up errors immediately if invalid.
    if (req.has_fuel_trace) {
        auto result = fuel_trace.load(req.fuel_trace);
        if (!result) return std::unexpected(result.error());
    }
    if (req.has_lox_trace) {
        auto result = lox_trace.load(req.lox_trace);
        if (!result) return std::unexpected(result.error());
    }
    return {};
}

std::expected<void, Error> Controller::handle_start_sequence(const StartSequenceRequest& req) {
    sequence_start_time = k_uptime_get();
    _state = SystemState_STATE_SEQUENCE;
    k_timer_start(&control_loop_timer, K_MSEC(1), K_MSEC(1));
    return {};
}

std::expected<void, Error> Controller::handle_halt_sequence(const HaltSequenceRequest& req) {
    trigger_abort();
    return {};
}

void Controller::stream_telemetry(const Sensors& sensors) {
    DataPacket packet = DataPacket_init_default;
    packet.time = k_uptime_ticks() / (float)CONFIG_SYS_CLOCK_TICKS_PER_SEC;
    packet.sensors = sensors;
    packet.state = _state;
    packet.is_abort = (_state == SystemState_STATE_ABORT);
    packet.sequence_number = udp_sequence_number++;

    packet.data_queue_size = k_msgq_num_used_get(&telemetry_msgq);
    packet.fuel_valve = {true, 0.0f, 0.0f, 0.0f};
    packet.lox_valve = {true, 0.0f, 0.0f, 0.0f};

    // Non-blocking drop into the message queue.
    // Network thread will pull this later. No stuttering!
    k_msgq_put(&telemetry_msgq, &packet, K_NO_WAIT);
}
