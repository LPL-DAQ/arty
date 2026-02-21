#include "Controller.h"
#include "sequencer.h"
#include "ThrottleValve.h"
#include <zephyr/kernel.h>

// Sequence progress tracking
static uint32_t step_count = 0;
static uint32_t total_steps = 0;

void Controller::init() {
    _state = State::IDLE;
    step_count = 0;
}

/**
 * Main 1ms Dispatcher. Routes to state handlers based on current system mode.
 */
void Controller::tick() {
    switch (_state) {
        case State::IDLE:     run_idle();     break;
        case State::SEQUENCE: run_sequence(); break;
        case State::ABORT:    run_abort();    break;
        default:              trigger_abort(); break;
    }
    stream_telemetry();
}

void Controller::run_idle() {
    // Lead Requirement: Continuous sensor data collection (handled in stream_telemetry)
    // Valves remain in their last commanded position or safe-halt
}

void Controller::run_sequence() {
    if (step_count >= total_steps) {
        _state = State::IDLE;
        FuelValve::stop();
        LoxValve::stop();
        return;
    }

    // Pulling calculated targets from the sequencer module
    float f_target = sequencer_get_fuel_at(step_count);
    float l_target = sequencer_get_lox_at(step_count);

    FuelValve::tick(f_target);
    LoxValve::tick(l_target);

    step_count++;
}

void Controller::run_abort() {
    FuelValve::stop();
    LoxValve::stop();

    // Lead Requirement: Run for ~0.5s before allowing state transition
    if (k_uptime_get() - abort_entry_time > 500) {
        _state = State::IDLE;
    }
}

void Controller::trigger_abort() {
    _state = State::ABORT;
    abort_entry_time = k_uptime_get();
}

// Timer Logic: Every 1ms, submit the dispatcher to the work queue
static void control_timer_expiry(struct k_timer *t) {
    Controller::get().tick();
}
K_TIMER_DEFINE(control_loop_timer, control_timer_expiry, NULL);

std::expected<void, Error> Controller::handle_load_motor_sequence(const LoadMotorSequenceRequest& req) {
    // Logic to validate and load sequence into breakpoints
    sequencer_load_from_proto(req);
    total_steps = sequencer_get_total_steps();
    return {};
}

std::expected<void, Error> Controller::handle_start_sequence(const StartSequenceRequest& req) {
    if (total_steps == 0) return std::unexpected(Error::from_cause("No sequence loaded"));

    step_count = 0;
    _state = State::SEQUENCE;
    k_timer_start(&control_loop_timer, K_MSEC(1), K_MSEC(1));
    return {};
}

std::expected<void, Error> Controller::handle_halt_sequence(const HaltSequenceRequest& req) {
    trigger_abort();
    return {};
}

void Controller::stream_telemetry() {
    // Final logic to pack DataPacket and send to server
}
