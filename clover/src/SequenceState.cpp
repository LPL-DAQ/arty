#include "SequenceState.h"
#include "Controller.h"
#include "ThrottleValve.h"
#include "IdleState.h"
#include <zephyr/kernel.h>

void SequenceState::init() {
    // Timer is reset in handle_start_sequence before entering
}

void SequenceState::run(const Sensors& sensors) {
    float current_time_ms = k_uptime_get() - Controller::sequence_start_time;

    // Sample the traces
    auto f_target = Controller::fuel_trace.sample(current_time_ms);
    auto l_target = Controller::lox_trace.sample(current_time_ms);

    // If sample fails (e.g. past end of trace or error), sequence is over
    if (!f_target || !l_target) {
        Controller::change_state(&IdleState::get());
        return;
    }

    // Pass the raw float values
    FuelValve::tick(*f_target);
    LoxValve::tick(*l_target);
}

void SequenceState::end() {
    // Stop valves when leaving the sequence
    FuelValve::stop();
    LoxValve::stop();
}
