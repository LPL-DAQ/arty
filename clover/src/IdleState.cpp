#include "IdleState.h"
#include "ThrottleValve.h"

void IdleState::init() {
    // Ensure valves are stopped when entering idle
    FuelValve::stop();
    LoxValve::stop();
}

void IdleState::run(const Sensors& sensors) {
    // Continuous sensor data collection is handled by Controller::stream_telemetry
}

void IdleState::end() {
    // Nothing to clean up
}
