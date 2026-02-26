#include "AbortState.h"
#include "Controller.h"
#include "ThrottleValve.h"
#include "IdleState.h"
#include <zephyr/kernel.h>

void AbortState::init() {
    // Drive valves to nominal safe positions quickly on entry
    FuelValve::tick(Controller::DEFAULT_FUEL_POS);
    LoxValve::tick(Controller::DEFAULT_LOX_POS);
}

void AbortState::run(const Sensors& sensors) {
    // Hold safe positions
    FuelValve::tick(Controller::DEFAULT_FUEL_POS);
    LoxValve::tick(Controller::DEFAULT_LOX_POS);

    // Run for ~0.5s before allowing state transition back to idle
    if (k_uptime_get() - Controller::abort_entry_time > 500) {
        Controller::change_state(&IdleState::get());
    }
}

void AbortState::end() {
    FuelValve::stop();
    LoxValve::stop();
}
