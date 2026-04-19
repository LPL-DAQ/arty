#include "RangerRcs.h"
#include "MutexGuard.h"
#include <zephyr/kernel.h>

K_MUTEX_DEFINE(ranger_rcs_lock);

/// Reset internal state before an active control trace
void RangerRcs::reset()
{
    MutexGuard ranger_rcs_guard{&ranger_rcs_lock};
    // TODO
}

/// Generate a comomand for the cs and ccs RCS valves.
std::expected<std::tuple<bool, bool, RangerRcsMetrics>, Error> RangerRcs::tick(float roll_command_deg)
{
    MutexGuard ranger_rcs_guard{&ranger_rcs_lock};

    bool rcs_valve_cw_command = false;
    bool rcs_valve_ccw_command = false;
    RangerRcsMetrics metrics = RangerRcsMetrics_init_default;
    return {{rcs_valve_cw_command, rcs_valve_ccw_command, metrics}};
}
