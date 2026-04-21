#include "HornetRcs.h"
#include "MutexGuard.h"
#include <zephyr/kernel.h>

K_MUTEX_DEFINE(hornet_rcs_lock);

/// Reset internal state before an active control trace
void HornetRcs::reset()
{
    MutexGuard hornet_rcs_guard{&hornet_rcs_lock};
    // TODO
}

/// Generate a comomand for the cs and ccs RCS propellers.
std::expected<std::tuple<bool, bool, HornetRcsMetrics>, Error> HornetRcs::tick(float roll_command_deg)
{
    MutexGuard hornet_rcs_guard{&hornet_rcs_lock};

    bool rcs_propeller_cw_command = false;
    bool rcs_propeller_ccw_command = false;
    HornetRcsMetrics metrics = HornetRcsMetrics_init_default;
    return {{rcs_propeller_cw_command, rcs_propeller_ccw_command, metrics}};
}
