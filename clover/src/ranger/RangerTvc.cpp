#include "RangerTvc.h"
#include "MutexGuard.h"
#include <zephyr/kernel.h>

K_MUTEX_DEFINE(ranger_tvc_lock);

/// Reset internal state before an active control trace
void RangerTvc::reset()
{
    MutexGuard ranger_tvc_guard{&ranger_tvc_lock};
    // TODO
}

/// Generate a comomand for the pitch and yaw actuators.
std::expected<std::tuple<TvcActuatorCommand, TvcActuatorCommand, RangerTvcMetrics>, Error> RangerTvc::tick(float pitch_command_deg, float yaw_command_deg)
{
    MutexGuard ranger_tvc_guard{&ranger_tvc_lock};

    TvcActuatorCommand pitch_actuator_command = TvcActuatorCommand_init_default;
    TvcActuatorCommand yaw_actuator_command = TvcActuatorCommand_init_default;
    RangerTvcMetrics metrics = RangerTvcMetrics_init_default;
    return {{pitch_actuator_command, yaw_actuator_command, metrics}};
}
