#include "FlightController.h"
#include "MutexGuard.h"
#include <zephyr/kernel.h>

K_MUTEX_DEFINE(flight_controller_lock);

/// Called before entrance into FLIGHT state to reset internal logic.
void FlightController::reset()
{
    MutexGuard flight_controller_guard(&flight_controller_lock);
}

/// Called every tick in FLIGHT state. Returns a tuple of:
/// - throttle_thrust_command_lbf
/// - tvc_pitch_command_deg
/// - tvc_yaw_command_deg
/// - rcs_roll_command_deg
/// - FlightControllerMetrics
std::expected<std::tuple<float, float, float, float, FlightControllerMetrics>, Error>
FlightController::tick(float x_command_m, float y_command_m, float z_command_m, float roll_command_deg)
{
    MutexGuard flight_controller_guard(&flight_controller_lock);

    float throttle_thrust_command_lbf = -67.67f;
    float tvc_pitch_command_deg = -67.67f;
    float tvc_yaw_command_deg = -67.67f;
    float rcs_roll_command_deg = -67.67f;
    FlightControllerMetrics metrics = FlightControllerMetrics_init_default;

    // logic...

    return {{throttle_thrust_command_lbf, tvc_pitch_command_deg, tvc_yaw_command_deg, rcs_roll_command_deg, metrics}};
}

/// Configure controller gains.
std::expected<void, Error> FlightController::handle_configure_gains(/* TODO */)
{
    MutexGuard flight_controller_guard(&flight_controller_lock);
    // TODO
    return {};
}
