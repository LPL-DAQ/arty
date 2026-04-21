#pragma once

#include "../Error.h"
#include "clover.pb.h"
#include <expected>
#include <tuple>

namespace FlightController {
void reset();
/// Returns (pitch_angular_accel_rad_s2, yaw_angular_accel_rad_s2, z_accel_m_s2, FlightControllerMetrics)
std::expected<std::tuple<float, float, float, FlightControllerMetrics>, Error>
tick(EstimatedState state, float x_command_m, float y_command_m, float z_command_m);

std::expected<void, Error> handle_configure_gains(const ConfigureFlightControllerGainsRequest& req);

#if CONFIG_TEST
    void set_desired_state_for_testing(const FlightControllerDesiredState& state);
    void set_loop_count_for_testing(uint32_t count);
#endif

}  // namespace FlightController


