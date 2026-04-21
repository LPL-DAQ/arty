#include "FlightController.h"
#include "../MutexGuard.h"
#include "../PID.h"
#include "../math_util.h"
#include "../config.h"
#include <zephyr/kernel.h>
#include <limits>
#include <cmath>
#include <array>


K_MUTEX_DEFINE(flight_controller_lock);

namespace {
    const float maxGimble = 12.0;   // degrees, this is an estimate
    const float maxTiltRad = 8.0f * 0.0174532925f;  // 8 degrees in radians – max intentional tilt

    // TODO: Tune, including adding integral terms (is requried)
    // reference: kp, ki, kd, min out, max out, min integral, max integral, integral zone, deriv filter
    // integral cant command more than 1/3rd output range, with 10 Hz derivative lowpass
    PID pidXTilt(FLIGHT_PID_X_TILT_KP, FLIGHT_PID_X_TILT_KI, FLIGHT_PID_X_TILT_KD, -1e6, 1e6, -maxGimble / 3, maxGimble / 3, std::numeric_limits<double>::infinity(), 10.0);   // about the X axis
    PID pidYTilt(FLIGHT_PID_Y_TILT_KP, FLIGHT_PID_Y_TILT_KI, FLIGHT_PID_Y_TILT_KD, -1e6, 1e6, -maxGimble / 3, maxGimble / 3, std::numeric_limits<double>::infinity(), 10.0);   // about the Y axis
    PID pidX(FLIGHT_PID_X_KP, FLIGHT_PID_X_KI, FLIGHT_PID_X_KD);              // needs tuning, these are complete guesses
    PID pidY(FLIGHT_PID_Y_KP, FLIGHT_PID_Y_KI, FLIGHT_PID_Y_KD);              // needs tuning, these are complete guesses
    // only use integral within 5 cm of target.
    PID pidZ(FLIGHT_PID_Z_KP, FLIGHT_PID_Z_KI, FLIGHT_PID_Z_KD, -1e6, 1e6, -1e6, 1e6, 0.05);
    // TODO: add max and min out
    PID pidZVelocity(FLIGHT_PID_Z_VEL_KP, FLIGHT_PID_Z_VEL_KI, FLIGHT_PID_Z_VEL_KD);      // needs tuning

    static uint32_t loopCount = 0;

    float dt = 0.01; // TODO: make this an actual DT measurement ( or at least research if i should)

    FlightControllerDesiredState des_state = FlightControllerDesiredState_init_default;

#if CONFIG_HORNET
    constexpr float yaw_MOI = -67.67f;
    constexpr float pitch_MOI = -67.67f;
    constexpr float yaw_moment_arm = -67.67f;
    constexpr float pitch_moment_arm = -67.67f;

#else  // if CONFIG_RANGER
    constexpr float yaw_MOI = -67.67f;
    constexpr float pitch_MOI = -67.67f;
    constexpr float yaw_moment_arm = -67.67f;
    constexpr float pitch_moment_arm = -67.67f;
#endif
}

// TODO: get the real mass estimates
static float get_mass_kg(){
#if CONFIG_HORNET
    return 6.8f; // 15 lbs

#else  // if CONFIG_RANGER
    return 272.0f; // 600 lbs
#endif
}

// TODO: unit test
// converts angular accelerations to tvc commanded angles based on vehicle
static std::expected<std::tuple<float, float>, Error> find_tvc_angles(float pitch_accel, float yaw_accel, float thrust){
    // Physics: Angular acceleration = (Thrust * moment_arm * sin(angle)) / MOI
    // Therefore: sin(angle) = (angular_accel * MOI) / (Thrust * moment_arm)
    // angle_rad = arcsin((angular_accel * MOI) / (Thrust * moment_arm))

    // may be needed if we have an updating MOI
    // float mass_kg = get_mass_kg();

    // Calculate sin of gimbal angles
    float yaw_sin_angle = (yaw_accel * yaw_MOI) / (thrust * yaw_moment_arm);
    float pitch_sin_angle = (pitch_accel * pitch_MOI) / (thrust * pitch_moment_arm);

    // Clamp sin values to valid domain [-1, 1]
    yaw_sin_angle = std::clamp(yaw_sin_angle, -1.0f, 1.0f);
    pitch_sin_angle = std::clamp(pitch_sin_angle, -1.0f, 1.0f);

    // Calculate gimbal angles in radians using arcsin
    float yaw_angle_rad = std::asin(yaw_sin_angle);
    float pitch_angle_rad = std::asin(pitch_sin_angle);

    // Convert from radians to degrees
    float yaw_angle_deg = yaw_angle_rad * RAD2DEG_F;
    float pitch_angle_deg = pitch_angle_rad * RAD2DEG_F;

    // Clamp to maximum gimbal angle
    yaw_angle_deg = std::clamp(yaw_angle_deg, -maxGimble, maxGimble);
    pitch_angle_deg = std::clamp(pitch_angle_deg, -maxGimble, maxGimble);

    return {{pitch_angle_deg, yaw_angle_deg}};
}


// returns {pitch acceleration, yaw acceleration}
static std::array<float, 2> lateralPID(EstimatedState state, FlightControllerMetrics& metrics)
{
    std::array<float, 2> output_accelerations{};

    Quaternion q_wb = math_util::createQuaternion(
        state.R_WB.qw,
        state.R_WB.qx,
        state.R_WB.qy,
        state.R_WB.qz
    );
    q_wb = math_util::normalizeQuaternion(q_wb);

    Quaternion q_bw = math_util::conjugateQuaternion(q_wb);

    // Outer loop: desired literal tilt angles
    if (loopCount % FLIGHT_OUTER_LOOP_DIVISOR == 0)
    {
        float outer_dt = FLIGHT_OUTER_LOOP_DIVISOR * dt;

        des_state.world_tilt_x = pidX.calculate(des_state.position.x, state.position.x, state.velocity.x, outer_dt);
        des_state.world_tilt_y = pidY.calculate(des_state.position.y, state.position.y, state.velocity.y, outer_dt);

        // Clamp if needed
        des_state.world_tilt_x = std::clamp(des_state.world_tilt_x, -maxTiltRad, maxTiltRad);
        des_state.world_tilt_y = std::clamp(des_state.world_tilt_y, -maxTiltRad, maxTiltRad);
    }

    // Actual vertical axis in world
    Vector3D z_act_w = math_util::multiplyQuaternionVector(q_bw, math_util::unitZ());
    metrics.actual_world_tilt_x_rad = std::atan2(z_act_w.x, z_act_w.z);
    metrics.actual_world_tilt_y_rad = std::atan2(z_act_w.y, z_act_w.z);

    // Desired thrust axis in world from desired literal tilt angles
    Vector3D z_des_w = math_util::createVector3D(
        std::tan(des_state.world_tilt_x),
        std::tan(des_state.world_tilt_y),
        1.0
    );
    // Normalize the vector
    z_des_w = math_util::normalizeVector3D(z_des_w);

    // Desired thrust axis expressed in body frame
    Vector3D z_des_b = math_util::multiplyQuaternionVector(q_wb, z_des_w);

    // Body-frame reduced attitude error
    // Unit Z because we are in body frame
    Vector3D axis_error_b = math_util::crossProduct(math_util::unitZ(), z_des_b);

    // Inner loop on body-axis tilt error
    // TODO: Check if this needs a negative sign.
    // TODO: do i need to unscale because of the TVC thrust scaling? i think it should be 3% of 3% error, so its chill, but should double check
    // TODO: find angular rates to feed to derivative

    // Feed body-axis error
    output_accelerations[0] = pidXTilt.calculate(0.0f, axis_error_b.x, dt);
    output_accelerations[1] = pidYTilt.calculate(0.0f, axis_error_b.y, dt);

    metrics.desired_world_tilt_x_rad = des_state.world_tilt_x;
    metrics.desired_world_tilt_y_rad = des_state.world_tilt_y;
    metrics.commanded_pitch_acceleration_rad_s2 = output_accelerations[0];
    metrics.commanded_yaw_acceleration_rad_s2 = output_accelerations[1];

    return output_accelerations;
}

static float verticalPID(EstimatedState state, FlightControllerMetrics& metrics){

    // outerloop on position
    if (loopCount % FLIGHT_OUTER_LOOP_DIVISOR == 0)
    {
        des_state.vz_m_s = pidZ.calculate(des_state.position.z, state.position.z, dt);
    }

    metrics.desired_vertical_velocity_m_s = des_state.vz_m_s;

    //TODO: make this an acceleration delta
    // innerloop on velocity
    float desired_acceleration = pidZVelocity.calculate(des_state.vz_m_s, state.velocity.z, dt);
    metrics.commanded_vertical_acceleration_m_s2 = desired_acceleration;
    return desired_acceleration;
}

/// Called before entrance into FLIGHT state to reset internal logic.
void FlightController::reset()
{
    MutexGuard flight_controller_guard(&flight_controller_lock);
    pidXTilt.reset();
    pidYTilt.reset();
    pidX.reset();
    pidY.reset();
    pidZ.reset();
    pidZVelocity.reset();
    loopCount = 0;
    des_state = FlightControllerDesiredState_init_default;
}

// TODO: roll control, but also handled in a header visable method so that it supports roll traces outside of flight state

/// Called every tick in FLIGHT state. Returns a tuple of:
/// - throttle_thrust_command_lbf
/// - tvc_pitch_command_deg
/// - tvc_yaw_command_deg
/// - FlightControllerMetrics
std::expected<std::tuple<float, float, float, FlightControllerMetrics>, Error>
FlightController::tick(EstimatedState state, float x_command_m, float y_command_m, float z_command_m)
{
    MutexGuard flight_controller_guard(&flight_controller_lock);

    des_state.position.x = x_command_m;
    des_state.position.y = y_command_m;
    des_state.position.z = z_command_m;

    FlightControllerMetrics metrics = FlightControllerMetrics_init_default;

    float z_acceleration = verticalPID(state, metrics);
    float throttle_thrust_command_N = z_acceleration * get_mass_kg() + 9.80665f * get_mass_kg();
    // TODO: make magic number a config constant
    float throttle_thrust_command_lbf = throttle_thrust_command_N * 0.224809f; // convert to lbf
    auto angular_accelerations = lateralPID(state, metrics);
    float pitch_acceleration_command = angular_accelerations[0];
    float yaw_acceleration_command = angular_accelerations[1];

    auto tvc_angles = find_tvc_angles(pitch_acceleration_command, yaw_acceleration_command, throttle_thrust_command_lbf);
    if (!tvc_angles) {
        return std::unexpected(Error::from_cause("failed to compute TVC angles"));
    }
    float tvc_pitch_command_deg = std::get<0>(*tvc_angles);
    float tvc_yaw_command_deg = std::get<1>(*tvc_angles);
    metrics.tvc_pitch_command_deg = tvc_pitch_command_deg;
    metrics.tvc_yaw_command_deg = tvc_yaw_command_deg;

    loopCount++;
    return {{throttle_thrust_command_lbf, tvc_pitch_command_deg, tvc_yaw_command_deg, metrics}};
}

/// Configure controller gains.
std::expected<void, Error> FlightController::handle_configure_gains(const ConfigureFlightControllerGainsRequest& req)
{
    MutexGuard flight_controller_guard(&flight_controller_lock);

    // Update PID gains if provided
    pidXTilt.setGains(
        req.has_pidXTilt_kp ? req.pidXTilt_kp : pidXTilt.getP(),
        req.has_pidXTilt_ki ? req.pidXTilt_ki : pidXTilt.getI(),
        req.has_pidXTilt_kd ? req.pidXTilt_kd : pidXTilt.getD()
    );

    pidYTilt.setGains(
        req.has_pidYTilt_kp ? req.pidYTilt_kp : pidYTilt.getP(),
        req.has_pidYTilt_ki ? req.pidYTilt_ki : pidYTilt.getI(),
        req.has_pidYTilt_kd ? req.pidYTilt_kd : pidYTilt.getD()
    );

    pidX.setGains(
        req.has_pidX_kp ? req.pidX_kp : pidX.getP(),
        req.has_pidX_ki ? req.pidX_ki : pidX.getI(),
        req.has_pidX_kd ? req.pidX_kd : pidX.getD()
    );

    pidY.setGains(
        req.has_pidY_kp ? req.pidY_kp : pidY.getP(),
        req.has_pidY_ki ? req.pidY_ki : pidY.getI(),
        req.has_pidY_kd ? req.pidY_kd : pidY.getD()
    );

    pidZ.setGains(
        req.has_pidZ_kp ? req.pidZ_kp : pidZ.getP(),
        req.has_pidZ_ki ? req.pidZ_ki : pidZ.getI(),
        req.has_pidZ_kd ? req.pidZ_kd : pidZ.getD()
    );

    pidZVelocity.setGains(
        req.has_pidZVelocity_kp ? req.pidZVelocity_kp : pidZVelocity.getP(),
        req.has_pidZVelocity_ki ? req.pidZVelocity_ki : pidZVelocity.getI(),
        req.has_pidZVelocity_kd ? req.pidZVelocity_kd : pidZVelocity.getD()
    );



    if ((req.has_pidXTilt_min_out && req.has_pidXTilt_max_out)) {
        pidXTilt.setOutputLimits(req.pidXTilt_min_out, req.pidXTilt_max_out);
    }
    if ((req.has_pidYTilt_min_out && req.has_pidYTilt_max_out)) {
        pidYTilt.setOutputLimits(req.pidYTilt_min_out, req.pidYTilt_max_out);
    }
    if ((req.has_pidX_min_out && req.has_pidX_max_out)) {
        pidX.setOutputLimits(req.pidX_min_out, req.pidX_max_out);
    }
    if ((req.has_pidY_min_out && req.has_pidY_max_out)) {
        pidY.setOutputLimits(req.pidY_min_out, req.pidY_max_out);
    }
    if ((req.has_pidZ_min_out && req.has_pidZ_max_out)) {
        pidZ.setOutputLimits(req.pidZ_min_out, req.pidZ_max_out);
    }
    if ((req.has_pidZVelocity_min_out && req.has_pidZVelocity_max_out)) {
        pidZVelocity.setOutputLimits(req.pidZVelocity_min_out, req.pidZVelocity_max_out);
    }

    // Update integral limits if provided
    if ((req.has_pidXTilt_min_integral && req.has_pidXTilt_max_integral)) {
        pidXTilt.setIntegralLimits(req.pidXTilt_min_integral, req.pidXTilt_max_integral);
    }
    if ((req.has_pidYTilt_min_integral && req.has_pidYTilt_max_integral)) {
        pidYTilt.setIntegralLimits(req.pidYTilt_min_integral, req.pidYTilt_max_integral);
    }
    if ((req.has_pidX_min_integral && req.has_pidX_max_integral)) {
        pidX.setIntegralLimits(req.pidX_min_integral, req.pidX_max_integral);
    }
    if ((req.has_pidY_min_integral && req.has_pidY_max_integral)) {
        pidY.setIntegralLimits(req.pidY_min_integral, req.pidY_max_integral);
    }
    if ((req.has_pidZ_min_integral && req.has_pidZ_max_integral)) {
        pidZ.setIntegralLimits(req.pidZ_min_integral, req.pidZ_max_integral);
    }
    if ((req.has_pidZVelocity_min_integral && req.has_pidZVelocity_max_integral)) {
        pidZVelocity.setIntegralLimits(req.pidZVelocity_min_integral, req.pidZVelocity_max_integral);
    }

    // Update integral zones if provided
    if (req.has_pidXTilt_integral_zone) {
        pidXTilt.setIntegralZone(req.pidXTilt_integral_zone);
    }
    if (req.has_pidYTilt_integral_zone) {
        pidYTilt.setIntegralZone(req.pidYTilt_integral_zone);
    }
    if (req.has_pidX_integral_zone) {
        pidX.setIntegralZone(req.pidX_integral_zone);
    }
    if (req.has_pidY_integral_zone) {
        pidY.setIntegralZone(req.pidY_integral_zone);
    }
    if (req.has_pidZ_integral_zone) {
        pidZ.setIntegralZone(req.pidZ_integral_zone);
    }
    if (req.has_pidZVelocity_integral_zone) {
        pidZVelocity.setIntegralZone(req.pidZVelocity_integral_zone);
    }

    // Update derivative low-pass filter if provided
    if (req.has_pidXTilt_deriv_lp_hz) {
        pidXTilt.setDerivativeLowPass(req.pidXTilt_deriv_lp_hz);
    }
    if (req.has_pidYTilt_deriv_lp_hz) {
        pidYTilt.setDerivativeLowPass(req.pidYTilt_deriv_lp_hz);
    }
    if (req.has_pidX_deriv_lp_hz) {
        pidX.setDerivativeLowPass(req.pidX_deriv_lp_hz);
    }
    if (req.has_pidY_deriv_lp_hz) {
        pidY.setDerivativeLowPass(req.pidY_deriv_lp_hz);
    }
    if (req.has_pidZ_deriv_lp_hz) {
        pidZ.setDerivativeLowPass(req.pidZ_deriv_lp_hz);
    }
    if (req.has_pidZVelocity_deriv_lp_hz) {
        pidZVelocity.setDerivativeLowPass(req.pidZVelocity_deriv_lp_hz);
    }

    return {};
}

#if CONFIG_TEST
void FlightController::set_desired_state_for_testing(const FlightControllerDesiredState& state) {
    des_state = state;
}

void FlightController::set_loop_count_for_testing(uint32_t count) {
    loopCount = count;
}
#endif
