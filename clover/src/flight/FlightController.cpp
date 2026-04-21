#include "FlightController.h"
#include "../MutexGuard.h"
#include "../PID.h"
#include "../math_util.h"
#include "../config.h"
#include <zephyr/kernel.h>
#include <cmath>


K_MUTEX_DEFINE(flight_controller_lock);

static const float maxGimble = 12.0;   // degrees, this is an estimate
static const float maxTiltRad = 8.0f * DEG2RAD_F;  // 8 degrees in radians – max intentional tilt

// TODO: Tune, including adding integral terms (is requried)
// reference: kp, ki, kd, min out, max out, min integral, max integral, integral zone, deriv filter
// integral cant command more than 1/3rd output range, with 10 Hz derivative lowpass
static PID pidXTilt(FLIGHT_PID_X_TILT_KP, FLIGHT_PID_X_TILT_KI, FLIGHT_PID_X_TILT_KD, FLOAT_NEG_INFINITY, FLOAT_INFINITY, -maxGimble / 3, maxGimble / 3, FLOAT_INFINITY, 10.0);   // about the X axis
static PID pidYTilt(FLIGHT_PID_Y_TILT_KP, FLIGHT_PID_Y_TILT_KI, FLIGHT_PID_Y_TILT_KD, FLOAT_NEG_INFINITY, FLOAT_INFINITY, -maxGimble / 3, maxGimble / 3, FLOAT_INFINITY, 10.0);   // about the Y axis
static PID pidX(FLIGHT_PID_X_KP, FLIGHT_PID_X_KI, FLIGHT_PID_X_KD);              // needs tuning, these are complete guesses
static PID pidY(FLIGHT_PID_Y_KP, FLIGHT_PID_Y_KI, FLIGHT_PID_Y_KD);              // needs tuning, these are complete guesses
// only use integral within 5 cm of target.
static PID pidZ(FLIGHT_PID_Z_KP, FLIGHT_PID_Z_KI, FLIGHT_PID_Z_KD, FLOAT_NEG_INFINITY, FLOAT_INFINITY, FLOAT_NEG_INFINITY, FLOAT_INFINITY, 0.05);
// TODO: add max and min out
static PID pidZVelocity(FLIGHT_PID_Z_VEL_KP, FLIGHT_PID_Z_VEL_KI, FLIGHT_PID_Z_VEL_KD);      // needs tuning

static uint32_t loopCount = 0;

static float dt = 0.01; // TODO: make this an actual DT measurement ( or at least research if i should)

static FlightControllerDesiredState des_state = FlightControllerDesiredState_init_default;


// returns {pitch acceleration, yaw acceleration}
static std::pair<float, float> lateralPID(EstimatedState state, FlightControllerMetrics& metrics)
{
    std::pair<float, float> output_accelerations{};

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
    output_accelerations.first  = pidXTilt.calculate(0.0f, axis_error_b.x, dt);
    output_accelerations.second = pidYTilt.calculate(0.0f, axis_error_b.y, dt);

    metrics.desired_world_tilt_x_rad = des_state.world_tilt_x;
    metrics.desired_world_tilt_y_rad = des_state.world_tilt_y;
    metrics.commanded_pitch_acceleration_rad_s2 = output_accelerations.first;
    metrics.commanded_yaw_acceleration_rad_s2 = output_accelerations.second;

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

/// Called every tick in FLIGHT state. Returns a tuple of:
/// - pitch_angular_accel_rad_s2
/// - yaw_angular_accel_rad_s2
/// - z_accel_m_s2
/// - FlightControllerMetrics
std::expected<std::tuple<float, float, float, FlightControllerMetrics>, Error>
FlightController::tick(EstimatedState state, float x_command_m, float y_command_m, float z_command_m)
{
    MutexGuard flight_controller_guard(&flight_controller_lock);

    des_state.position.x = x_command_m;
    des_state.position.y = y_command_m;
    des_state.position.z = z_command_m;

    FlightControllerMetrics metrics = FlightControllerMetrics_init_default;

    float z_accel_m_s2 = verticalPID(state, metrics) + GRAVITY_M_S2;
    auto angular_accelerations = lateralPID(state, metrics);
    float pitch_accel_rad_s2 = angular_accelerations.first;
    float yaw_accel_rad_s2 = angular_accelerations.second;

    loopCount++;
    return {{pitch_accel_rad_s2, yaw_accel_rad_s2, z_accel_m_s2, metrics}};
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
