#include "FlightController.h"
#include "MutexGuard.h"
#include "PID.h"
#include <zephyr/kernel.h>
#include <limits>

K_MUTEX_DEFINE(flight_controller_lock);

namespace {
    const float maxGimble = 12.0;   // degrees, this is an estimate
    const float maxTiltDeg = 8.0;  // What we dont want the rocket to purposefully tilt more than

    // TODO: Tune, including adding integral terms (is requried)
    // reference: kp, ki, kd, min out, max out, min integral, max integral, integral zone, deriv filter
    // integral cant command more than 1/3rd output range, with 10 Hz derivative lowpass
    PID pidXTilt(0.385, 0.0, 0.44, -1e6, 1e6, -maxGimble / 3, maxGimble / 3, std::numeric_limits<double>::infinity(), 10.0);   // about the X axis
    PID pidYTilt(0.385, 0.0, 0.44, -1e6, 1e6, -maxGimble / 3, maxGimble / 3, std::numeric_limits<double>::infinity(), 10.0);   // about the Y axis
    PID pidX(0, 0, 0);              // needs tuning
    PID pidY(0, 0, 0);              // needs tuning
    // only use integral within 5 cm of target.
    PID pidZ(0.075, 0.01, 0, -1e6, 1e6, -1e6, 1e6, 0.05);
    // TODO: add max and min out
    PID pidZVelocity(0, 0, 0);      // needs tuning

    static uint32_t loopCount = 0;

    float dt = 0.001; // TODO: make this an actual DT measurement ( or at least research if i should)

    constexpr float DEG2RAD_F = 0.0174532925f;
    constexpr float RAD2DEG_F = 57.2957795f;

    DesiredState des_state = DesiredState_init_default;

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
static std::expected<std::tuple<float, float>, Error> find_tvc_angles(float yaw_accel, float pitch_accel, float thrust){
    // Physics: Angular acceleration = (Thrust * moment_arm * sin(angle)) / MOI
    // Therefore: sin(angle) = (angular_accel * MOI) / (Thrust * moment_arm)
    // angle_rad = arcsin((angular_accel * MOI) / (Thrust * moment_arm))

    // Use nominal thrust based on vehicle mass and a typical thrust-to-weight ratio (~3:1)
    float mass_kg = get_mass_kg();

    // Calculate sin of gimbal angles
    float yaw_sin_angle = (yaw_accel * yaw_MOI) / (thrust * yaw_moment_arm);
    float pitch_sin_angle = (pitch_accel * pitch_MOI) / (thrust * pitch_moment_arm);

    // Clamp sin values to valid domain [-1, 1]
    yaw_sin_angle = util::clamp(yaw_sin_angle, -1.0f, 1.0f);
    pitch_sin_angle = util::clamp(pitch_sin_angle, -1.0f, 1.0f);

    // Calculate gimbal angles in radians using arcsin
    float yaw_angle_rad = std::asin(yaw_sin_angle);
    float pitch_angle_rad = std::asin(pitch_sin_angle);

    // Convert from radians to degrees
    float yaw_angle_deg = yaw_angle_rad * RAD2DEG_F;
    float pitch_angle_deg = pitch_angle_rad * RAD2DEG_F;

    // Clamp to maximum gimbal angle
    yaw_angle_deg = util::clamp(yaw_angle_deg, -maxGimble, maxGimble);
    pitch_angle_deg = util::clamp(pitch_angle_deg, -maxGimble, maxGimble);

    return {{pitch_angle_deg, yaw_angle_deg}};
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
    des_state = DesiredState_init_default;
}

/// Called every tick in FLIGHT state. Returns a tuple of:
/// - throttle_thrust_command_lbf
/// - tvc_pitch_command_deg
/// - tvc_yaw_command_deg
/// - FlightControllerMetrics
std::expected<std::tuple<float, float, float, FlightControllerMetrics>, Error>
FlightController::tick(float x_command_m, float y_command_m, float z_command_m)
{
    MutexGuard flight_controller_guard(&flight_controller_lock);

    float throttle_thrust_command_lbf = -67.67f;
    float (tvc_pitch_command_deg,tvc_yaw_command_deg) = find_tvc_angles(-67.67f,-67.67f);
    FlightControllerMetrics metrics = FlightControllerMetrics_init_default;


    return {{throttle_thrust_command_lbf, tvc_pitch_command_deg, tvc_yaw_command_deg, metrics}};
}

/// Configure controller gains.
std::expected<void, Error> FlightController::handle_configure_gains(const ConfigureFlightControllerGainsRequest& req)
{
    MutexGuard flight_controller_guard(&flight_controller_lock);

    // Update PID gains if provided
    pidXTilt.setGains(
        req.has_pidXTilt_kp() ? req.pidXTilt_kp() : pidXTilt.getP(),
        req.has_pidXTilt_ki() ? req.pidXTilt_ki() : pidXTilt.getI(),
        req.has_pidXTilt_kd() ? req.pidXTilt_kd() : pidXTilt.getD()
    );

    pidYTilt.setGains(
        req.has_pidYTilt_kp() ? req.pidYTilt_kp() : pidYTilt.getP(),
        req.has_pidYTilt_ki() ? req.pidYTilt_ki() : pidYTilt.getI(),
        req.has_pidYTilt_kd() ? req.pidYTilt_kd() : pidYTilt.getD()
    );

    pidX.setGains(
        req.has_pidX_kp() ? req.pidX_kp() : pidX.getP(),
        req.has_pidX_ki() ? req.pidX_ki() : pidX.getI(),
        req.has_pidX_kd() ? req.pidX_kd() : pidX.getD()
    );

    pidY.setGains(
        req.has_pidY_kp() ? req.pidY_kp() : pidY.getP(),
        req.has_pidY_ki() ? req.pidY_ki() : pidY.getI(),
        req.has_pidY_kd() ? req.pidY_kd() : pidY.getD()
    );

    pidZ.setGains(
        req.has_pidZ_kp() ? req.pidZ_kp() : pidZ.getP(),
        req.has_pidZ_ki() ? req.pidZ_ki() : pidZ.getI(),
        req.has_pidZ_kd() ? req.pidZ_kd() : pidZ.getD()
    );

    pidZVelocity.setGains(
        req.has_pidZVelocity_kp() ? req.pidZVelocity_kp() : pidZVelocity.getP(),
        req.has_pidZVelocity_ki() ? req.pidZVelocity_ki() : pidZVelocity.getI(),
        req.has_pidZVelocity_kd() ? req.pidZVelocity_kd() : pidZVelocity.getD()
    );



    if ((req.has_pidXTilt_min_out() && req.has_pidXTilt_max_out())) {
        pidXTilt.setOutputLimits(req.pidXTilt_min_out(), req.pidXTilt_max_out());
    }
    if ((req.has_pidYTilt_min_out() && req.has_pidYTilt_max_out())) {
        pidYTilt.setOutputLimits(req.pidYTilt_min_out(), req.pidYTilt_max_out());
    }
    if ((req.has_pidX_min_out() && req.has_pidX_max_out())) {
        pidX.setOutputLimits(req.pidX_min_out(), req.pidX_max_out());
    }
    if ((req.has_pidY_min_out() && req.has_pidY_max_out())) {
        pidY.setOutputLimits(req.pidY_min_out(), req.pidY_max_out());
    }
    if ((req.has_pidZ_min_out() && req.has_pidZ_max_out())) {
        pidZ.setOutputLimits(req.pidZ_min_out(), req.pidZ_max_out());
    }
    if ((req.has_pidZVelocity_min_out() && req.has_pidZVelocity_max_out())) {
        pidZVelocity.setOutputLimits(req.pidZVelocity_min_out(), req.pidZVelocity_max_out());
    }

    // Update integral limits if provided
    if ((req.has_pidXTilt_min_integral() && req.has_pidXTilt_max_integral())) {
        pidXTilt.setIntegralLimits(req.pidXTilt_min_integral(), req.pidXTilt_max_integral());
    }
    if ((req.has_pidYTilt_min_integral() && req.has_pidYTilt_max_integral())) {
        pidYTilt.setIntegralLimits(req.pidYTilt_min_integral(), req.pidYTilt_max_integral());
    }
    if ((req.has_pidX_min_integral() && req.has_pidX_max_integral())) {
        pidX.setIntegralLimits(req.pidX_min_integral(), req.pidX_max_integral());
    }
    if ((req.has_pidY_min_integral() && req.has_pidY_max_integral())) {
        pidY.setIntegralLimits(req.pidY_min_integral(), req.pidY_max_integral());
    }
    if ((req.has_pidZ_min_integral() && req.has_pidZ_max_integral())) {
        pidZ.setIntegralLimits(req.pidZ_min_integral(), req.pidZ_max_integral());
    }
    if ((req.has_pidZVelocity_min_integral() && req.has_pidZVelocity_max_integral())) {
        pidZVelocity.setIntegralLimits(req.pidZVelocity_min_integral(), req.pidZVelocity_max_integral());
    }

    // Update integral zones if provided
    if (req.has_pidXTilt_integral_zone()) {
        pidXTilt.setIntegralZone(req.pidXTilt_integral_zone());
    }
    if (req.has_pidYTilt_integral_zone()) {
        pidYTilt.setIntegralZone(req.pidYTilt_integral_zone());
    }
    if (req.has_pidX_integral_zone()) {
        pidX.setIntegralZone(req.pidX_integral_zone());
    }
    if (req.has_pidY_integral_zone()) {
        pidY.setIntegralZone(req.pidY_integral_zone());
    }
    if (req.has_pidZ_integral_zone()) {
        pidZ.setIntegralZone(req.pidZ_integral_zone());
    }
    if (req.has_pidZVelocity_integral_zone()) {
        pidZVelocity.setIntegralZone(req.pidZVelocity_integral_zone());
    }

    // Update derivative low-pass filter if provided
    if (req.has_pidXTilt_deriv_lp_hz()) {
        pidXTilt.setDerivativeLowPass(req.pidXTilt_deriv_lp_hz());
    }
    if (req.has_pidYTilt_deriv_lp_hz()) {
        pidYTilt.setDerivativeLowPass(req.pidYTilt_deriv_lp_hz());
    }
    if (req.has_pidX_deriv_lp_hz()) {
        pidX.setDerivativeLowPass(req.pidX_deriv_lp_hz());
    }
    if (req.has_pidY_deriv_lp_hz()) {
        pidY.setDerivativeLowPass(req.pidY_deriv_lp_hz());
    }
    if (req.has_pidZ_deriv_lp_hz()) {
        pidZ.setDerivativeLowPass(req.pidZ_deriv_lp_hz());
    }
    if (req.has_pidZVelocity_deriv_lp_hz()) {
        pidZVelocity.setDerivativeLowPass(req.pidZVelocity_deriv_lp_hz());
    }

    return {};
}
