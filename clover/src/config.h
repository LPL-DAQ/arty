#ifndef APP_CONFIG_H
#define APP_CONFIG_H

#include <limits>

constexpr int CONTROLLER_STEP_WORK_Q_PRIORITY = -10;
constexpr int ANALOG_SENSORS_THREAD_PRIORITY = -5;
constexpr int LIDAR_1_THREAD_PRIORITY = -5;
constexpr int LIDAR_2_THREAD_PRIORITY = -5;
constexpr int VECTORNAV_THREAD_PRIORITY = -5;
constexpr int GNSS_THREAD_PRIORITY = -5;

// Unit conversion
constexpr float DEG2RAD_F = 0.0174532925f;
constexpr float RAD2DEG_F = 57.2957795f;
constexpr float N_TO_LBF  = 0.224809f;
constexpr float GRAVITY_M_S2 = 9.80665f;

// PWM pulse width range (microseconds): 0% throttle = 1000 µs, 100% throttle = 2000 µs
constexpr uint32_t MIN_PWM_PULSE_US = 1000;
constexpr uint32_t MAX_PWM_PULSE_US = 2000;

// RCS Hornet PWM Throttle
constexpr uint32_t HORNET_RCS_THROTTLE_PERCENT = 1.0f; // 100% throttle corresponds to 2000 µs pulse

// Flight controller outer loop runs every Nth inner-loop tick
constexpr uint32_t FLIGHT_OUTER_LOOP_DIVISOR = 3;

// Inifinity and negative infinity for floats
constexpr float FLOAT_INFINITY = std::numeric_limits<float>::infinity();
constexpr float FLOAT_NEG_INFINITY = -std::numeric_limits<float>::infinity();

// FlightController PID gains
constexpr float FLIGHT_PID_X_TILT_KP = 0.385f;
constexpr float FLIGHT_PID_X_TILT_KI = 0.01f;
constexpr float FLIGHT_PID_X_TILT_KD = 0.44f;

constexpr float FLIGHT_PID_Y_TILT_KP = 0.385f;
constexpr float FLIGHT_PID_Y_TILT_KI = 0.01f;
constexpr float FLIGHT_PID_Y_TILT_KD = 0.44f;

constexpr float FLIGHT_PID_X_KP = 0.2f;
constexpr float FLIGHT_PID_X_KI = 0.1f;
constexpr float FLIGHT_PID_X_KD = 0.05f;

constexpr float FLIGHT_PID_Y_KP = 0.2f;
constexpr float FLIGHT_PID_Y_KI = 0.1f;
constexpr float FLIGHT_PID_Y_KD = 0.05f;

constexpr float FLIGHT_PID_Z_KP = 0.075f;
constexpr float FLIGHT_PID_Z_KI = 0.01f;
constexpr float FLIGHT_PID_Z_KD = 0.0f;

constexpr float FLIGHT_PID_Z_VEL_KP = 0.1f;
constexpr float FLIGHT_PID_Z_VEL_KI = 0.0f;
constexpr float FLIGHT_PID_Z_VEL_KD = 0.0f;

// HornetRcs PID gains
constexpr float HORNET_RCS_ROLL_KP = 2.0f;
constexpr float HORNET_RCS_ROLL_KI = 0.0f;
constexpr float HORNET_RCS_ROLL_KD = 5.0f;


// TODO: fill in with new thrust char
// HornetThrottle thrust curve
constexpr float HORNET_THROTTLE_LOW_THRUST_THRESHOLD_LBF = 10.0f;
constexpr float HORNET_THROTTLE_CROSSOVER_PERCENT       = 0.63f;
// Low-thrust regime: throttle = lbf / HORNET_THROTTLE_LOW_SLOPE
constexpr float HORNET_THROTTLE_LOW_SLOPE               = 15.7166f;
// High-thrust regime: throttle = (lbf + HORNET_THROTTLE_HIGH_OFFSET) / HORNET_THROTTLE_HIGH_SLOPE
constexpr float HORNET_THROTTLE_HIGH_OFFSET             = 7.3123f;
constexpr float HORNET_THROTTLE_HIGH_SLOPE              = 27.321f;

#endif  // APP_CONFIG_H
