#pragma once

// Ranger TVC configuration. Every hardware- or tuning-dependent constant for the TVC gimbal lives here, with its unit
// in the name. Values marked TODO(adit) are unknown/unverified; the TVC refuses to enable while any required one is
// still a placeholder (see tvc::config_is_complete()).

#include <cmath>
#include <cstdint>

namespace tvc {

// ---------------------------------------------------------------------------------------------------------------------
// Gimbal geometry (from inversekinematicsactuatorsnewgimbal.m). Same structure for pitch and yaw.
// ---------------------------------------------------------------------------------------------------------------------

/// Vertical distance, gimbal center to structure-side actuator mount height.
constexpr float TVC_X_IN = 11.2120f;
/// Horizontal distance, engine center axis to structure-side actuator mount.
constexpr float TVC_TE_IN = 8.3981f;
/// Along the engine axis, gimbal center to engine-side actuator mount.
constexpr float TVC_E_IN = 6.6941f;
/// Radial, engine centerline to engine-side actuator mount.
constexpr float TVC_ER_IN = 4.349f;

/// Angle between Lts and Lre with the engine centered, pitch axis.
constexpr float TVC_PHI0_DEG = 110.155f;
/// Angle between Lts and Lre with the engine centered, yaw axis.
/// TODO(adit): the supplied TVCyawnew.csv was generated with 110.155 deg (it is identical to TVCpitchnew.csv), but the
/// MATLAB script says 113.696 deg. Using the script value until confirmed; yaw center length differs by ~0.35 in.
constexpr float TVC_PSI0_DEG = 113.696f;

// ---------------------------------------------------------------------------------------------------------------------
// Command limits
// ---------------------------------------------------------------------------------------------------------------------

/// Hard clamp on commanded gimbal angle, per axis, symmetric.
constexpr float TVC_ANGLE_LIMIT_DEG = 12.0f;
/// Slew-rate limit on the commanded gimbal angle, per axis.
/// TODO(adit): tune against actuator capability. Conservative placeholder.
constexpr float TVC_SLEW_LIMIT_DEG_S = 30.0f;
/// Acceleration limit handed to moteus' trajectory generator, expressed as gimbal angle.
/// TODO(adit): tune.
constexpr float TVC_ACCEL_LIMIT_DEG_S2 = 300.0f;
/// moteus velocity limit = slew limit * this margin, so moteus' own trajectory never lags our slew-limited setpoint.
constexpr float TVC_MOTEUS_VELOCITY_MARGIN = 1.5f;
/// Upper bound on |dL/dangle| over +/-12 deg for both axes (computed: 0.1131 in/deg), used to turn angle rates into
/// actuator rates conservatively.
constexpr float TVC_MAX_DL_DANGLE_IN_PER_DEG = 0.115f;

// ---------------------------------------------------------------------------------------------------------------------
// Actuators: length <-> moteus output revolutions
//   rev = (L - L_center) * turns_per_inch * direction_sign
// moteus reports position in *output* revolutions (after motor_position.rotor_to_output_ratio), so turns_per_inch is
// (moteus output revolutions per lead screw revolution) / (lead screw lead, in per rev).
// ---------------------------------------------------------------------------------------------------------------------

struct AxisConfig {
    const char* name;
    /// Center angle between Lts and Lre (phi0 or psi0).
    float theta0_deg;
    /// Actuator length at gimbal center. The actuator is homed here, so 0 rev == this length. Must equal
    /// length_in(0, theta0_deg); a unit test enforces this.
    float l_center_in;
    /// moteus output revolutions per inch of actuator travel. 0 = not configured.
    float turns_per_inch;
    /// +1 or -1 depending on which way positive revolutions move the actuator. 0 = not configured.
    float direction_sign;
    /// moteus CAN ID (moteus `id.id`).
    uint8_t moteus_id;
};

constexpr AxisConfig TVC_PITCH = {
    .name = "pitch",
    .theta0_deg = TVC_PHI0_DEG,
    .l_center_in = 18.3582104f,
    // TODO(adit): lead screw lead (in/rev) and gear ratio -> turns_per_inch. Do not guess.
    .turns_per_inch = 0.0f,
    // TODO(adit): direction (+1/-1). Do not guess.
    .direction_sign = 0.0f,
    .moteus_id = 1,  // Bench-tested ID from prabhu/moteusTest.
};

constexpr AxisConfig TVC_YAW = {
    .name = "yaw",
    .theta0_deg = TVC_PSI0_DEG,
    .l_center_in = 18.7041255f,  // TODO(adit): 18.3582104 if psi0 is really 110.155 (see TVC_PSI0_DEG).
    // TODO(adit): lead screw lead (in/rev) and gear ratio -> turns_per_inch. Do not guess.
    .turns_per_inch = 0.0f,
    // TODO(adit): direction (+1/-1). Do not guess.
    .direction_sign = 0.0f,
    .moteus_id = 2,  // TODO(adit): set yaw moteus `id.id` to 2, or change this.
};

constexpr int TVC_AXIS_COUNT = 2;
constexpr int TVC_AXIS_PITCH = 0;
constexpr int TVC_AXIS_YAW = 1;
constexpr const AxisConfig* TVC_AXES[TVC_AXIS_COUNT] = {&TVC_PITCH, &TVC_YAW};

// ---------------------------------------------------------------------------------------------------------------------
// CAN-FD / moteus
// ---------------------------------------------------------------------------------------------------------------------

/// Our 7-bit source ID on the moteus bus.
constexpr uint8_t TVC_CAN_SOURCE_ID = 0;
/// 13-bit CAN ID prefix (moteus `can.prefix`). 0 unless the controllers were configured otherwise.
constexpr uint16_t TVC_CAN_PREFIX = 0;
/// CAN-FD bit rate switching for the data phase. The devicetree configures 5 Mbps data; the moteus bench test ran with
/// BRS off (data phase at 1 Mbps), which still fits the latency budget (~0.5 ms per 48-byte frame). Flip once BRS has
/// been verified on the vehicle harness.
constexpr bool TVC_CAN_BRS = false;
/// Max torque sent with each position command. NaN = moteus' configured servo.max_current_A limit.
/// TODO(adit): set an explicit limit once the actuator's safe force is known.
constexpr float TVC_MAX_TORQUE_NM = NAN;
/// moteus watchdog: if no command arrives within this time, moteus enters its timeout mode and stops driving.
constexpr float TVC_MOTEUS_WATCHDOG_S = 0.1f;

// ---------------------------------------------------------------------------------------------------------------------
// Timing
// ---------------------------------------------------------------------------------------------------------------------

constexpr uint32_t TVC_LOOP_PERIOD_US = 20'000;  // 50 Hz
/// How long after the last transmit we wait for both replies.
constexpr uint32_t TVC_REPLY_TIMEOUT_US = 5'000;

// ---------------------------------------------------------------------------------------------------------------------
// Fault detection
// ---------------------------------------------------------------------------------------------------------------------

/// Consecutive cycles without a reply from an axis before it is declared faulted.
constexpr uint32_t TVC_MAX_MISSED_REPLIES = 3;
/// Max |commanded - measured| actuator length while enabled.
/// TODO(adit): tune once the actuator's tracking performance is known.
constexpr float TVC_POSITION_ERROR_MAX_IN = 0.10f;
/// Consecutive cycles the position error must exceed the limit before faulting (filters transients).
constexpr uint32_t TVC_POSITION_ERROR_CYCLES = 5;
/// Max |measured length - L_center| allowed when enabling. Guards against enabling an unhomed actuator.
constexpr float TVC_HOMED_TOLERANCE_IN = 0.05f;

enum class FaultPolicy {
    /// Healthy axes servo to gimbal center; axes that are unresponsive or faulted inside moteus are stopped.
    HOLD_CENTER,
    /// Every axis is commanded to moteus stop (mode 0), i.e. unpowered.
    STOP,
};
constexpr FaultPolicy TVC_FAULT_POLICY = FaultPolicy::HOLD_CENTER;

/// If true, a latched TVC fault makes RangerTvc::tick() return an error during active control, which sends the
/// controller to ABORT. TODO(adit): confirm (question was left open).
constexpr bool TVC_FAULT_ABORTS_CONTROLLER = true;

// ---------------------------------------------------------------------------------------------------------------------
// Bench sweep (only used with CONFIG_RANGER_TVC_BENCH_SWEEP)
// ---------------------------------------------------------------------------------------------------------------------

constexpr float TVC_BENCH_SWEEP_AMPLITUDE_DEG = 2.0f;
constexpr float TVC_BENCH_SWEEP_RATE_DEG_S = 0.5f;

/// True when every hardware constant needed to drive the actuators has been filled in.
constexpr bool axis_config_is_complete(const AxisConfig& axis)
{
    return axis.turns_per_inch > 0.0f && (axis.direction_sign == 1.0f || axis.direction_sign == -1.0f) && axis.moteus_id > 0 && axis.moteus_id < 0x80;
}

constexpr bool config_is_complete()
{
    return axis_config_is_complete(TVC_PITCH) && axis_config_is_complete(TVC_YAW);
}

}  // namespace tvc
