#pragma once

// Ranger TVC supervisor: the per-cycle decision logic of the TVC loop, kept free of Zephyr/CAN so every safety path is
// unit testable on the host. RangerTvc.cpp owns the thread and the bus; each cycle it feeds the replies to the previous
// cycle's frames in, and sends whatever frames this returns.
//
// State machine:
//
//   STARTUP --both reply--> CLEARING --both stopped--> READY --enable checks pass--> ENABLED
//                                                        ^                              |
//                                       home request     |                              | fault detected
//                                   (any state but       |                              v
//                                    STARTUP/CLEARING)   +---- HOMING <-------------- FAULT (latched)
//
// READY only enables when: config is complete (no TODO placeholders), both axes reply, both report moteus home state
// OUTPUT (set by a "set output exact" since moteus last powered up), both are stopped and unfaulted, and both measured
// lengths are within TVC_HOMED_TOLERANCE_IN of L_center. So position mode is never entered on an unhomed actuator.
//
// HOMING assumes the operator has mechanically centered the gimbal. It stops both controllers, writes "set output exact
// = 0" so moteus defines the current position as center, then waits for the home state to read back as OUTPUT.

#include "moteus.h"
#include "tvc_config.h"

#include <cstdint>

namespace tvc {

enum class State : uint8_t {
    STARTUP = 0,
    CLEARING = 1,
    READY = 2,
    HOMING = 3,
    ENABLED = 4,
    FAULT = 5,
};

/// Per-axis fault bits. Latched while in FAULT.
enum FaultBits : uint32_t {
    FAULT_NO_REPLY = 1u << 0,         // TVC_MAX_MISSED_REPLIES consecutive cycles without a reply.
    FAULT_MOTEUS_FAULT = 1u << 1,     // moteus mode 1; see fault_code.
    FAULT_MOTEUS_TIMEOUT = 1u << 2,   // moteus watchdog expired (mode 11).
    FAULT_UNEXPECTED_MODE = 1u << 3,  // Not in position mode while enabled.
    FAULT_POSITION_ERROR = 1u << 4,   // |commanded - measured| length over limit for too long.
};

/// Why READY has not enabled yet (bitmask, for telemetry).
enum EnableBlockers : uint32_t {
    BLOCKED_CONFIG_INCOMPLETE = 1u << 0,
    BLOCKED_NO_REPLY = 1u << 1,
    BLOCKED_NOT_HOMED = 1u << 2,
    BLOCKED_NOT_CENTERED = 1u << 3,
    BLOCKED_NOT_STOPPED = 1u << 4,
};

enum class Action : uint8_t {
    QUERY = 0,
    STOP = 1,
    POSITION = 2,
    SET_OUTPUT_EXACT = 3,
};

/// Reply to last cycle's frame for one axis.
struct AxisFeedback {
    bool replied = false;
    bool parsed = false;
    moteus::QueryResult query;
};

struct Inputs {
    /// Latest gimbal command from GNC/traces.
    float pitch_command_deg = 0.0f;
    float yaw_command_deg = 0.0f;
    /// Rising-edge requests; consumed this cycle.
    bool home_request = false;
    bool clear_faults_request = false;
    AxisFeedback feedback[TVC_AXIS_COUNT];
};

/// What to send to one axis this cycle.
struct AxisOutput {
    Action action = Action::QUERY;
    moteus::PositionCommand position;  // Valid when action == POSITION.
};

/// Per-axis state exposed for telemetry.
struct AxisStatus {
    // Command chain (valid when a POSITION frame was sent this cycle).
    float target_angle_deg = 0.0f;
    float target_length_in = 0.0f;
    float target_rev = 0.0f;
    // Measurement chain, from the latest reply.
    float measured_rev = 0.0f;
    float measured_length_in = 0.0f;
    float measured_angle_deg = 0.0f;
    float velocity_rev_s = 0.0f;
    float torque_nm = 0.0f;
    float voltage_v = 0.0f;
    float temperature_c = 0.0f;
    int8_t mode = -1;
    int8_t home_state = -1;
    int8_t fault_code = 0;
    float position_error_in = 0.0f;
    uint32_t missed_replies = 0;
    uint32_t faults = 0;
    /// moteus fault code captured when FAULT_MOTEUS_FAULT latched (the live code is cleared once we stop the axis).
    int8_t latched_fault_code = 0;
    /// True if the latest reply parsed and contained every queried register. Unparseable replies count as missed.
    bool replied = false;
};

class Supervisor {
public:
    explicit Supervisor(const AxisConfig& pitch = TVC_PITCH, const AxisConfig& yaw = TVC_YAW, float dt_s = TVC_LOOP_PERIOD_US * 1e-6f);

    /// Run one cycle: ingest feedback, advance the state machine, fill `out` with this cycle's frames.
    void step(const Inputs& in, AxisOutput out[TVC_AXIS_COUNT]);

    State state() const
    {
        return state_;
    }
    const AxisStatus& axis(int i) const
    {
        return status_[i];
    }
    uint32_t enable_blockers() const
    {
        return enable_blockers_;
    }
    bool homing_failed() const
    {
        return homing_failed_;
    }
    bool fault_latched() const
    {
        return state_ == State::FAULT;
    }

    /// Angle clamp + non-finite rejection. Returns `fallback_deg` for NaN/inf.
    static float clamp_angle(float deg, float limit_deg, float fallback_deg);
    /// Moves `current` toward `target` by at most rate * dt.
    static float slew(float current, float target, float max_rate_per_s, float dt_s);

private:
    void ingest(const Inputs& in);
    void detect_enabled_faults();
    uint32_t compute_enable_blockers() const;
    void enter(State s);
    void command_axis(int i, float requested_deg, AxisOutput& out);
    void hold_or_stop(int i, AxisOutput& out);
    bool axis_drivable(int i) const;

    AxisConfig axes_[TVC_AXIS_COUNT];
    float dt_s_;
    float l_min_in_[TVC_AXIS_COUNT];
    float l_max_in_[TVC_AXIS_COUNT];

    State state_ = State::STARTUP;
    uint32_t cycles_in_state_ = 0;
    uint32_t enable_blockers_ = 0;
    bool homing_failed_ = false;

    float slewed_deg_[TVC_AXIS_COUNT] = {};
    float last_target_length_in_[TVC_AXIS_COUNT] = {};
    bool last_sent_position_[TVC_AXIS_COUNT] = {};
    uint32_t position_error_cycles_[TVC_AXIS_COUNT] = {};
    uint32_t wrong_mode_cycles_[TVC_AXIS_COUNT] = {};
    AxisStatus status_[TVC_AXIS_COUNT];
};

/// Triangle-wave bench sweep: pitch 0 -> +A -> -A -> 0, then yaw the same, repeating. Used with
/// CONFIG_RANGER_TVC_BENCH_SWEEP only.
void bench_sweep(float t_s, float amplitude_deg, float rate_deg_s, float& pitch_deg, float& yaw_deg);

}  // namespace tvc
