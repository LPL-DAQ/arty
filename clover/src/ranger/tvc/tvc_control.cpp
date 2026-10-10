#include "tvc_control.h"
#include "tvc_kinematics.h"

#include <algorithm>
#include <cmath>

namespace tvc {

namespace {

/// Cycles a moteus may report a non-position mode right after enabling (it passes through "preparing" modes 2-4).
constexpr uint32_t WRONG_MODE_GRACE_CYCLES = 3;
/// Give up on homing if moteus has not reported OUTPUT home state at ~0 rev after this many cycles (0.5 s at 50 Hz).
constexpr uint32_t HOMING_TIMEOUT_CYCLES = 25;
/// |position| after "set output exact = 0" that counts as success. moteus sets it exactly; this only absorbs jitter.
constexpr float HOMING_ZERO_TOLERANCE_REV = 0.01f;

bool is_mode(int8_t mode, moteus::Mode m)
{
    return mode == static_cast<int8_t>(m);
}

}  // namespace

Supervisor::Supervisor(const AxisConfig& pitch, const AxisConfig& yaw, float dt_s) : axes_{pitch, yaw}, dt_s_(dt_s)
{
    for (int i = 0; i < TVC_AXIS_COUNT; i++) {
        l_min_in_[i] = length_in(axes_[i], -TVC_ANGLE_LIMIT_DEG);
        l_max_in_[i] = length_in(axes_[i], TVC_ANGLE_LIMIT_DEG);
        last_target_length_in_[i] = axes_[i].l_center_in;
        status_[i].installed = axes_[i].installed;
        if (!axes_[i].installed) {
            status_[i].measured_length_in = NAN;
            status_[i].measured_angle_deg = NAN;
        }
    }
}

float Supervisor::clamp_angle(float deg, float limit_deg, float fallback_deg)
{
    if (!std::isfinite(deg)) {
        return fallback_deg;
    }
    return std::clamp(deg, -limit_deg, limit_deg);
}

float Supervisor::slew(float current, float target, float max_rate_per_s, float dt_s)
{
    const float max_step = max_rate_per_s * dt_s;
    return current + std::clamp(target - current, -max_step, max_step);
}

void Supervisor::ingest(const Inputs& in)
{
    for (int i = 0; i < TVC_AXIS_COUNT; i++) {
        const AxisFeedback& fb = in.feedback[i];
        AxisStatus& st = status_[i];
        const AxisConfig& axis = axes_[i];
        if (!axis.installed) {
            continue;
        }

        // An unparseable or incomplete reply is as good as none.
        st.replied = fb.replied && fb.parsed && fb.query.complete();
        if (!st.replied) {
            st.missed_replies++;
            continue;
        }
        st.missed_replies = 0;

        const moteus::QueryResult& q = fb.query;
        st.mode = q.mode;
        st.home_state = q.home_state;
        st.fault_code = q.fault;
        st.measured_rev = q.position_rev;
        st.velocity_rev_s = q.velocity_rev_s;
        st.torque_nm = q.torque_nm;
        st.voltage_v = q.voltage_v;
        st.temperature_c = q.temperature_c;

        if (axis_config_is_complete(axis)) {
            st.measured_length_in = rev_to_length(axis, q.position_rev);
            st.measured_angle_deg = angle_deg(axis, st.measured_length_in);
        }
        else {
            st.measured_length_in = NAN;
            st.measured_angle_deg = NAN;
        }

        // This reply answers last cycle's frame, so compare against the length that frame commanded.
        st.position_error_in = last_sent_position_[i] ? (last_target_length_in_[i] - st.measured_length_in) : 0.0f;
    }
}

void Supervisor::detect_enabled_faults()
{
    for (int i = 0; i < TVC_AXIS_COUNT; i++) {
        AxisStatus& st = status_[i];
        // Belt and braces: ingest() never counts misses for an uninstalled axis, so it could not fault anyway.
        if (!axes_[i].installed) {
            continue;
        }

        if (st.missed_replies >= TVC_MAX_MISSED_REPLIES) {
            st.faults |= FAULT_NO_REPLY;
        }
        if (!st.replied) {
            continue;
        }

        if (is_mode(st.mode, moteus::Mode::FAULT)) {
            st.faults |= FAULT_MOTEUS_FAULT;
            st.latched_fault_code = st.fault_code;
        }
        else if (is_mode(st.mode, moteus::Mode::TIMEOUT)) {
            st.faults |= FAULT_MOTEUS_TIMEOUT;
        }
        else if (!is_mode(st.mode, moteus::Mode::POSITION)) {
            if (++wrong_mode_cycles_[i] > WRONG_MODE_GRACE_CYCLES) {
                st.faults |= FAULT_UNEXPECTED_MODE;
            }
        }
        else {
            wrong_mode_cycles_[i] = 0;
        }

        if (last_sent_position_[i] && !(std::fabs(st.position_error_in) <= TVC_POSITION_ERROR_MAX_IN)) {
            if (++position_error_cycles_[i] >= TVC_POSITION_ERROR_CYCLES) {
                st.faults |= FAULT_POSITION_ERROR;
            }
        }
        else {
            position_error_cycles_[i] = 0;
        }
    }
}

uint32_t Supervisor::compute_enable_blockers() const
{
    uint32_t blockers = 0;
    if (!axes_[TVC_AXIS_PITCH].installed && !axes_[TVC_AXIS_YAW].installed) {
        return BLOCKED_CONFIG_INCOMPLETE;
    }
    for (int i = 0; i < TVC_AXIS_COUNT; i++) {
        const AxisStatus& st = status_[i];
        const AxisConfig& axis = axes_[i];
        if (!axis.installed) {
            continue;
        }

        if (!axis_config_is_complete(axis)) {
            blockers |= BLOCKED_CONFIG_INCOMPLETE;
        }
        if (!st.replied) {
            blockers |= BLOCKED_NO_REPLY;
            continue;
        }
        if (st.home_state != static_cast<int8_t>(moteus::HomeState::OUTPUT)) {
            blockers |= BLOCKED_NOT_HOMED;
        }
        if (!is_mode(st.mode, moteus::Mode::STOPPED)) {
            blockers |= BLOCKED_NOT_STOPPED;
        }
        // NaN (unconfigured axis) fails this comparison and blocks too.
        if (!(std::fabs(st.measured_length_in - axis.l_center_in) <= TVC_HOMED_TOLERANCE_IN)) {
            blockers |= BLOCKED_NOT_CENTERED;
        }
    }
    return blockers;
}

bool Supervisor::all_installed_replied() const
{
    for (int i = 0; i < TVC_AXIS_COUNT; i++) {
        if (axes_[i].installed && !status_[i].replied) {
            return false;
        }
    }
    return true;
}

bool Supervisor::all_installed_stopped() const
{
    for (int i = 0; i < TVC_AXIS_COUNT; i++) {
        if (axes_[i].installed && !is_mode(status_[i].mode, moteus::Mode::STOPPED)) {
            return false;
        }
    }
    return true;
}

void Supervisor::enter(State s)
{
    if (s == State::ENABLED) {
        for (int i = 0; i < TVC_AXIS_COUNT; i++) {
            // Start the slew limiter where the gimbal actually is (within tolerance of center), so enabling never
            // steps the setpoint.
            slewed_deg_[i] = axes_[i].installed ? status_[i].measured_angle_deg : 0.0f;
            position_error_cycles_[i] = 0;
            wrong_mode_cycles_[i] = 0;
        }
    }
    if (s == State::HOMING || s == State::CLEARING) {
        for (AxisStatus& st : status_) {
            st.faults = 0;
            st.latched_fault_code = 0;
        }
    }
    state_ = s;
    cycles_in_state_ = 0;
}

bool Supervisor::axis_drivable(int i) const
{
    const AxisStatus& st = status_[i];
    return axis_config_is_complete(axes_[i]) && st.missed_replies < TVC_MAX_MISSED_REPLIES && !is_mode(st.mode, moteus::Mode::FAULT) &&
        !is_mode(st.mode, moteus::Mode::TIMEOUT) && st.home_state == static_cast<int8_t>(moteus::HomeState::OUTPUT);
}

void Supervisor::command_axis(int i, float requested_deg, AxisOutput& out)
{
    const AxisConfig& axis = axes_[i];
    AxisStatus& st = status_[i];

    const float clamped_deg = clamp_angle(requested_deg, TVC_ANGLE_LIMIT_DEG, slewed_deg_[i]);
    slewed_deg_[i] = slew(slewed_deg_[i], clamped_deg, TVC_SLEW_LIMIT_DEG_S, dt_s_);

    // Second safety layer: clamp the actuator length itself to the +/-limit envelope.
    const float length = std::clamp(length_in(axis, slewed_deg_[i]), l_min_in_[i], l_max_in_[i]);
    const float rev = length_to_rev(axis, length);

    const float rev_per_deg = TVC_MAX_DL_DANGLE_IN_PER_DEG * axis.turns_per_inch;
    out.action = Action::POSITION;
    out.position = moteus::PositionCommand{
        .position_rev = rev,
        .velocity_rev_s = 0.0f,
        .max_torque_nm = TVC_MAX_TORQUE_NM,
        .watchdog_timeout_s = TVC_MOTEUS_WATCHDOG_S,
        .velocity_limit_rev_s = TVC_SLEW_LIMIT_DEG_S * TVC_MOTEUS_VELOCITY_MARGIN * rev_per_deg,
        .accel_limit_rev_s2 = TVC_ACCEL_LIMIT_DEG_S2 * rev_per_deg,
    };

    st.target_angle_deg = slewed_deg_[i];
    st.target_length_in = length;
    st.target_rev = rev;
    last_target_length_in_[i] = length;
    last_sent_position_[i] = true;
}

void Supervisor::hold_or_stop(int i, AxisOutput& out)
{
    if (TVC_FAULT_POLICY == FaultPolicy::HOLD_CENTER && axis_drivable(i)) {
        command_axis(i, 0.0f, out);
    }
    else {
        out.action = Action::STOP;
    }
}

void Supervisor::step(const Inputs& in, AxisOutput out[TVC_AXIS_COUNT])
{
    // 1. Ingest replies to last cycle's frames.
    ingest(in);

    // 2. Operator requests.
    if (in.home_request && state_ != State::STARTUP && state_ != State::CLEARING) {
        homing_failed_ = false;
        enter(State::HOMING);
    }
    else if (in.clear_faults_request && state_ == State::FAULT) {
        enter(State::CLEARING);
    }

    // With a single-actuator bench build, "all" means every installed axis.
    const bool all_replied = all_installed_replied();

    // 3. Transitions, judged on feedback. cycles_in_state_ counts frames already sent in the current state, so
    //    cycles_in_state_ >= 1 means this feedback answers a frame sent from this state.
    switch (state_) {
    case State::STARTUP:
        // all_replied is vacuously true with no installed axis; never leave STARTUP in that (misconfigured) case.
        if (all_replied && (axes_[TVC_AXIS_PITCH].installed || axes_[TVC_AXIS_YAW].installed)) {
            enter(State::CLEARING);
        }
        break;

    case State::CLEARING:
        if (cycles_in_state_ >= 1 && all_replied && all_installed_stopped()) {
            enter(State::READY);
        }
        break;

    case State::READY:
        enable_blockers_ = compute_enable_blockers();
        if (enable_blockers_ == 0) {
            enter(State::ENABLED);
        }
        break;

    case State::HOMING: {
        // Frames: cycle 0 STOP, cycle 1 SET_OUTPUT_EXACT (+query), then queries.
        if (cycles_in_state_ >= 2 && all_replied) {
            bool homed = true;
            for (int i = 0; i < TVC_AXIS_COUNT; i++) {
                const AxisStatus& st = status_[i];
                homed &= !axes_[i].installed ||
                    (st.home_state == static_cast<int8_t>(moteus::HomeState::OUTPUT) && std::fabs(st.measured_rev) <= HOMING_ZERO_TOLERANCE_REV);
            }
            if (homed) {
                enter(State::READY);
                break;
            }
        }
        if (cycles_in_state_ >= HOMING_TIMEOUT_CYCLES) {
            homing_failed_ = true;
            enter(State::READY);
        }
        break;
    }

    case State::ENABLED:
        if (cycles_in_state_ >= 1) {
            detect_enabled_faults();
            if (status_[0].faults || status_[1].faults) {
                enter(State::FAULT);
            }
        }
        break;

    case State::FAULT:
        break;
    }

    // 4. Frames for this cycle.
    const float requested[TVC_AXIS_COUNT] = {in.pitch_command_deg, in.yaw_command_deg};
    for (int i = 0; i < TVC_AXIS_COUNT; i++) {
        out[i] = AxisOutput{};
        last_sent_position_[i] = false;
        if (!axes_[i].installed) {
            out[i].action = Action::NONE;
            continue;
        }

        switch (state_) {
        case State::STARTUP:
        case State::READY:
            out[i].action = Action::QUERY;
            break;
        case State::CLEARING:
            out[i].action = Action::STOP;
            break;
        case State::HOMING:
            out[i].action = (cycles_in_state_ == 0) ? Action::STOP : (cycles_in_state_ == 1) ? Action::SET_OUTPUT_EXACT : Action::QUERY;
            break;
        case State::ENABLED:
            command_axis(i, requested[i], out[i]);
            break;
        case State::FAULT:
            hold_or_stop(i, out[i]);
            break;
        }
    }

    cycles_in_state_++;
}

void bench_sweep(float t_s, float amplitude_deg, float rate_deg_s, float& pitch_deg, float& yaw_deg)
{
    // One triangle period per axis: 0 -> +A -> -A -> 0 covers 4A degrees.
    const float period_s = 4.0f * amplitude_deg / rate_deg_s;
    const float phase = std::fmod(t_s, 2.0f * period_s);
    const float u = std::fmod(phase, period_s) / period_s;

    float wave;
    if (u < 0.25f) {
        wave = 4.0f * amplitude_deg * u;
    }
    else if (u < 0.75f) {
        wave = 2.0f * amplitude_deg - 4.0f * amplitude_deg * u;
    }
    else {
        wave = 4.0f * amplitude_deg * u - 4.0f * amplitude_deg;
    }

    const bool pitch_phase = phase < period_s;
    pitch_deg = pitch_phase ? wave : 0.0f;
    yaw_deg = pitch_phase ? 0.0f : wave;
}

}  // namespace tvc
