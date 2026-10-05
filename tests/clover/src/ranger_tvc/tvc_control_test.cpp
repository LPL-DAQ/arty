#include "ranger/tvc/tvc_control.h"
#include "ranger/tvc/tvc_kinematics.h"

#include <cmath>
#include <zephyr/ztest.h>

using namespace tvc;

namespace {

// Complete configs for testing (the shipped ones carry TODO placeholders and must never enable).
constexpr AxisConfig TEST_PITCH = {
    .name = "pitch",
    .theta0_deg = TVC_PHI0_DEG,
    .l_center_in = 18.3582104f,
    .turns_per_inch = 5.0f,
    .direction_sign = 1.0f,
    .moteus_id = 1,
};
constexpr AxisConfig TEST_YAW = {
    .name = "yaw",
    .theta0_deg = TVC_PSI0_DEG,
    .l_center_in = 18.7041255f,
    .turns_per_inch = 5.0f,
    .direction_sign = -1.0f,
    .moteus_id = 2,
};
constexpr float DT_S = TVC_LOOP_PERIOD_US * 1e-6f;

/// Simulated moteus controller: reacts to the frames the supervisor sends and produces the next reply.
struct FakeMoteus {
    int8_t mode = static_cast<int8_t>(moteus::Mode::STOPPED);
    int8_t home_state = static_cast<int8_t>(moteus::HomeState::ROTOR);
    int8_t fault = 0;
    float position_rev = 0.0f;
    bool responsive = true;
    bool stuck = false;  // Ignores position setpoints (jammed actuator).

    AxisFeedback apply(const AxisOutput& out)
    {
        switch (out.action) {
        case Action::QUERY:
            break;
        case Action::STOP:
            mode = static_cast<int8_t>(moteus::Mode::STOPPED);
            fault = 0;
            break;
        case Action::SET_OUTPUT_EXACT:
            position_rev = 0.0f;
            home_state = static_cast<int8_t>(moteus::HomeState::OUTPUT);
            break;
        case Action::POSITION:
            if (mode == static_cast<int8_t>(moteus::Mode::STOPPED) || mode == static_cast<int8_t>(moteus::Mode::POSITION)) {
                mode = static_cast<int8_t>(moteus::Mode::POSITION);
                if (!stuck) {
                    position_rev = out.position.position_rev;
                }
            }
            break;
        }

        AxisFeedback fb;
        fb.replied = responsive;
        fb.parsed = responsive;
        if (responsive) {
            fb.query.mode = mode;
            fb.query.position_rev = position_rev;
            fb.query.home_state = home_state;
            fb.query.fault = fault;
            fb.query.voltage_v = 24.0f;
            fb.query.temperature_c = 30.0f;
            fb.query.present = 0xffffffffu;
        }
        return fb;
    }
};

struct Harness {
    Supervisor sup{TEST_PITCH, TEST_YAW, DT_S};
    FakeMoteus motor[TVC_AXIS_COUNT];
    Inputs in;
    AxisOutput out[TVC_AXIS_COUNT];

    void cycle()
    {
        sup.step(in, out);
        in.home_request = false;
        in.clear_faults_request = false;
        for (int i = 0; i < TVC_AXIS_COUNT; i++) {
            in.feedback[i] = motor[i].apply(out[i]);
        }
    }

    void cycles(int n)
    {
        for (int k = 0; k < n; k++) {
            cycle();
        }
    }

    /// Boot, home, and run until enabled.
    void enable()
    {
        cycles(5);
        in.home_request = true;
        cycles(10);
        zassert_equal(sup.state(), State::ENABLED, "harness failed to enable (blockers 0x%x)", sup.enable_blockers());
    }
};

}  // namespace

ZTEST(TvcControl_tests, test_clamp_angle)
{
    zassert_equal(Supervisor::clamp_angle(5.0f, 12.0f, 0.0f), 5.0f);
    zassert_equal(Supervisor::clamp_angle(20.0f, 12.0f, 0.0f), 12.0f);
    zassert_equal(Supervisor::clamp_angle(-20.0f, 12.0f, 0.0f), -12.0f);
    zassert_equal(Supervisor::clamp_angle(NAN, 12.0f, 3.0f), 3.0f);
    zassert_equal(Supervisor::clamp_angle(INFINITY, 12.0f, -1.0f), -1.0f);
}

ZTEST(TvcControl_tests, test_slew)
{
    zassert_within(Supervisor::slew(0.0f, 10.0f, 30.0f, 0.02f), 0.6f, 1e-6f);
    zassert_within(Supervisor::slew(0.0f, -10.0f, 30.0f, 0.02f), -0.6f, 1e-6f);
    zassert_within(Supervisor::slew(0.0f, 0.3f, 30.0f, 0.02f), 0.3f, 1e-6f);
}

ZTEST(TvcControl_tests, test_startup_sequence_reaches_ready_without_driving)
{
    Harness h;
    h.cycle();
    zassert_equal(h.sup.state(), State::STARTUP);
    zassert_equal(h.out[0].action, Action::QUERY);
    h.cycle();  // Both replied -> CLEARING, sends STOP.
    zassert_equal(h.sup.state(), State::CLEARING);
    zassert_equal(h.out[0].action, Action::STOP);
    zassert_equal(h.out[1].action, Action::STOP);
    h.cycle();  // STOP answered -> READY.
    zassert_equal(h.sup.state(), State::READY);

    // Not homed (moteus boots with home state ROTOR): must never enable or send a position command.
    for (int k = 0; k < 50; k++) {
        h.cycle();
        zassert_equal(h.sup.state(), State::READY);
        zassert_not_equal(h.out[0].action, Action::POSITION);
        zassert_not_equal(h.out[1].action, Action::POSITION);
    }
    zassert_true(h.sup.enable_blockers() & BLOCKED_NOT_HOMED);
}

ZTEST(TvcControl_tests, test_startup_waits_for_both_controllers)
{
    Harness h;
    h.motor[1].responsive = false;
    h.cycles(20);
    zassert_equal(h.sup.state(), State::STARTUP);
    h.motor[1].responsive = true;
    h.cycles(3);
    zassert_equal(h.sup.state(), State::READY);
}

ZTEST(TvcControl_tests, test_homing_sequence_then_enable)
{
    Harness h;
    h.cycles(5);
    h.in.home_request = true;
    h.cycle();
    zassert_equal(h.sup.state(), State::HOMING);
    zassert_equal(h.out[0].action, Action::STOP);
    h.cycle();
    zassert_equal(h.out[0].action, Action::SET_OUTPUT_EXACT);
    zassert_equal(h.out[1].action, Action::SET_OUTPUT_EXACT);
    h.cycle();  // Reply to SET_OUTPUT_EXACT shows OUTPUT home at 0 -> READY.
    zassert_equal(h.sup.state(), State::READY);
    h.cycle();  // Enable checks pass -> ENABLED, first position command.
    zassert_equal(h.sup.state(), State::ENABLED);
    zassert_equal(h.out[0].action, Action::POSITION);
    zassert_within(h.out[0].position.position_rev, 0.0f, 1e-4f);
    zassert_false(h.sup.homing_failed());
}

ZTEST(TvcControl_tests, test_shipped_config_never_enables)
{
    Supervisor sup;  // TVC_PITCH / TVC_YAW with TODO(adit) placeholders.
    FakeMoteus motor[TVC_AXIS_COUNT];
    for (FakeMoteus& m : motor) {
        m.home_state = static_cast<int8_t>(moteus::HomeState::OUTPUT);
    }
    Inputs in;
    AxisOutput out[TVC_AXIS_COUNT];
    for (int k = 0; k < 50; k++) {
        sup.step(in, out);
        for (int i = 0; i < TVC_AXIS_COUNT; i++) {
            zassert_not_equal(out[i].action, Action::POSITION);
            in.feedback[i] = motor[i].apply(out[i]);
        }
    }
    zassert_equal(sup.state(), State::READY);
    zassert_true(sup.enable_blockers() & BLOCKED_CONFIG_INCOMPLETE);
}

ZTEST(TvcControl_tests, test_off_center_actuator_blocks_enable)
{
    Harness h;
    for (FakeMoteus& m : h.motor) {
        m.home_state = static_cast<int8_t>(moteus::HomeState::OUTPUT);
    }
    h.motor[0].position_rev = 0.5f;  // 0.1 in off center at 5 turns/in, over the 0.05 in tolerance.
    h.cycles(20);
    zassert_equal(h.sup.state(), State::READY);
    zassert_true(h.sup.enable_blockers() & BLOCKED_NOT_CENTERED);

    h.motor[0].position_rev = 0.1f;  // 0.02 in, within tolerance.
    h.cycles(2);
    zassert_equal(h.sup.state(), State::ENABLED);
}

ZTEST(TvcControl_tests, test_commands_follow_ik_and_direction)
{
    Harness h;
    h.enable();
    h.in.pitch_command_deg = 3.0f;
    h.in.yaw_command_deg = -3.0f;
    h.cycles(20);  // 3 deg at 30 deg/s needs 5 cycles.

    const AxisStatus& p = h.sup.axis(TVC_AXIS_PITCH);
    zassert_within(p.target_angle_deg, 3.0f, 1e-5f);
    zassert_within(p.target_length_in, length_in(TEST_PITCH, 3.0f), 1e-5f);
    zassert_within(h.out[0].position.position_rev, (length_in(TEST_PITCH, 3.0f) - TEST_PITCH.l_center_in) * 5.0f, 1e-3f);

    // Yaw direction_sign = -1 flips the revolution sign.
    zassert_within(h.out[1].position.position_rev, -(length_in(TEST_YAW, -3.0f) - TEST_YAW.l_center_in) * 5.0f, 1e-3f);
    zassert_true(h.out[1].position.position_rev > 0.0f);

    // Measured chain back-computes the commanded angle (ideal fake plant).
    zassert_within(h.sup.axis(TVC_AXIS_YAW).measured_angle_deg, -3.0f, 5e-3f);

    // Watchdog and limits are always populated.
    zassert_equal(h.out[0].position.watchdog_timeout_s, TVC_MOTEUS_WATCHDOG_S);
    zassert_true(h.out[0].position.velocity_limit_rev_s > 0.0f);
    zassert_true(h.out[0].position.accel_limit_rev_s2 > 0.0f);
}

ZTEST(TvcControl_tests, test_slew_limits_step_command)
{
    Harness h;
    h.enable();
    h.in.pitch_command_deg = 10.0f;
    float prev = h.sup.axis(TVC_AXIS_PITCH).target_angle_deg;
    for (int k = 0; k < 10; k++) {
        h.cycle();
        const float now = h.sup.axis(TVC_AXIS_PITCH).target_angle_deg;
        zassert_true(now - prev <= TVC_SLEW_LIMIT_DEG_S * DT_S + 1e-5f, "slew exceeded: %f -> %f", (double)prev, (double)now);
        // The actuator command must follow the slewed angle, not the raw request.
        zassert_within(h.sup.axis(TVC_AXIS_PITCH).target_length_in, length_in(TEST_PITCH, now), 1e-5f);
        prev = now;
    }
    zassert_true(prev < 10.0f);
}

ZTEST(TvcControl_tests, test_angle_and_length_clamped)
{
    Harness h;
    h.enable();
    h.in.pitch_command_deg = 45.0f;
    h.in.yaw_command_deg = -45.0f;
    h.cycles(100);
    zassert_within(h.sup.axis(TVC_AXIS_PITCH).target_angle_deg, TVC_ANGLE_LIMIT_DEG, 1e-5f);
    zassert_within(h.sup.axis(TVC_AXIS_YAW).target_angle_deg, -TVC_ANGLE_LIMIT_DEG, 1e-5f);
    zassert_true(h.sup.axis(TVC_AXIS_PITCH).target_length_in <= length_in(TEST_PITCH, TVC_ANGLE_LIMIT_DEG) + 1e-5f);
    zassert_true(h.sup.axis(TVC_AXIS_YAW).target_length_in >= length_in(TEST_YAW, -TVC_ANGLE_LIMIT_DEG) - 1e-5f);
    zassert_equal(h.sup.state(), State::ENABLED);
}

ZTEST(TvcControl_tests, test_nan_command_holds_previous)
{
    Harness h;
    h.enable();
    h.in.pitch_command_deg = 2.0f;
    h.cycles(10);
    h.in.pitch_command_deg = NAN;
    h.cycles(5);
    zassert_within(h.sup.axis(TVC_AXIS_PITCH).target_angle_deg, 2.0f, 1e-5f);
    zassert_false(std::isnan(h.out[0].position.position_rev));
}

ZTEST(TvcControl_tests, test_missed_replies_fault_holds_center_on_healthy_axis)
{
    Harness h;
    h.enable();
    h.in.pitch_command_deg = 4.0f;
    h.cycles(20);

    // Feedback lags one cycle: after k cycles without replies the supervisor has seen k - 1 misses.
    h.motor[1].responsive = false;
    h.cycles(TVC_MAX_MISSED_REPLIES);
    zassert_equal(h.sup.state(), State::ENABLED, "faulted before N missed replies");
    h.cycle();
    zassert_equal(h.sup.state(), State::FAULT, "not faulted after N missed replies");
    zassert_true(h.sup.axis(TVC_AXIS_YAW).faults & FAULT_NO_REPLY);
    zassert_equal(h.sup.axis(TVC_AXIS_PITCH).faults, 0u);

    // Unresponsive yaw gets STOP; healthy pitch slews back to center.
    zassert_equal(h.out[1].action, Action::STOP);
    zassert_equal(h.out[0].action, Action::POSITION);
    h.cycles(20);
    zassert_within(h.sup.axis(TVC_AXIS_PITCH).target_angle_deg, 0.0f, 1e-5f);

    // Latched: recovering the bus alone does not re-enable.
    h.motor[1].responsive = true;
    h.cycles(20);
    zassert_equal(h.sup.state(), State::FAULT);
}

ZTEST(TvcControl_tests, test_moteus_fault_latches_code_and_stops_axis)
{
    Harness h;
    h.enable();
    h.motor[0].mode = static_cast<int8_t>(moteus::Mode::FAULT);
    h.motor[0].fault = 33;  // Motor driver fault.
    h.cycles(2);
    zassert_equal(h.sup.state(), State::FAULT);
    zassert_true(h.sup.axis(TVC_AXIS_PITCH).faults & FAULT_MOTEUS_FAULT);
    zassert_equal(h.sup.axis(TVC_AXIS_PITCH).latched_fault_code, 33);
    zassert_equal(h.out[0].action, Action::STOP);
    zassert_equal(h.out[1].action, Action::POSITION);  // Yaw holds center.

    // STOP clears the live code inside moteus; the latched one survives.
    h.cycles(5);
    zassert_equal(h.sup.axis(TVC_AXIS_PITCH).fault_code, 0);
    zassert_equal(h.sup.axis(TVC_AXIS_PITCH).latched_fault_code, 33);
}

ZTEST(TvcControl_tests, test_moteus_timeout_faults)
{
    Harness h;
    h.enable();
    h.motor[1].mode = static_cast<int8_t>(moteus::Mode::TIMEOUT);
    h.cycles(2);
    zassert_equal(h.sup.state(), State::FAULT);
    zassert_true(h.sup.axis(TVC_AXIS_YAW).faults & FAULT_MOTEUS_TIMEOUT);
    zassert_equal(h.out[1].action, Action::STOP);
}

ZTEST(TvcControl_tests, test_position_error_faults_after_persistence)
{
    Harness h;
    h.enable();
    h.motor[0].stuck = true;
    h.in.pitch_command_deg = 10.0f;
    // Error grows 0.6 deg (~0.068 in) per cycle; crosses 0.10 in on cycle 2, then needs TVC_POSITION_ERROR_CYCLES.
    h.cycles(3);
    zassert_equal(h.sup.state(), State::ENABLED);
    h.cycles(TVC_POSITION_ERROR_CYCLES + 1);
    zassert_equal(h.sup.state(), State::FAULT);
    zassert_true(h.sup.axis(TVC_AXIS_PITCH).faults & FAULT_POSITION_ERROR);
}

ZTEST(TvcControl_tests, test_unexpected_mode_faults_after_grace)
{
    Harness h;
    h.enable();
    h.motor[0].mode = 12;  // "zero velocity": someone else took over the controller.
    h.motor[0].stuck = true;
    h.cycles(5);
    zassert_equal(h.sup.state(), State::FAULT);
    zassert_true(h.sup.axis(TVC_AXIS_PITCH).faults & FAULT_UNEXPECTED_MODE);
}

ZTEST(TvcControl_tests, test_rehoming_clears_fault_and_reenables)
{
    Harness h;
    h.enable();
    h.motor[0].mode = static_cast<int8_t>(moteus::Mode::TIMEOUT);
    h.cycles(3);
    zassert_equal(h.sup.state(), State::FAULT);

    h.in.home_request = true;
    h.cycles(6);
    zassert_equal(h.sup.state(), State::ENABLED);
    zassert_equal(h.sup.axis(TVC_AXIS_PITCH).faults, 0u);
}

ZTEST(TvcControl_tests, test_clear_faults_request_reenables_if_still_homed)
{
    Harness h;
    h.enable();
    h.motor[1].mode = static_cast<int8_t>(moteus::Mode::TIMEOUT);
    h.cycles(3);
    zassert_equal(h.sup.state(), State::FAULT);
    h.in.clear_faults_request = true;
    h.cycles(5);
    zassert_equal(h.sup.state(), State::ENABLED);
}

ZTEST(TvcControl_tests, test_homing_timeout)
{
    Harness h;
    h.cycles(5);
    h.motor[0].responsive = false;
    h.in.home_request = true;
    h.cycles(40);
    zassert_true(h.sup.homing_failed());
    zassert_equal(h.sup.state(), State::READY);
}

ZTEST(TvcControl_tests, test_bench_sweep_shape)
{
    float p, y;
    const float a = 2.0f, r = 0.5f;  // Period per axis = 16 s.
    bench_sweep(0.0f, a, r, p, y);
    zassert_within(p, 0.0f, 1e-5f);
    bench_sweep(4.0f, a, r, p, y);
    zassert_within(p, 2.0f, 1e-5f);
    zassert_within(y, 0.0f, 1e-5f);
    bench_sweep(12.0f, a, r, p, y);
    zassert_within(p, -2.0f, 1e-5f);
    bench_sweep(20.0f, a, r, p, y);
    zassert_within(p, 0.0f, 1e-5f);
    zassert_within(y, 2.0f, 1e-5f);
    // Never exceeds the amplitude, never moves faster than the rate.
    float prev_p = 0.0f, prev_y = 0.0f;
    for (float t = 0.0f; t < 64.0f; t += 0.02f) {
        bench_sweep(t, a, r, p, y);
        zassert_true(std::fabs(p) <= a + 1e-5f && std::fabs(y) <= a + 1e-5f);
        zassert_true(std::fabs(p - prev_p) <= r * 0.02f + 1e-4f && std::fabs(y - prev_y) <= r * 0.02f + 1e-4f);
        prev_p = p;
        prev_y = y;
    }
}

ZTEST_SUITE(TvcControl_tests, NULL, NULL, NULL, NULL, NULL);
