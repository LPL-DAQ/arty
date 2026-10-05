#include "RangerTvc.h"
#include "MutexGuard.h"
#include "config.h"
#include "tvc/moteus.h"
#include "tvc/moteus_can.h"
#include "tvc/tvc_config.h"
#include "tvc/tvc_control.h"

#include <algorithm>
#include <cmath>
#include <zephyr/kernel.h>
#include <zephyr/logging/log.h>

LOG_MODULE_REGISTER(RangerTvc, CONFIG_LOG_DEFAULT_LEVEL);

// Threading and timing
// --------------------
// One TVC thread owns the CAN bus and the tvc::Supervisor; nothing else touches either. It runs off a periodic k_timer
// (k_timer_status_sync), so the 20 ms period is kept by the kernel timer rather than by sleeping after the work, and
// cannot drift; missed expirations are counted as overruns rather than silently stretching the period.
//
// Each cycle: copy the latest command -> Supervisor::step() on the replies from the previous cycle -> send one frame per
// axis (command + full query) -> wait up to TVC_REPLY_TIMEOUT_US for both replies -> publish telemetry. Fault decisions
// therefore see measurements one cycle (20 ms) old; moteus closes the actual position loop internally at 30+ kHz.
//
// The controller workqueue (1.5 ms tick) and the TVC thread only share `shared` below, behind a k_mutex. Both threads
// are cooperative (negative priority), hold the mutex for a few struct copies and never block while holding it, so there
// is no priority inversion and the controller never waits on CAN: RangerTvc::tick() is a pure hand-off. TVC sits below
// the controller workqueue (-10) so a controller tick always runs first when both are ready, and above the sensor
// threads (-5) so a slow sensor read cannot delay a TVC cycle. Its CPU time per cycle is tens of microseconds; the rest
// of the cycle it is blocked on the timer or the reply queues.

namespace {

constexpr size_t TVC_THREAD_STACK_SIZE = 3072;
/// Window over which loop_jitter_max_us is accumulated (1 s).
constexpr uint32_t JITTER_WINDOW_CYCLES = 1'000'000 / tvc::TVC_LOOP_PERIOD_US;
/// Periodic timing log (10 s). Logging is immediate-mode, so keep it rare.
constexpr uint32_t STATS_LOG_CYCLES = 10 * JITTER_WINDOW_CYCLES;
/// Bench sweep progress log (5 Hz).
[[maybe_unused]] constexpr uint32_t BENCH_LOG_CYCLES = JITTER_WINDOW_CYCLES / 5;

K_THREAD_STACK_DEFINE(tvc_thread_stack, TVC_THREAD_STACK_SIZE);
k_thread tvc_thread;
K_TIMER_DEFINE(tvc_period_timer, nullptr, nullptr);

/// State shared between the TVC thread and everyone else. Guarded by shared_lock.
struct Shared {
    // Inputs to the TVC thread.
    float pitch_command_deg = 0.0f;
    float yaw_command_deg = 0.0f;
    bool home_request = false;
    // Outputs from the TVC thread.
    TvcActuatorCommand pitch_command = TvcActuatorCommand_init_default;
    TvcActuatorCommand yaw_command = TvcActuatorCommand_init_default;
    RangerTvcMetrics metrics = RangerTvcMetrics_init_default;
    RangerTvc::HomingStatus homing = RangerTvc::HomingStatus::SUCCEEDED;
};
K_MUTEX_DEFINE(shared_lock);
Shared shared;

// TVC-thread-only state.
tvc::Supervisor supervisor;

TvcState to_proto(tvc::State s)
{
    return static_cast<TvcState>(static_cast<int>(s));
}

void fill_status(TvcActuatorStatus& out, const tvc::AxisStatus& st, float latency_us)
{
    out.responding = st.replied;
    out.mode = st.mode;
    out.home_state = st.home_state;
    out.fault_code = st.fault_code;
    out.latched_fault_code = st.latched_fault_code;
    out.faults = st.faults;
    out.missed_replies = st.missed_replies;
    out.measured_position_rev = st.measured_rev;
    out.measured_velocity_rev_s = st.velocity_rev_s;
    out.measured_torque_nm = st.torque_nm;
    out.measured_length_in = st.measured_length_in;
    out.measured_angle_deg = st.measured_angle_deg;
    out.position_error_in = st.position_error_in;
    out.bus_voltage_v = st.voltage_v;
    out.temperature_c = st.temperature_c;
    out.reply_latency_us = latency_us;
}

void fill_command(TvcActuatorCommand& out, const tvc::AxisStatus& st, const tvc::AxisOutput& sent)
{
    out.driving = sent.action == tvc::Action::POSITION;
    out.target_angle_deg = st.target_angle_deg;
    out.target_length_in = st.target_length_in;
    out.target_position_rev = st.target_rev;
}

size_t build_frame(const tvc::AxisOutput& out, uint8_t* buf)
{
    switch (out.action) {
    case tvc::Action::QUERY:
        return moteus::make_query_frame(buf);
    case tvc::Action::STOP:
        return moteus::make_stop_frame(buf);
    case tvc::Action::POSITION:
        return moteus::make_position_frame(out.position, buf);
    case tvc::Action::SET_OUTPUT_EXACT:
        return moteus::make_set_output_exact_frame(0.0f, buf);
    }
    return 0;
}

void log_transitions(tvc::State prev_state, uint32_t prev_blockers)
{
    const tvc::State state = supervisor.state();
    if (state != prev_state) {
        LOG_INF("TVC state %d -> %d", static_cast<int>(prev_state), static_cast<int>(state));
        if (state == tvc::State::FAULT) {
            const auto& p = supervisor.axis(tvc::TVC_AXIS_PITCH);
            const auto& y = supervisor.axis(tvc::TVC_AXIS_YAW);
            LOG_ERR("TVC FAULT latched: pitch faults 0x%x (moteus code %d), yaw faults 0x%x (moteus code %d)",
                p.faults,
                p.latched_fault_code,
                y.faults,
                y.latched_fault_code);
        }
        if (prev_state == tvc::State::HOMING) {
            if (supervisor.homing_failed()) {
                LOG_ERR("TVC homing failed: moteus did not report OUTPUT home state at 0 rev");
            }
            else {
                LOG_INF("TVC homed");
            }
        }
    }
    if (state == tvc::State::READY && supervisor.enable_blockers() != prev_blockers) {
        LOG_INF("TVC not enabling, blockers 0x%x", supervisor.enable_blockers());
    }
}

void tvc_loop(void*, void*, void*)
{
    tvc::Inputs inputs;
    tvc::AxisOutput outputs[tvc::TVC_AXIS_COUNT];
    float latency_us[tvc::TVC_AXIS_COUNT] = {};

    uint32_t loop_count = 0;
    uint32_t overruns = 0;
    uint32_t prev_wake = k_cycle_get_32();
    float period_us = 0.0f;
    float jitter_window_max_us = 0.0f;
    float jitter_reported_us = 0.0f;
    float stats_max_latency_us = 0.0f;
    float stats_max_cycle_us = 0.0f;
    [[maybe_unused]] uint32_t bench_start_cycle = 0;

    k_timer_start(&tvc_period_timer, K_USEC(tvc::TVC_LOOP_PERIOD_US), K_USEC(tvc::TVC_LOOP_PERIOD_US));

    while (true) {
        // Absolute-period wait. Returns the number of expirations since the last call; >1 means we missed a cycle.
        const uint32_t expirations = k_timer_status_sync(&tvc_period_timer);
        const uint32_t wake = k_cycle_get_32();
        if (expirations > 1) {
            overruns += expirations - 1;
        }
        period_us = k_cyc_to_us_near32(wake - prev_wake);
        prev_wake = wake;
        if (loop_count > 0) {
            jitter_window_max_us = std::max(jitter_window_max_us, std::fabs(period_us - tvc::TVC_LOOP_PERIOD_US));
        }

        // 1. Latest command and requests from the controller.
        {
            MutexGuard guard{&shared_lock};
            inputs.pitch_command_deg = shared.pitch_command_deg;
            inputs.yaw_command_deg = shared.yaw_command_deg;
            inputs.home_request = shared.home_request;
            shared.home_request = false;
        }

#ifdef CONFIG_RANGER_TVC_BENCH_SWEEP
        // Bench mode ignores GNC entirely and sweeps each axis slowly through +/-amplitude while enabled.
        if (supervisor.state() == tvc::State::ENABLED) {
            const float t_s = (loop_count - bench_start_cycle) * (tvc::TVC_LOOP_PERIOD_US * 1e-6f);
            tvc::bench_sweep(t_s, tvc::TVC_BENCH_SWEEP_AMPLITUDE_DEG, tvc::TVC_BENCH_SWEEP_RATE_DEG_S, inputs.pitch_command_deg, inputs.yaw_command_deg);
        }
        else {
            bench_start_cycle = loop_count + 1;
            inputs.pitch_command_deg = 0.0f;
            inputs.yaw_command_deg = 0.0f;
        }
#endif

        // 2. Decide this cycle's frames from the previous cycle's replies.
        const tvc::State prev_state = supervisor.state();
        const uint32_t prev_blockers = supervisor.enable_blockers();
        supervisor.step(inputs, outputs);
        log_transitions(prev_state, prev_blockers);

        // 3. Send one frame per axis; every frame requests a reply.
        uint32_t sent_at[tvc::TVC_AXIS_COUNT] = {};
        bool sent[tvc::TVC_AXIS_COUNT] = {};
        for (int i = 0; i < tvc::TVC_AXIS_COUNT; i++) {
            uint8_t frame[moteus::MAX_FRAME_BYTES];
            const size_t len = build_frame(outputs[i], frame);
            moteus_can::flush(i);
            sent_at[i] = k_cycle_get_32();
            sent[i] = len > 0 && moteus_can::send(i, frame, len, true);
        }

        // 4. Collect both replies against one shared deadline.
        const k_timepoint_t deadline = sys_timepoint_calc(K_USEC(tvc::TVC_REPLY_TIMEOUT_US));
        for (int i = 0; i < tvc::TVC_AXIS_COUNT; i++) {
            tvc::AxisFeedback& fb = inputs.feedback[i];
            fb = tvc::AxisFeedback{};
            moteus_can::Reply reply;
            if (sent[i] && moteus_can::receive(i, reply, deadline)) {
                latency_us[i] = k_cyc_to_us_near32(k_cycle_get_32() - sent_at[i]);
                fb.replied = true;
                fb.parsed = moteus::parse_reply(reply.data, reply.len, fb.query);
                stats_max_latency_us = std::max(stats_max_latency_us, latency_us[i]);
            }
            else {
                latency_us[i] = NAN;
            }
        }
        const float cycle_us = k_cyc_to_us_near32(k_cycle_get_32() - wake);
        stats_max_cycle_us = std::max(stats_max_cycle_us, cycle_us);

        if (++loop_count % JITTER_WINDOW_CYCLES == 0) {
            jitter_reported_us = jitter_window_max_us;
            jitter_window_max_us = 0.0f;
        }

        // 5. Publish. Axis status reflects the replies just collected only after the next step(); publishing the
        //    supervisor's view keeps telemetry consistent with what fault detection saw.
        {
            MutexGuard guard{&shared_lock};
            fill_command(shared.pitch_command, supervisor.axis(tvc::TVC_AXIS_PITCH), outputs[tvc::TVC_AXIS_PITCH]);
            fill_command(shared.yaw_command, supervisor.axis(tvc::TVC_AXIS_YAW), outputs[tvc::TVC_AXIS_YAW]);

            RangerTvcMetrics& m = shared.metrics;
            m.state = to_proto(supervisor.state());
            m.enable_blockers = supervisor.enable_blockers();
            m.homing_failed = supervisor.homing_failed();
            m.requested_pitch_deg = inputs.pitch_command_deg;
            m.requested_yaw_deg = inputs.yaw_command_deg;
            fill_status(m.pitch, supervisor.axis(tvc::TVC_AXIS_PITCH), latency_us[tvc::TVC_AXIS_PITCH]);
            fill_status(m.yaw, supervisor.axis(tvc::TVC_AXIS_YAW), latency_us[tvc::TVC_AXIS_YAW]);
            m.loop_count = loop_count;
            m.loop_period_us = period_us;
            m.loop_jitter_max_us = jitter_reported_us;
            m.cycle_time_us = cycle_us;
            m.loop_overruns = overruns;
#ifdef CONFIG_RANGER_TVC_BENCH_SWEEP
            m.bench_sweep = true;
#endif

            if (supervisor.state() == tvc::State::HOMING) {
                shared.homing = RangerTvc::HomingStatus::IN_PROGRESS;
            }
            else if (prev_state == tvc::State::HOMING) {
                shared.homing = supervisor.homing_failed() ? RangerTvc::HomingStatus::FAILED : RangerTvc::HomingStatus::SUCCEEDED;
            }
            else if (inputs.home_request) {
                // Rejected: the supervisor only homes once both controllers have replied and been stopped.
                shared.homing = RangerTvc::HomingStatus::FAILED;
            }
        }
        // Log outside the lock: immediate-mode logging can take milliseconds and the controller tick waits on it.
        if (inputs.home_request && supervisor.state() != tvc::State::HOMING) {
            LOG_ERR("TVC homing rejected: controllers not up yet (state %d)", static_cast<int>(supervisor.state()));
        }

        if (loop_count % STATS_LOG_CYCLES == 0) {
            LOG_INF("TVC timing: max jitter %d us, max reply latency %d us, max cycle %d us, overruns %u, missed p/y %u/%u, tx rejected %u",
                static_cast<int>(jitter_reported_us),
                static_cast<int>(stats_max_latency_us),
                static_cast<int>(stats_max_cycle_us),
                overruns,
                supervisor.axis(tvc::TVC_AXIS_PITCH).missed_replies,
                supervisor.axis(tvc::TVC_AXIS_YAW).missed_replies,
                moteus_can::tx_rejected());
            stats_max_latency_us = 0.0f;
            stats_max_cycle_us = 0.0f;
        }

#ifdef CONFIG_RANGER_TVC_BENCH_SWEEP
        if (supervisor.state() == tvc::State::ENABLED && loop_count % BENCH_LOG_CYCLES == 0) {
            const auto& p = supervisor.axis(tvc::TVC_AXIS_PITCH);
            const auto& y = supervisor.axis(tvc::TVC_AXIS_YAW);
            LOG_INF("TVC sweep mdeg: pitch cmd %d meas %d, yaw cmd %d meas %d",
                static_cast<int>(p.target_angle_deg * 1000.0f),
                static_cast<int>(p.measured_angle_deg * 1000.0f),
                static_cast<int>(y.target_angle_deg * 1000.0f),
                static_cast<int>(y.measured_angle_deg * 1000.0f));
        }
#endif
    }
}

}  // namespace

std::expected<void, Error> RangerTvc::init()
{
#ifdef CONFIG_RANGER_TVC_BENCH_SWEEP
    LOG_WRN("CONFIG_RANGER_TVC_BENCH_SWEEP is set: TVC ignores GNC commands and sweeps +/-%d deg. BENCH USE ONLY.",
        static_cast<int>(tvc::TVC_BENCH_SWEEP_AMPLITUDE_DEG));
#endif
    if (!tvc::config_is_complete()) {
        LOG_WRN("TVC config has TODO(adit) placeholders (turns_per_inch / direction_sign): actuators will not enable");
    }

    const uint8_t ids[moteus_can::MAX_DEVICES] = {tvc::TVC_PITCH.moteus_id, tvc::TVC_YAW.moteus_id};
    if (auto result = moteus_can::init(ids, tvc::TVC_CAN_PREFIX, tvc::TVC_CAN_SOURCE_ID, tvc::TVC_CAN_BRS); !result) {
        return std::unexpected(result.error().context("failed to initialize TVC CAN bus"));
    }

    k_thread_create(
        &tvc_thread, tvc_thread_stack, K_THREAD_STACK_SIZEOF(tvc_thread_stack), tvc_loop, nullptr, nullptr, nullptr, TVC_THREAD_PRIORITY, 0, K_NO_WAIT);
    k_thread_name_set(&tvc_thread, "tvc");
    return {};
}

void RangerTvc::reset()
{
}

std::expected<RangerTvc::Telemetry, Error> RangerTvc::tick(float pitch_command_deg, float yaw_command_deg)
{
    Telemetry t;
    {
        MutexGuard guard{&shared_lock};
        shared.pitch_command_deg = pitch_command_deg;
        shared.yaw_command_deg = yaw_command_deg;
        t = {shared.pitch_command, shared.yaw_command, shared.metrics};
    }

    if (auto ready = ensure_ready(); !ready) {
        return std::unexpected(ready.error());
    }
    return t;
}

void RangerTvc::hold_center()
{
    MutexGuard guard{&shared_lock};
    shared.pitch_command_deg = 0.0f;
    shared.yaw_command_deg = 0.0f;
}

RangerTvc::Telemetry RangerTvc::telemetry()
{
    MutexGuard guard{&shared_lock};
    return {shared.pitch_command, shared.yaw_command, shared.metrics};
}

std::expected<void, Error> RangerTvc::ensure_ready()
{
    RangerTvcMetrics m;
    {
        MutexGuard guard{&shared_lock};
        m = shared.metrics;
    }

    if (m.state == TvcState_TVC_STATE_ENABLED) {
        return {};
    }
    if (m.state == TvcState_TVC_STATE_FAULT) {
        if (!tvc::TVC_FAULT_ABORTS_CONTROLLER) {
            return {};  // Holding center; let the sequence continue.
        }
        return std::unexpected(Error::from_cause("TVC fault latched (pitch faults 0x%x, yaw faults 0x%x)", m.pitch.faults, m.yaw.faults));
    }
    return std::unexpected(Error::from_cause("TVC not enabled (state %d, enable blockers 0x%x)", static_cast<int>(m.state), m.enable_blockers));
}

void RangerTvc::request_home()
{
    MutexGuard guard{&shared_lock};
    // The TVC enables right after homing; make it hold center rather than resume a stale trace command.
    shared.pitch_command_deg = 0.0f;
    shared.yaw_command_deg = 0.0f;
    shared.home_request = true;
    shared.homing = HomingStatus::IN_PROGRESS;
}

RangerTvc::HomingStatus RangerTvc::homing_status()
{
    MutexGuard guard{&shared_lock};
    return shared.homing;
}
