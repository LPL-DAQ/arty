#include "sequencer.h"
#include "throttle_valve.h"
#include "pts.h"

#include <vector>
#include <zephyr/logging/log.h>
#include <zephyr/kernel.h>
#include <zephyr/sys/util.h>
#include <zephyr/sys/cbprintf.h>
#include <array>
#include <zephyr/net/socket.h>
#include <algorithm>
#include <string>
#include <cstdint>
#include <numbers>
#include <cmath>
#include "server.h"

LOG_MODULE_REGISTER(sequencer, CONFIG_LOG_DEFAULT_LEVEL);

constexpr uint64_t
    NSEC_PER_CONTROL_TICK = 1'000'000; // 1 ms

static constexpr int MAX_BREAKPOINTS = 67;

K_MUTEX_DEFINE(sequence_lock);

static int gap_millis;
static std::vector<float> fuel_breakpoints;
static std::vector<float> lox_breakpoints;

static std::vector<float> sine_offsets_fuel;
static std::vector<float> sine_amplitudes_fuel;
static std::vector<float> sine_periods_fuel;
static std::vector<float> sine_phases_fuel;
static std::vector<float> sine_offsets_lox;
static std::vector<float> sine_amplitudes_lox;
static std::vector<float> sine_periods_lox;
static std::vector<float> sine_phases_lox;


static int data_sock = -1;
static bool motor_only = false;
static float sine_seq_offset_fuel = 0.0f;
static float sine_seq_amplitude_fuel = 0.0f;
static float sine_seq_period_fuel = 0.0f;
static float sine_seq_phase_fuel = 0.0f;
static float sine_seq_offset_lox = 0.0f;
static float sine_seq_amplitude_lox = 0.0f;
static float sine_seq_period_lox = 0.0f;
static float sine_seq_phase_lox = 0.0f;
static bool sine_mode = false;
static bool combo_mode = false;

volatile int step_count = 0;
volatile int count_to = 0;
uint64_t start_clock = 0;

// Data that ought be logged for each control loop iteration.
struct control_iter_data {
    float time;
    uint32_t data_queue_size;
    float fuel_valve_setpoint;
    float fuel_valve_internal_pos;
    float fuel_valve_encoder_pos;
    float fuel_valve_velocity;
    float fuel_valve_acceleration;
    uint64_t fuel_valve_nsec_per_pulse;
    float lox_valve_setpoint;
    float lox_valve_internal_pos;
    float lox_valve_encoder_pos;
    float lox_valve_velocity;
    float lox_valve_acceleration;
    uint64_t lox_valve_nsec_per_pulse;
    float pt102;
    float pt103;
    float pto401;
    float pt202;
    float pt203;
    float ptf401;
    float ptc401;
    float ptc402;
};

static constexpr int MAX_CONTROL_DATA_QUEUE_SIZE = 250;
K_MSGQ_DEFINE(control_data_msgq,
              sizeof(control_iter_data), MAX_CONTROL_DATA_QUEUE_SIZE, 1);

// Performs one iteration of the control loop. This must execute very quickly, so any physical actions or
// interactions with peripherals should be asynchronous.
static void step_control_loop(k_work*)
{

    // Last iter of control loop, execute cleanup tasks. step_count is [1, count_to] for normal iterations,
    // and step_count == count_to+1 for the last cleanup iteration.
    if (step_count > count_to) {
        FuelValve::stop();
        LoxValve::stop();
        // HACK: relinquish control for a little bit to allow client connection to flush data.
        // There really ought to be a cleaner way to do this.
        k_sleep(K_MSEC(100));
        k_msgq_purge(&control_data_msgq); // Signals client connection that no more
        return;
    }

    int next_millis = step_count + 1;

    // assign sinusoidal with tis offset or linear
    if (combo_mode && gap_millis > 0 && !sine_offsets_fuel.empty() && !sine_offsets_lox.empty()) {
        int seg_idx = next_millis / gap_millis;
        int max_seg_idx = static_cast<int>(sine_offsets_fuel.size()) - 1;
        if (seg_idx < 0) seg_idx = 0;
        if (seg_idx > max_seg_idx) seg_idx = max_seg_idx;
        if (sine_offsets_fuel[seg_idx] != 0.0f) {
            sine_mode=true;
            sine_seq_offset_fuel=sine_offsets_fuel[seg_idx];
            sine_seq_amplitude_fuel=sine_amplitudes_fuel[seg_idx];
            sine_seq_period_fuel=sine_periods_fuel[seg_idx];
            sine_seq_phase_fuel=sine_phases_fuel[seg_idx];

            sine_seq_offset_lox=sine_offsets_lox[seg_idx];
            sine_seq_amplitude_lox=sine_amplitudes_lox[seg_idx];
            sine_seq_period_lox=sine_periods_lox[seg_idx];
            sine_seq_phase_lox=sine_phases_lox[seg_idx];

        } else {
            sine_mode=false;
        }
    }

    float fuel_trace_target;
    float lox_trace_target;
    if (!sine_mode) {
        // Interpolate breakpoints to find instantaneous trace target.
        int low_bp_index = MIN(next_millis / gap_millis, std::ssize(fuel_breakpoints) - 1);
        int high_bp_index = low_bp_index + 1;
        if (low_bp_index < 0) {
            LOG_WRN("Low index is less than 0, how is that possible? curr: %d, gap: %d", next_millis, gap_millis);
            low_bp_index = 0;
        }
        if (high_bp_index >= std::ssize(fuel_breakpoints)) {
            fuel_trace_target = fuel_breakpoints.back();
            lox_trace_target = lox_breakpoints.back();
        }
        else {
            float tween = static_cast<float>(next_millis - (low_bp_index * gap_millis)) / gap_millis;
            fuel_trace_target = fuel_breakpoints[low_bp_index] +
                                (fuel_breakpoints[high_bp_index] - fuel_breakpoints[low_bp_index]) * tween;
            lox_trace_target = lox_breakpoints[low_bp_index] +
                               (lox_breakpoints[high_bp_index] - lox_breakpoints[low_bp_index]) * tween;
        }
    }
    else { //  sine mode
        fuel_trace_target =
            std::sin(static_cast<float>(next_millis) / sine_seq_period_fuel * std::numbers::pi_v<float> * 2.0f +
                     sine_seq_phase_fuel) *
            sine_seq_amplitude_fuel +
            sine_seq_offset_fuel;

        lox_trace_target = std::sin(static_cast<float>(next_millis) / sine_seq_period_lox * std::numbers::pi_v<float> * 2.0f +
                     sine_seq_phase_lox) *
            sine_seq_amplitude_lox +
            sine_seq_offset_lox;
    }

    // Timestamp
    uint64_t since_start = k_cycle_get_64() - start_clock;
    uint64_t ns_since_start = k_cyc_to_ns_floor64(since_start);

    // PTs
    pt_readings readings = pts_sample();

    float fuel_valve_target;
    float lox_valve_target;
    if (motor_only) {
        fuel_valve_target = fuel_trace_target;
        lox_valve_target = lox_trace_target;
    }
    else {
        // Insert closed-loop logic here.
        fuel_valve_target = 45.0f;
        lox_valve_target = 45.0f;
    }

    // move to target
    FuelValve::tick(fuel_valve_target);
    LoxValve::tick(lox_valve_target);

    // Log current data
    control_iter_data iter_data = {
        .time = static_cast<float>(ns_since_start) / 1e9f,
        .data_queue_size = k_msgq_num_used_get(&control_data_msgq),

        .fuel_valve_setpoint = fuel_valve_target,
        .fuel_valve_internal_pos = FuelValve::get_pos_internal(),
        .fuel_valve_encoder_pos = FuelValve::get_pos_encoder(),
        .fuel_valve_velocity = FuelValve::get_velocity(),
        .fuel_valve_acceleration = FuelValve::get_acceleration(),
        .fuel_valve_nsec_per_pulse = FuelValve::get_nsec_per_pulse(),

        .lox_valve_setpoint = lox_valve_target,
        .lox_valve_internal_pos = LoxValve::get_pos_internal(),
        .lox_valve_encoder_pos = LoxValve::get_pos_encoder(),
        .lox_valve_velocity = LoxValve::get_velocity(),
        .lox_valve_acceleration = LoxValve::get_acceleration(),
        .lox_valve_nsec_per_pulse = LoxValve::get_nsec_per_pulse(),

        .pt102 = readings.pt102,
        .pt103 = readings.pt103,
        .pto401 = readings.pto401,
        .pt202 = readings.pt202,
        .pt203 = readings.pt203,
        .ptf401 = readings.ptf401,
        .ptc401 = readings.ptc401,
        .ptc402 = readings.ptc402,
    };
    int err = k_msgq_put(&control_data_msgq, &iter_data, K_NO_WAIT);
    if (err) {
        // Adding to msgq can only fail with -ENOMSG.
        LOG_ERR("Control data queue is full! Data is being lost!!!");
    }
}

K_WORK_DEFINE(control_loop, step_control_loop
);

// ISR that schedules a control iteration in the work queue.
static void control_loop_schedule(k_timer* timer)
{
    // step_count is [0, count_to) during a normal iteration. step_count == count_to
    // schedules the final iteration.
    if (step_count > count_to) {
        k_msgq_purge(&control_data_msgq);
        k_timer_stop(timer);
        return;
    }
    k_work_submit(&control_loop);
    step_count += 1;
}

K_TIMER_DEFINE(control_loop_schedule_timer, control_loop_schedule, nullptr
);

int sequencer_start_trace()
{
    if (fuel_breakpoints.size() < 2) {
        LOG_ERR("No fuel breakpoints specified.");
        return 1;
    }
    if (lox_breakpoints.size() < 2) {
        LOG_ERR("No lox breakpoints specified.");
        return 1;
    }
    if (fuel_breakpoints.size() > MAX_BREAKPOINTS) {
        LOG_ERR("Too many breakpoints: %u", fuel_breakpoints.size());
        return 1;
    }
    if (gap_millis < 1) {
        LOG_ERR("Breakpoint gap_millis is too short: %d ms", gap_millis);
        return 1;
    }
    if (fuel_breakpoints.size() != lox_breakpoints.size()) {
        LOG_ERR("Both fuel and lox breakpoints should be specified");
        return 1;
    }

    k_mutex_lock(&sequence_lock, K_FOREVER);
    if (data_sock == -1) {
        LOG_WRN("Data socket is not set, all data will be lost.");
    }

    // Set first breakpoint to whatever the current is to prevent sudden jolts.
    if (motor_only) {
        LOG_INF("Running open-loop motor trace.");
        fuel_breakpoints.front() = FuelValve::get_pos_internal();
        lox_breakpoints.front() = LoxValve::get_pos_internal();
    }
    else {
        LOG_INF("Running closed-loop control trace.");
        // Something should be put here for a real closed loop.
        fuel_breakpoints.front() = 0.0f;
        lox_breakpoints.front() = 0.0f;
    }

    LOG_INF("Got breakpoints:");
    for (int i = 0; i < std::ssize(fuel_breakpoints); ++i) {
        LOG_INF("t=%d ms, fuel=%f, lox=%f", i * gap_millis, static_cast<double>(fuel_breakpoints[i]),
                static_cast<double>(lox_breakpoints[i]));
    }

    step_count = 0;
    count_to = (std::ssize(fuel_breakpoints) - 1) * gap_millis;

    start_clock = k_cycle_get_64();

    // Start control iterations
    k_timer_start(&control_loop_schedule_timer, K_NSEC(NSEC_PER_CONTROL_TICK), K_NSEC(NSEC_PER_CONTROL_TICK));

    if (data_sock == -1) {
        LOG_WRN("Exiting early as no data will be collected.");
        return 0;
    }

    // Print header
    send_string_fully(data_sock, ">>>>SEQ START<<<<\n");
    send_string_fully(data_sock,
                      "time,data_queue_size,"
                      "fuel_valve_setpoint,fuel_valve_internal_pos,fuel_valve_encoder_pos,"
                      "fuel_valve_velocity,fuel_valve_acceleration,fuel_valve_nsec_per_pulse,"
                      "lox_valve_setpoint,lox_valve_internal_pos,lox_valve_encoder_pos,"
                      "lox_valve_velocity,lox_valve_acceleration,lox_valve_nsec_per_pulse,"
                      "pt102,pt103,pto401,pt202,pt203,ptf401,ptc401,ptc402\n");
    // Dump data as we get it.
    control_iter_data data = {0};
    while (true) {
        int err = k_msgq_get(&control_data_msgq, &data, K_FOREVER);
        if (err) {
            break;
        }

        constexpr int MAX_DATA_LEN = 512;
        char buf[MAX_DATA_LEN];

        int would_write = snprintfcb(
            buf, MAX_DATA_LEN,
            "%.8f,%d,"
            "%.8f,%.8f,%.8f,%.8f,%.8f,%llu,"
            "%.8f,%.8f,%.8f,%.8f,%.8f,%llu,"
            "%.8f,%.8f,%.8f,%.8f,%.8f,%.8f,%.8f,%.8f\n",
            static_cast<double>(data.time),
            data.data_queue_size,
            static_cast<double>(data.fuel_valve_setpoint),
            static_cast<double>(data.fuel_valve_internal_pos),
            static_cast<double>(data.fuel_valve_encoder_pos),
            static_cast<double>(data.fuel_valve_velocity),
            static_cast<double>(data.fuel_valve_acceleration),
            data.fuel_valve_nsec_per_pulse,
            static_cast<double>(data.lox_valve_setpoint),
            static_cast<double>(data.lox_valve_internal_pos),
            static_cast<double>(data.lox_valve_encoder_pos),
            static_cast<double>(data.lox_valve_velocity),
            static_cast<double>(data.lox_valve_acceleration),
            data.lox_valve_nsec_per_pulse,
            static_cast<double>(data.pt102),
            static_cast<double>(data.pt103),
            static_cast<double>(data.pto401),
            static_cast<double>(data.pt202),
            static_cast<double>(data.pt203),
            static_cast<double>(data.ptf401),
            static_cast<double>(data.ptc401),
            static_cast<double>(data.ptc402)
        );
        int actually_written = std::min(would_write, MAX_DATA_LEN - 1);
        err = send_fully(data_sock, buf, actually_written);
        if (err) {
            LOG_WRN("Failed to send data: err %d", err);
        }
    }

    send_string_fully(data_sock, ">>>>SEQ END<<<<\n");

    data_sock = -1;
    k_mutex_unlock(&sequence_lock);
    return 0;
}

int sequencer_prepare(int gap, std::vector<float> fuel_bps, std::vector<float> lox_bps, bool mot_only)
{
    if (gap <= 0 || fuel_bps.empty() || lox_bps.empty()) {
        return 1;
    }

    gap_millis = gap;
    fuel_breakpoints = fuel_bps;
    lox_breakpoints = lox_bps;
    motor_only = mot_only;
    sine_mode = false;
    combo_mode = false;
    sine_offsets_fuel.clear();
    sine_amplitudes_fuel.clear();
    sine_periods_fuel.clear();
    sine_phases_fuel.clear();

    sine_offsets_lox.clear();
    sine_amplitudes_lox.clear();
    sine_periods_lox.clear();
    sine_phases_lox.clear();

    return 0;
}

int sequencer_prepare_sine(int total_time, float offset, float amplitude, float period, float phase)
{
    if (amplitude <= 0.0f || period <= 0.0f) {
        return 1;
    }
    gap_millis = total_time;
    fuel_breakpoints = std::vector<float>{0.0f, 0.0f};
    lox_breakpoints = std::vector<float>{0.0f, 0.0f};
    sine_seq_offset_fuel = offset;
    sine_seq_amplitude_fuel = amplitude;
    sine_seq_period_fuel = period;
    sine_seq_phase_fuel = phase / 360.0f * 2.0f * std::numbers::pi_v<float>;

    sine_seq_offset_lox = offset;
    sine_seq_amplitude_lox = amplitude;
    sine_seq_period_lox = period;
    sine_seq_phase_lox = phase / 360.0f * 2.0f * std::numbers::pi_v<float>;

    motor_only = true;
    sine_mode = true;
    combo_mode = false;
    sine_offsets_fuel.clear();
    sine_amplitudes_fuel.clear();
    sine_periods_fuel.clear();
    sine_phases_fuel.clear();

    sine_offsets_lox.clear();
    sine_amplitudes_lox.clear();
    sine_periods_lox.clear();
    sine_phases_lox.clear();
    return 0;
}

int sequencer_prepare_combo(
    int gap,
    const std::vector<float>& fuel_bps,
    const std::vector<float>& lox_bps,
    const std::vector<float>& seq_sine_offsets_fuel,
    const std::vector<float>& seq_sine_amps_fuel,
    const std::vector<float>& seq_sine_periods_fuel,
    const std::vector<float>& seq_sine_phases_fuel,
    const std::vector<float>& seq_sine_offsets_lox,
    const std::vector<float>& seq_sine_amps_lox,
    const std::vector<float>& seq_sine_periods_lox,
    const std::vector<float>& seq_sine_phases_lox,
    bool mot_only);{
    if (gap <= 0 || fuel_bps.empty() || lox_bps.empty()) {
        return 1;
    }
    if (fuel_bps.size() != lox_bps.size()) {
        return 1;
    }
    // Expect one sine-segment entry per segment (breakpoints - 1)
    const size_t num_segments = fuel_bps.size() - 1;
    auto check_size = [num_segments](const std::vector<float>& v) {
        return v.size() == num_segments;
    };
    if (!check_size(seq_sine_offsets_fuel)  ||
        !check_size(seq_sine_amps_fuel)     ||
        !check_size(seq_sine_periods_fuel)  ||
        !check_size(seq_sine_phases_fuel)   ||
        !check_size(seq_sine_offsets_lox)   ||
        !check_size(seq_sine_amps_lox)      ||
        !check_size(seq_sine_periods_lox)   ||
        !check_size(seq_sine_phases_lox)) {
        return 1;
        }


    gap_millis = gap;
    fuel_breakpoints = fuel_bps;
    lox_breakpoints = lox_bps;
    sine_offsets_fuel = seq_sine_offsets_fuel;
    sine_amplitudes_fuel = seq_sine_amps_fuel;
    sine_periods_fuel = seq_sine_periods_fuel;
    sine_phases_fuel = seq_sine_phases_fuel;
    sine_offsets_lox = seq_sine_offsets_lox;
    sine_amplitudes_lox = seq_sine_amps_lox;
    sine_periods_lox = seq_sine_periods_lox;
    sine_phases_lox = seq_sine_phases_lox;

    auto deg_to_rad = [](float deg) {
        return deg / 360.0f * 2.0f * std::numbers::pi_v<float>;
        // equivalently: return deg * std::numbers::pi_v<float> / 180.0f;
    };

    for (size_t i = 0; i < sine_phases_fuel.size(); ++i) {
        sine_phases_fuel[i] = deg_to_rad(seq_sine_phases_fuel[i]);
        sine_phases_lox[i]  = deg_to_rad(seq_sine_phases_lox[i]);
    }

    motor_only = mot_only;
    combo_mode = true;
    return 0;
}

void sequencer_set_data_recipient(int sock)
{
    k_mutex_lock(&sequence_lock, K_FOREVER);
    data_sock = sock;
    k_mutex_unlock(&sequence_lock);

    LOG_INF("Registered data recipient with socket %d", sock);
}

void sequencer_halt()
{
    count_to = 0;
}
