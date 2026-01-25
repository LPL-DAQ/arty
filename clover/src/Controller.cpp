//
// Created by lpl on 1/24/26.
//

#include "Controller.h"

#include "Trace.h"
#include <zephyr/kernel.h>

volatile int step_count = 0;
volatile int count_to = 0;
uint64_t start_clock = 0;

/// Performs one iteration of the control loop. This must execute very quickly, so any physical actions or
/// interactions with peripherals should be asynchronous.
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
        k_msgq_purge(&control_data_msgq);  // Signals client connection that no more
        return;
    }

    int next_millis = step_count + 1;

    // assign sinusoidal with tis offset or linear
    if (combo_mode && gap_millis > 0 && !sine_offsets_fuel.empty() && !sine_offsets_lox.empty()) {
        int seg_idx = next_millis / gap_millis;
        int max_seg_idx = static_cast<int>(sine_offsets_fuel.size()) - 1;
        if (seg_idx < 0) {
            seg_idx = 0;
        }
        if (seg_idx > max_seg_idx) {
            seg_idx = max_seg_idx;
        }
        if (sine_offsets_fuel[seg_idx] != 0.0f) {
            sine_mode = true;
            sine_seq_offset_fuel = sine_offsets_fuel[seg_idx];
            sine_seq_amplitude_fuel = sine_amplitudes_fuel[seg_idx];
            sine_seq_period_fuel = sine_periods_fuel[seg_idx];
            sine_seq_phase_fuel = sine_phases_fuel[seg_idx];

            sine_seq_offset_lox = sine_offsets_lox[seg_idx];
            sine_seq_amplitude_lox = sine_amplitudes_lox[seg_idx];
            sine_seq_period_lox = sine_periods_lox[seg_idx];
            sine_seq_phase_lox = sine_phases_lox[seg_idx];
        }
        else {
            sine_mode = false;
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
    //  sine mode
    else {
        fuel_trace_target =
            std::sin(
                static_cast<float>(next_millis) / sine_seq_period_fuel * std::numbers::pi_v<float> * 2.0f +
                sine_seq_phase_fuel) *
                sine_seq_amplitude_fuel +
            sine_seq_offset_fuel;

        lox_trace_target =
            std::sin(
                static_cast<float>(next_millis) / sine_seq_period_lox * std::numbers::pi_v<float> * 2.0f +
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

K_WORK_DEFINE(control_loop, step_control_loop);

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

K_TIMER_DEFINE(control_loop_schedule_timer, control_loop_schedule, nullptr);

std::expected<void, Error> Controller::handle_load_motor_sequence(const LoadMotorSequenceRequest& req)
{

}

std::expected<void, Error> Controller::handle_start_sequence(const StartSequenceRequest& req)
{

}

std::expected<void, Error> Controller::handle_halt_sequence(const HaltSequenceRequest& req)
{

}
