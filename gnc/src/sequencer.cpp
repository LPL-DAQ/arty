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
#include "server.h"

LOG_MODULE_REGISTER(sequencer, CONFIG_LOG_DEFAULT_LEVEL);

constexpr uint64_t NSEC_PER_CONTROL_TICK = 1'000'000; // 1 ms

static constexpr int MAX_BREAKPOINTS = 20;

K_MUTEX_DEFINE(sequence_lock);

static int gap_millis;
static std::vector<float> breakpoints;
static int data_sock = -1;

volatile int step_count = 0;
volatile int count_to = 0;
uint64_t start_clock = 0;


/// Data that ought be logged for each control loop iteration.
struct control_iter_data {
    float time;
    uint32_t queue_size;
    float motor_target;
    float motor_pos;
    float motor_velocity;
    float motor_acceleration;
    uint64_t motor_nsec_per_pulse;
    float pt202;
    float pt203;
    float ptf401;
};

/// Control loop iteration will enqueue data for broadcasting over ethernet by another thread.
K_MSGQ_DEFINE(control_data_msgq, sizeof(control_iter_data), 100, 1);

/// Performs one iteration of the control loop. This must execute very quickly, so any physical actions or
/// interactions with peripherals should be asynchronous.
static void step_control_loop(k_work *) {
    // Last iter of control loop, execute cleanup tasks. step_count is [1, count_to] for normal iterations,
    // and step_count == count_to+1 for the last cleanup iteration.
    if (step_count > count_to) {
        throttle_valve_stop();
        // HACK: relinquish control for a little bit to allow client connection to flush data.
        // There really ought to be a cleaner way to do this.
        k_sleep(K_MSEC(100));
        k_msgq_purge(&control_data_msgq); // Signals client connection that no more
        return;
    }

    int next_millis = step_count + 1;

    int low_bp_index = MIN(next_millis / gap_millis, std::ssize(breakpoints) - 1);
    int high_bp_index = low_bp_index + 1;
    if (low_bp_index < 0) {
        LOG_WRN("Low index is less than 0, how is that possible? curr: %d, gap: %d", next_millis, gap_millis);
        low_bp_index = 0;
    }
    float target;
    if (high_bp_index >= std::ssize(breakpoints)) {
        target = breakpoints.back();
    } else {
        float tween = static_cast<float>(next_millis - (low_bp_index * gap_millis)) / gap_millis;
        target = breakpoints[low_bp_index] + (breakpoints[high_bp_index] - breakpoints[low_bp_index]) * tween;
    }


    // move to target
    throttle_valve_move(target);

    // Log current data
    // TODO - this copies :/ can we put the message in place?
    uint64_t since_start = k_cycle_get_64() - start_clock;
    uint64_t ns_since_start = k_cyc_to_ns_floor64(since_start);
    pt_readings readings = pts_sample();
    control_iter_data iter_data = {
            .time = static_cast<float>(ns_since_start) / 1e9f,
            .queue_size = k_msgq_num_used_get(&control_data_msgq),
            .motor_target = target,
            .motor_pos = throttle_valve_get_pos(),
            .motor_velocity = throttle_valve_get_velocity(),
            .motor_acceleration = throttle_valve_get_acceleration(),
            .motor_nsec_per_pulse = throttle_valve_get_nsec_per_pulse(),
            .pt202 = readings.pt202,
            .pt203 = readings.pt203,
            .ptf401 = readings.ptf401
    };
    int err = k_msgq_put(&control_data_msgq, &iter_data, K_NO_WAIT);
    if (err) {
        // Adding to msgq can only fail with -ENOMSG.
        LOG_ERR("Control data queue is full! Data is being lost!!!");
    }
}

K_WORK_DEFINE(control_loop, step_control_loop);

/// ISR that schedules a control iteration in the work queue.
static void control_loop_schedule(k_timer *timer) {
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

int sequencer_start_trace() {
    if (breakpoints.size() < 2) {
        LOG_ERR("No breakpoints specified.");
        return 1;
    }
    if (breakpoints.size() > MAX_BREAKPOINTS) {
        LOG_ERR("Too many breakpoints: %u", breakpoints.size());
        return 1;
    }
    if (gap_millis < 1) {
        LOG_ERR("Breakpoint gap_millis is too short: %d ms", gap_millis);
        return 1;
    }

    k_mutex_lock(&sequence_lock, K_FOREVER);
    if (data_sock == -1) {
        LOG_ERR("Data socket is not set");
        k_mutex_unlock(&sequence_lock);
        return 1;
    }


    // Replace first breakpoint with current position
    breakpoints.front() = throttle_valve_get_pos();
    LOG_INF("Got breakpoints:");
    for (int i = 0; i < std::ssize(breakpoints); ++i) {
        LOG_INF("t=%d ms, bp=%f", i * gap_millis, static_cast<double>(breakpoints[i]));
    }

    step_count = 0;
    count_to = (std::ssize(breakpoints) - 1) * gap_millis;

    start_clock = k_cycle_get_64();

    // Start control iterations
    k_timer_start(&control_loop_schedule_timer, K_NSEC(NSEC_PER_CONTROL_TICK), K_NSEC(NSEC_PER_CONTROL_TICK));

    // Print header
    send_string_fully(data_sock, ">>>>SEQ START<<<<\n");
    send_string_fully(data_sock,
                      "time,queue_size,motor_target,motor_pos,motor_velocity,motor_acceleration,motor_nsec_per_pulse,pt202,pt203,ptf401\n");

    // Dump data as we get it. Connection client is preemptible while control sequence is in system workqueue
    // (cooperative) so sending data should never block processing of control iter.
    control_iter_data data = {0};
    while (true) {
        int err = k_msgq_get(&control_data_msgq, &data, K_FOREVER);
        // -ENOMSG is sent when queue is purged to signal end of control seq. No other possible
        // error as we are waiting forever.
        if (err) {
            break;
        }

        constexpr int MAX_DATA_LEN = 512;
        char buf[MAX_DATA_LEN];

        int would_write = snprintfcb(buf, MAX_DATA_LEN, "%.8f,%d,%.8f,%.8f,%.8f,%.8f,%llu,%.8f,%.8f,%.8f\n",
                                     static_cast<double>(data.time),
                                     data.queue_size,
                                     static_cast<double>(data.motor_target), static_cast<double>(data.motor_pos),
                                     static_cast<double>(data.motor_velocity),
                                     static_cast<double>(data.motor_acceleration),
                                     data.motor_nsec_per_pulse,
                                     static_cast<double>(data.pt202),
                                     static_cast<double>(data.pt203), static_cast<double>(data.ptf401));
        // snprintfcb's would_write excludes null byte, but max via MAX_DATA_LEN would include null byte.
        int actually_written = std::min(would_write, MAX_DATA_LEN - 1);
        err = send_fully(data_sock, buf, actually_written);
        if (err) {
            LOG_WRN("Failed to send data");
        }
    }

    send_string_fully(data_sock, ">>>>SEQ END<<<<\n");

    // Next data recipient should be explicitly re-set.
    data_sock = -1;
    k_mutex_unlock(&sequence_lock);
    return 0;
}

int sequencer_prepare(int gap, std::vector<float> bps) {
    if (gap <= 0 || bps.empty()) {
        return 1;
    }

    gap_millis = gap;
    breakpoints = bps;
    return 0;
}

void sequencer_set_data_recipient(int sock) {
    k_mutex_lock(&sequence_lock, K_FOREVER);
    data_sock = sock;
    k_mutex_unlock(&sequence_lock);
}
