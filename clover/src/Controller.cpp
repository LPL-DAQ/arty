#include "Controller.h"
#include "ControllerConfig.h"
#include "server.h"
#include "config.h"
#include <zephyr/kernel.h>
#include <zephyr/kernel/thread_stack.h>
#include <zephyr/logging/log.h>

LOG_MODULE_REGISTER(Controller, LOG_LEVEL_INF);

K_MSGQ_DEFINE(telemetry_msgq, sizeof(DataPacket), 50, 1);

// Controller tick workqueue thread
K_THREAD_STACK_DEFINE(controller_step_thread_stack, 4096);
k_work_q controller_step_work_q;

std::expected<void, Error> Controller::change_state(SystemState new_state)
{
    if (current_state == new_state)
        return {};

    return {};
}

K_WORK_DEFINE(control_loop, Controller::step_control_loop);

// ISR that schedules a control iteration in the work queue.
static void control_loop_schedule(k_timer* timer)
{
    k_work_submit(&control_loop);
}

K_TIMER_DEFINE(control_loop_schedule_timer, control_loop_schedule, nullptr);

std::expected<void, Error> Controller::init()
{
    LOG_INF("Triggering initial sensor readings");
    k_sched_lock();
    AnalogSensors::start_sense();
    // Other sensors here...
    k_sched_unlock();

    LOG_INF("Pausing for initial sensor readings to complete");
    k_sleep(K_MSEC(500));

    // Set up workqueue
    LOG_INF("Initializing workqueue");
    k_work_queue_init(&controller_step_work_q);
    k_work_queue_start(
        &controller_step_work_q, controller_step_thread_stack, K_THREAD_STACK_SIZEOF(controller_step_thread_stack), CONTROLLER_STEP_WORK_Q_PRIORITY, nullptr);

    auto ret = change_state(SystemState_STATE_IDLE);
    if (!ret.has_value()) {
        return std::unexpected(ret.error().context("Failed to change state to idle"));
    }
    LOG_INF("Beginning controller ticks");
    k_timer_start(&control_loop_schedule_timer, K_NSEC(NSEC_PER_CONTROL_TICK), K_NSEC(NSEC_PER_CONTROL_TICK));
    return {};
}

static int step_control_loop_debounce_warn_count = 0;

void Controller::step_control_loop(k_work*)
{
    int64_t current_time = k_uptime_get();
    uint64_t start_cycle = k_cycle_get_64();
    DataPacket data = DataPacket_init_default;

    // Read sensors
    auto analog_sensors_readings = AnalogSensors::read();
    if (analog_sensors_readings) {
        std::tie(data.analog_sensors, data.controller_timing.analog_sensors_sense_time_ns) = *analog_sensors_readings;
    }
    else {
        // LOG_WRN("Analog sensor data is not yet ready, leaving defaults.");
    }

    daq_client_status daq_status = get_daq_client_status();




}

std::expected<void, Error> Controller::handle_abort(const AbortRequest& req)
{
    LOG_INF("Received abort request");
    abort_entry_time = k_uptime_get();
    auto ret = change_state(SystemState_STATE_ABORT);
    if (!ret.has_value()) {
        return std::unexpected(ret.error().context("Failed to change state to abort"));
    }
    return {};
}

const char* Controller::get_state_name(SystemState state)
{
    if (state == SystemState_STATE_IDLE)
        return "Idle";
    if (state == SystemState_STATE_CALIBRATE_VALVE)
        return "Calibrate Valve";
    if (state == SystemState_STATE_VALVE_PRIMED)
        return "Valve Primed";
    if (state == SystemState_STATE_VALVE_SEQ)
        return "Valve Seq";
    if (state == SystemState_STATE_THRUST_PRIMED)
        return "Thrust Primed";
    if (state == SystemState_STATE_THRUST_SEQ)
        return "Thrust Seq";
    if (state == SystemState_STATE_ABORT)
        return "Abort";
    return "Unknown State";  // Unknown state
}
