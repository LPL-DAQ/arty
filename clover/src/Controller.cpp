#include "Controller.h"
#include "ControllerConfig.h"
#include "flight/FlightController.h"
#include "server.h"
#include "config.h"
#include "throttle/ThrottleController.h"
#include "rcs/RCSController.h"
#include "tvc/TVCController.h"
#include <zephyr/kernel.h>
#include <zephyr/kernel/thread_stack.h>
#include <zephyr/logging/log.h>

LOG_MODULE_REGISTER(Controller, LOG_LEVEL_INF);

K_MSGQ_DEFINE(telemetry_msgq, sizeof(DataPacket), 50, 1);

// Controller tick workqueue thread
K_THREAD_STACK_DEFINE(controller_step_thread_stack, 4096);
k_work_q controller_step_work_q;

bool Controller::should_tick_throttle()
{
    return current_state != SystemState_STATE_RCS && current_state != SystemState_STATE_TVC && current_state != SystemState_STATE_UNKNOWN && current_state != SystemState_STATE_IDLE;
}

bool Controller::should_tick_tvc()
{
    return current_state != SystemState_STATE_THROTTLE && current_state != SystemState_STATE_RCS && current_state != SystemState_STATE_UNKNOWN && current_state != SystemState_STATE_IDLE;
}

bool Controller::should_tick_rcs()
{
    return current_state != SystemState_STATE_THROTTLE && current_state != SystemState_STATE_TVC && current_state != SystemState_STATE_UNKNOWN && current_state != SystemState_STATE_IDLE;
}

bool Controller::should_tick_flight()
{
    return current_state != SystemState_STATE_THROTTLE && current_state != SystemState_STATE_TVC && current_state != SystemState_STATE_RCS && current_state != SystemState_STATE_TVC_THROTTLE && current_state != SystemState_STATE_TVC_THROTTLE_RCS && current_state != SystemState_STATE_UNKNOWN && current_state != SystemState_STATE_IDLE;
}

std::expected<void, Error> Controller::attempt_abort_subsystems()
{
    std::expected<void, Error> ret = {};

    switch (ThrottleController::current_state) {
    case ThrottleState_THROTTLE_STATE_VALVE_SEQ:
    case ThrottleState_THROTTLE_STATE_THRUST_SEQ:
    case ThrottleState_THROTTLE_STATE_FLIGHT:
        ret = ThrottleController::change_state(ThrottleState_THROTTLE_STATE_ABORT);
        if (!ret.has_value()) return ret;
        break;
    default:
        break;
    }

    switch (TVCController::current_state) {
    case TVCState_TVC_STATE_TRACE:
    case TVCState_TVC_STATE_FLIGHT:
        ret = TVCController::change_state(TVCState_TVC_STATE_ABORT);
        if (!ret.has_value()) return ret;
        break;
    default:
        break;
    }

    switch (RCSController::current_state) {
    case RCSState_RCS_STATE_VALVE_SEQ:
    case RCSState_RCS_STATE_ROLL_SEQ:
    case RCSState_RCS_STATE_FLIGHT:
        ret = RCSController::change_state(RCSState_RCS_STATE_ABORT);
        if (!ret.has_value()) return ret;
        break;
    default:
        break;
    }

    switch (FlightController::current_state) {
    case FlightState_FLIGHT_STATE_TAKEOFF:
    case FlightState_FLIGHT_STATE_FLIGHT_SEQ:
    case FlightState_FLIGHT_STATE_LANDING:
        ret = FlightController::change_state(FlightState_FLIGHT_STATE_ABORT);
        if (!ret.has_value()) return ret;
        break;
    default:
        break;
    }

    return {};
}

std::expected<void, Error> Controller::change_state(SystemState new_state)
{
    if (current_state == new_state)
        return {};

    if (new_state == SystemState_STATE_IDLE) {
        std::expected<void, Error> ret = {};

        if (ThrottleController::current_state != ThrottleState_THROTTLE_STATE_OFF) {
            ret = ThrottleController::change_state(ThrottleState_THROTTLE_STATE_IDLE);
            if (!ret.has_value()) {
                return std::unexpected(ret.error().context("Failed to set throttle controller idle"));
            }
        }

        if (TVCController::current_state != TVCState_TVC_STATE_OFF) {
            ret = TVCController::change_state(TVCState_TVC_STATE_IDLE);
            if (!ret.has_value()) {
                return std::unexpected(ret.error().context("Failed to set TVC controller idle"));
            }
        }

        if (RCSController::current_state != RCSState_RCS_STATE_OFF) {
            ret = RCSController::change_state(RCSState_RCS_STATE_IDLE);
            if (!ret.has_value()) {
                return std::unexpected(ret.error().context("Failed to set RCS controller idle"));
            }
        }

        if (FlightController::current_state != FlightState_FLIGHT_STATE_OFF) {
            ret = FlightController::change_state(FlightState_FLIGHT_STATE_IDLE);
            if (!ret.has_value()) {
                return std::unexpected(ret.error().context("Failed to set flight controller idle"));
            }
        }

        current_state = new_state;
        return {};
    }

    if (new_state == SystemState_STATE_ABORT) {
        abort_entry_time = k_uptime_get();
        auto ret = attempt_abort_subsystems();
        if (!ret.has_value()) {
            return std::unexpected(ret.error().context("Failed to cascade abort to subsystems"));
        }

        current_state = new_state;
        return {};
    }

    current_state = new_state;
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
    // TODO: What was start_cycle used for?
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

    // Dispatch to appropriate peripheral controllers based on current state
    // TODO: Check that data passing works properly
    // TODO: what exactly happens when system state is idle?
    if (should_tick_flight()) {
        FlightController::step_control_loop(data, analog_sensors_readings);
    } else {
        FlightController::change_state(FlightState_FLIGHT_STATE_OFF);
    }
    if (should_tick_throttle()) {
        ThrottleController::step_control_loop(data, analog_sensors_readings);
    } else {
        ThrottleController::change_state(ThrottleState_THROTTLE_STATE_OFF);
    }
    if (should_tick_tvc()) {
        TVCController::step_control_loop(data, analog_sensors_readings);
    } else {
        TVCController::change_state(TVCState_TVC_STATE_OFF);
    }
    if (should_tick_rcs()) {
        RCSController::step_control_loop(data, analog_sensors_readings);
    } else {
        RCSController::change_state(RCSState_RCS_STATE_OFF);
    }

    if (k_msgq_put(&telemetry_msgq, &data, K_NO_WAIT) != 0) {
        if (step_control_loop_debounce_warn_count < 5) {
            LOG_WRN("Telemetry queue full, packet dropped");
        }
        else if (step_control_loop_debounce_warn_count == 5) {
            LOG_WRN("Telemetry queue full, packet dropped (silencing further warnings)");
        }
        step_control_loop_debounce_warn_count++;
    }
    else {
        // Reset warning count
        step_control_loop_debounce_warn_count = 0;
    }

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
    if (state == SystemState_STATE_ABORT)
        return "Abort";
    if (state == SystemState_STATE_FLIGHT)
        return "Flight";
    if (state == SystemState_STATE_THROTTLE)
        return "Throttle";
    if (state == SystemState_STATE_TVC)
        return "TVC";
    if (state == SystemState_STATE_RCS)
        return "RCS";
    if (state == SystemState_STATE_TVC_THROTTLE)
        return "TVC_Throttle";
    if (state == SystemState_STATE_TVC_THROTTLE_RCS)
        return "TVC_Throttle_RCS";
    return "Unknown State";  // Unknown state
}

