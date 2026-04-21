#include "Controller.h"
#include "MutexGuard.h"
#include "config.h"
#include "sensors/AnalogSensors.h"
#include "server.h"
#include "util.h"
#include "flight/FlightController.h"
#include "flight/StateEstimator.h"

#include <zephyr/kernel.h>
#include <zephyr/kernel/thread_stack.h>
#include <zephyr/logging/log.h>


#if CONFIG_RANGER
#include "ranger/RangerRcs.h"
#include "ranger/RangerThrottle.h"
#include "ranger/RangerTvc.h"
#include "ranger/ThrottleValve.h"

#elif CONFIG_HORNET
#include "hornet/PwmActuator.h"
#include "hornet/HornetRcs.h"
#include "hornet/HornetThrottle.h"
#include "hornet/HornetTvc.h"

#else
#error "No configuration defined. Please define CONFIG_RANGER or CONFIG_HORNET in your build configuration."
#endif

LOG_MODULE_REGISTER(Controller, LOG_LEVEL_INF);

K_MSGQ_DEFINE(telemetry_msgq, sizeof(DataPacket), 50, 1);

// Controller tick workqueue thread
K_THREAD_STACK_DEFINE(controller_step_thread_stack, 4096);
k_work_q controller_step_work_q;

static inline SystemState current_state = SystemState_STATE_IDLE;

/// Count of how many packets had to be dropped, used for logging. Owned by controller tick workqueue thread.
static int recent_packets_attempted = 0;
static int recent_packets_dropped = 0;

/// Store previous actuator command. Actuators in idle states simply repeat whatever their last command was.
#ifdef CONFIG_RANGER
auto prev_fuel_valve_command = ThrottleValveCommand{.enable = true, .set_pos = false, .target_deg = 0.0f};
auto prev_lox_valve_command = ThrottleValveCommand{.enable = true, .set_pos = false, .target_deg = 0.0f};
auto prev_pitch_actuator_command = TvcActuatorCommand{};
auto prev_yaw_actuator_command = TvcActuatorCommand{};
#elif CONFIG_HORNET
float prev_main_propeller_command = 0.0f;
float prev_rcs_propeller_cw_command = 0.0f;
float prev_rcs_propeller_ccw_command = 0.0f;
float prev_pitch_servo_command = 0.0f;
float prev_yaw_servo_command = 0.0f;
#endif

// State-specific data. Each is only valid within their respective states.

// Valid in THROTTLE, TVC, RCS, and FLIGHT.
static uint64_t trace_start_cycle = 0;
static float trace_total_time_msec = 0;

// Valid in ABORT.
static uint64_t abort_start_cycle = 0;

// Valid in THROTTLE_PRIMED and THROTTLE
static Trace throttle_thrust_trace_N;

// Valid in TVC_PRIMED and TVC
static Trace tvc_pitch_trace_deg;
static Trace tvc_yaw_trace_deg;

// Valid in RCS_PRIMED and RCS
static Trace rcs_roll_trace_deg;

// Valid in FLIGHT_PRIMED and PRIMED
static Trace flight_x_trace_m;
static Trace flight_y_trace_m;
static Trace flight_z_trace_m;
static Trace flight_roll_trace_deg;

// Valid in THROTTLE_VALVE_PRIMED and THROTTLE_VALVE, and if CONFIG_RANGER is set.
static bool has_fuel_valve_trace = false;
static bool has_lox_valve_trace = false;
static Trace throttle_fuel_valve_trace_deg;
static Trace throttle_lox_valve_trace_deg;

// Valid in RCS_VALVE_PRIMED and RCS_VALVE.
static Trace rcs_cw_valve_trace;
static Trace rcs_ccw_valve_trace;

/// This lock must be acquired for anything reliant on Controller state.
K_MUTEX_DEFINE(controller_state_lock);

/// Sets up a few actuators during idle state transitions. Specifically required when just holding the previous
/// state is not desirable (e.g., valves that would keep venting, or motors that'd keep spinning). Must be called
/// when state lock is held.
static void setup_idle()
{
#ifdef CONFIG_RANGER
    // TODO -- kill RCS actuators.
#elif CONFIG_HORNET
    prev_main_propeller_command = 0.0f;
    prev_rcs_propeller_cw_command = 0.0f;
    prev_rcs_propeller_ccw_command = 0.0f;
#endif
}

/// Work item for each control loop tick
static void step_control_loop(k_work*);
K_WORK_DEFINE(step_control_loop_work, step_control_loop);

/// ISR that schedules a control iteration in the work queue.
static void control_loop_schedule(k_timer* timer)
{
    k_work_submit_to_queue(&controller_step_work_q, &step_control_loop_work);
}

K_TIMER_DEFINE(control_loop_schedule_timer, control_loop_schedule, nullptr);


//TODO roll control. the module should not accept a position as that is active control

/// Transform actuator commands into actuator commands, modifying the data pcket in-place.
/// If an abort is necessary, an Error is returned. This is called for all active control
/// states. trace_time_msec must be pre-populated.
static std::expected<void, Error> tick_active_control(DataPacket& data)
{
    // Throttle and RCS valves have traces which direct control the actuator.
    if (current_state == SystemState_STATE_THROTTLE_VALVE) {
        if (has_fuel_valve_trace) {
            auto fuel_sample = throttle_fuel_valve_trace_deg.sample(data.trace_time_msec);
            if (!fuel_sample.has_value()) {
                return std::unexpected(fuel_sample.error().context("failed to sample throttle fuel valve trace"));
            }
            data.fuel_valve_command = ThrottleValveCommand{.enable = true, .set_pos = false, .target_deg = *fuel_sample};
        }

        if (has_lox_valve_trace) {
            auto lox_sample = throttle_lox_valve_trace_deg.sample(data.trace_time_msec);
            if (!lox_sample.has_value()) {
                return std::unexpected(lox_sample.error().context("failed to sample throttle lox valve trace"));
            }
            data.lox_valve_command = ThrottleValveCommand{.enable = true, .set_pos = false, .target_deg = *lox_sample};
        }

        return {};
    }
    else if (current_state == SystemState_STATE_RCS_VALVE) {
        auto cw_sample = rcs_cw_valve_trace.sample(data.trace_time_msec);
        if (!cw_sample.has_value()) {
            return std::unexpected(cw_sample.error().context("failed to sample rcs cw valve trace"));
        }
        // TODO -- command RCS valve

        auto ccw_sample = rcs_ccw_valve_trace.sample(data.trace_time_msec);
        if (!ccw_sample.has_value()) {
            return std::unexpected(ccw_sample.error().context("failed to sample rcs ccw valve trace"));
        }
        // TODO -- command RCS valve

        return {};
    }

    // Sample flight trace and run flight controller
    if (current_state == SystemState_STATE_FLIGHT) {
    // Sample flight trace and run flight controller
    if (current_state == SystemState_STATE_FLIGHT) {
        // Sample flight traces
        auto x_sample = flight_x_trace_m.sample(data.trace_time_msec);
        if (!x_sample.has_value()) {
            return std::unexpected(x_sample.error().context("failed to sample flight x trace"));
        }
        data.has_flight_x_command_m = true;
        data.flight_x_command_m = *x_sample;

        auto y_sample = flight_y_trace_m.sample(data.trace_time_msec);
        if (!y_sample.has_value()) {
            return std::unexpected(y_sample.error().context("failed to sample flight y trace"));
        }
        data.has_flight_y_command_m = true;
        data.flight_y_command_m = *y_sample;

        auto z_sample = flight_z_trace_m.sample(data.trace_time_msec);
        if (!z_sample.has_value()) {
            return std::unexpected(z_sample.error().context("failed to sample flight z trace"));
        }
        data.has_flight_z_command_m = true;
        data.flight_z_command_m = *z_sample;

        auto roll_sample = flight_roll_trace_deg.sample(data.trace_time_msec);
        if (!roll_sample.has_value()) {
            return std::unexpected(roll_sample.error().context("failed to sample flight roll trace"));
        }
        data.has_rcs_roll_command_deg = true;
        data.rcs_roll_command_deg = *roll_sample;

        // Execute flight controller to generate flight commands
        auto flight_response = FlightController::tick(data.estimated_state, data.flight_x_command_m, data.flight_y_command_m, data.flight_z_command_m);
        if (!flight_response.has_value()) {
            return std::unexpected(flight_response.error().context("error in FlightController"));
        }
        data.has_throttle_thrust_command_N = true;
        data.has_tvc_pitch_command_deg = true;
        data.has_tvc_yaw_command_deg = true;
        data.has_flight_controller_metrics = true;
        std::tie(
            data.throttle_thrust_command_N, data.tvc_pitch_command_deg, data.tvc_yaw_command_deg, data.flight_controller_metrics) =
            *flight_response;
    }

    // Sample throttle trace
    if (current_state == SystemState_STATE_STATIC_FIRE || current_state == SystemState_STATE_THROTTLE) {
    // Sample throttle trace
    if (current_state == SystemState_STATE_STATIC_FIRE || current_state == SystemState_STATE_THROTTLE) {
        // Sample thrust trace
        auto thrust_sample = throttle_thrust_trace_N.sample(data.trace_time_msec);
        if (!thrust_sample.has_value()) {
            return std::unexpected(thrust_sample.error().context("failed to sample throttle thrust trace"));
        }
        data.has_throttle_thrust_command_N = true;
        data.throttle_thrust_command_N = *thrust_sample;
    }

    // Sample TVC trace
    if (current_state == SystemState_STATE_STATIC_FIRE || current_state == SystemState_STATE_TVC) {
        // Sample TVC trace
        auto pitch_sample = tvc_pitch_trace_deg.sample(data.trace_time_msec);
        if (!pitch_sample.has_value()) {
            return std::unexpected(pitch_sample.error().context("failed to sample tvc pitch trace"));
        }
        data.has_tvc_pitch_command_deg = true;
        data.tvc_pitch_command_deg = *pitch_sample;

        auto yaw_sample = tvc_yaw_trace_deg.sample(data.trace_time_msec);
        if (!yaw_sample.has_value()) {
            return std::unexpected(yaw_sample.error().context("failed to sample tvc yaw trace"));
        }
        data.has_tvc_yaw_command_deg = true;
        data.tvc_yaw_command_deg = *yaw_sample;
    }

    if (current_state == SystemState_STATE_RCS) {
    if (current_state == SystemState_STATE_RCS) {
        // Sample TVC trace
        auto roll_sample = tvc_pitch_trace_deg.sample(data.trace_time_msec);
        if (!roll_sample.has_value()) {
            return std::unexpected(roll_sample.error().context("failed to sample rcs roll trace"));
        }
        data.has_rcs_roll_command_deg = true;
        data.rcs_roll_command_deg = *roll_sample;
    }

    // Execute subordinate controllers
    if (current_state == SystemState_STATE_THROTTLE || current_state == SystemState_STATE_FLIGHT || current_state == SystemState_STATE_STATIC_FIRE) {
        if (!data.has_throttle_thrust_command_N) {
            return std::unexpected(Error::from_cause("missing throttle thrust command"));
        }
#ifdef CONFIG_RANGER
        auto throttle_response = RangerThrottle::tick(data.analog_sensors, data.throttle_thrust_command_N);
        if (!throttle_response.has_value()) {
            return std::unexpected(throttle_response.error().context("error in RangerThrottle"));
        }
        data.has_fuel_valve_command = true;
        data.has_lox_valve_command = true;
        data.has_ranger_throttle_metrics = true;
        std::tie(data.fuel_valve_command, data.lox_valve_command, data.ranger_throttle_metrics) = *throttle_response;

#elif CONFIG_HORNET
        auto throttle_response = HornetThrottle::tick(data.throttle_thrust_command_N);
        if (!throttle_response.has_value()) {
            return std::unexpected(throttle_response.error().context("error in HornetThrottle"));
        }
        data.has_main_propeller_command = true;
        data.has_hornet_throttle_metrics = true;
        std::tie(data.main_propeller_command, data.hornet_throttle_metrics) = *throttle_response;
#endif
    }

    if (current_state == SystemState_STATE_TVC || current_state == SystemState_STATE_FLIGHT || current_state == SystemState_STATE_STATIC_FIRE) {
    if (current_state == SystemState_STATE_TVC || current_state == SystemState_STATE_FLIGHT || current_state == SystemState_STATE_STATIC_FIRE) {
        if (!data.has_tvc_pitch_command_deg) {
            return std::unexpected(Error::from_cause("missing tvc pitch command"));
        }
        if (!data.has_tvc_yaw_command_deg) {
            return std::unexpected(Error::from_cause("missing tvc pitch command"));
        }

#ifdef CONFIG_RANGER
        auto tvc_response = RangerTvc::tick(data.tvc_pitch_command_deg, data.tvc_yaw_command_deg);
        if (!tvc_response.has_value()) {
            return std::unexpected(tvc_response.error().context("error in RangerTvc"));
        }
        data.has_pitch_actuator_command = true;
        data.has_yaw_actuator_command = true;
        data.has_ranger_tvc_metrics = true;
        std::tie(data.pitch_actuator_command, data.yaw_actuator_command, data.ranger_tvc_metrics) = *tvc_response;

#elif CONFIG_HORNET
        auto tvc_response = HornetTvc::tick(data.tvc_pitch_command_deg, data.tvc_yaw_command_deg);
        if (!tvc_response.has_value()) {
            return std::unexpected(tvc_response.error().context("error in HornetTvc"));
        }
        data.has_pitch_servo_command = true;
        data.has_yaw_servo_command = true;
        data.has_hornet_tvc_metrics = true;
        std::tie(data.pitch_servo_command, data.yaw_servo_command, data.hornet_tvc_metrics) = *tvc_response;
#endif
    }

    if (current_state == SystemState_STATE_RCS || current_state == SystemState_STATE_FLIGHT) {
        if (!data.has_rcs_roll_command_deg) {
            return std::unexpected(Error::from_cause("missing tvc pitch command"));
        }

#ifdef CONFIG_RANGER
        auto rcs_response = RangerRcs::tick(data.rcs_roll_command_deg);
        if (!rcs_response.has_value()) {
            return std::unexpected(rcs_response.error().context("error in RangerRcs"));
        }
        // TODO
        // ?? VALVES
        // TODO
        data.has_ranger_rcs_metrics = true;
        // TODO BELOW IS MEGA PLACEHOLDER
        std::tie(data.has_pitch_servo_command, data.has_pitch_servo_command, data.ranger_rcs_metrics) = *rcs_response;

#elif CONFIG_HORNET
        auto rcs_response = HornetRcs::tick(data.estimated_state, data.rcs_roll_command_deg);
        if (!rcs_response.has_value()) {
            return std::unexpected(rcs_response.error().context("error in HornetRcs"));
        }
        data.has_rcs_propeller_cw_command = true;
        data.has_rcs_propeller_ccw_command = true;
        data.has_hornet_rcs_metrics = true;
        std::tie(data.rcs_propeller_cw_command, data.rcs_propeller_ccw_command, data.hornet_rcs_metrics) = *rcs_response;
#endif
    }

    return {};
}


// TODO: from noah: i feel like something should be here, no?
static void tick_abort(DataPacket& data)
{
}

/// Kicks off the controller workqueue. Last phase of initial setup in the main function.
std::expected<void, Error> Controller::init()
{

    LOG_INF("Triggering initial sensor readings");
    k_sched_lock();
    AnalogSensors::start_sense();
    StateEstimator::init();
    // Other sensors here...
    k_sched_unlock();

    LOG_INF("Pausing for initial sensor readings to complete");
    k_sleep(K_MSEC(500));

    // Set up workqueue
    LOG_INF("Initializing workqueue");
    k_work_queue_init(&controller_step_work_q);
    k_work_queue_start(
        &controller_step_work_q, controller_step_thread_stack, K_THREAD_STACK_SIZEOF(controller_step_thread_stack), CONTROLLER_STEP_WORK_Q_PRIORITY, nullptr);

    LOG_INF("Beginning controller ticks");
    k_timer_start(&control_loop_schedule_timer, K_NSEC(Controller::NSEC_PER_CONTROL_TICK), K_NSEC(Controller::NSEC_PER_CONTROL_TICK));

    return {};
}

/// Execute one tick of the top-level controller.
static void step_control_loop(k_work*)
{
    MutexGuard current_state_guard{&controller_state_lock};

    uint64_t start_cycle = k_cycle_get_64();
    DataPacket data = DataPacket_init_default;
    data.state = current_state;

    // Read sensors
    auto analog_sensors_readings = AnalogSensors::read();
    if (analog_sensors_readings) {
        std::tie(data.analog_sensors, data.controller_timing.analog_sensors_sense_time_ns) = *analog_sensors_readings;
    }
    else {
        // LOG_WRN("Analog sensor data is not yet ready, leaving defaults.");
    }

    // TODO: dont provide whole data, this is temp caause we dont have the sensors
    auto estimated_state = StateEstimator::estimate(data);
    if (estimated_state) {
        data.estimated_state = *estimated_state;
    } else {
        // TODO: handle estimate failure; leaving defaults for now
    }

    daq_client_status daq_status = get_daq_client_status();

    // Populate default actuator commands -- essentially telling everybody to hold their current state.
    {
#ifdef CONFIG_RANGER
        data.has_fuel_valve_command = true;
        data.fuel_valve_command = prev_fuel_valve_command;

        data.has_lox_valve_command = true;
        data.lox_valve_command = prev_lox_valve_command;

        data.has_pitch_actuator_command = true;
        data.pitch_actuator_command = prev_pitch_actuator_command;

        data.has_yaw_actuator_command = true;
        data.yaw_actuator_command = prev_yaw_actuator_command;
#elif CONFIG_HORNET
        data.has_main_propeller_command = true;
        data.main_propeller_command = prev_main_propeller_command;

        data.has_rcs_propeller_cw_command = true;
        data.rcs_propeller_cw_command = prev_rcs_propeller_cw_command;

        data.has_rcs_propeller_ccw_command = true;
        data.rcs_propeller_ccw_command = prev_rcs_propeller_ccw_command;

        data.has_pitch_servo_command = true;
        data.pitch_servo_command = prev_pitch_servo_command;

        data.has_yaw_servo_command = true;
        data.yaw_servo_command = prev_yaw_servo_command;
#endif
    }

    // Transform sensor data into actuator commands. Different logic paths are applied via state machine.
    switch (current_state) {
    case SystemState_STATE_IDLE:
    case SystemState_STATE_THROTTLE_PRIMED:
    case SystemState_STATE_TVC_PRIMED:
    case SystemState_STATE_RCS_PRIMED:
    case SystemState_STATE_FLIGHT_PRIMED:
    case SystemState_STATE_STATIC_FIRE_PRIMED:
    case SystemState_STATE_THROTTLE_VALVE_PRIMED:
    case SystemState_STATE_RCS_VALVE_PRIMED: {
        // Maintain current positions.
        break;
    }

    // Commands all actuators back to nominal states.
    case SystemState_STATE_ABORT: {
        // TODO
        data.has_abort_time_msec = true;
        data.abort_time_msec = nsec_since_cycle(abort_start_cycle) / 1e6f;
        tick_abort(data);

        if (data.abort_time_msec > Controller::ABORT_TIME_MSEC) {
            LOG_INF("ABORT finished, entering IDLE");
            current_state = SystemState_STATE_IDLE;
        }
        break;
    }

    // Calibration of throttle valves.
    case SystemState_STATE_CALIBRATE_THROTTLE_VALVE:
        // TODO
        break;
    case SystemState_STATE_CALIBRATE_TVC:
        // TODO
        break;
    // Active control through traces.
    case SystemState_STATE_THROTTLE:
    case SystemState_STATE_TVC:
    case SystemState_STATE_RCS:
    case SystemState_STATE_FLIGHT:
    case SystemState_STATE_STATIC_FIRE:
    case SystemState_STATE_THROTTLE_VALVE:
    case SystemState_STATE_RCS_VALVE: {
        // Get msec since start of trace.
        data.trace_time_msec = nsec_since_cycle(trace_start_cycle) / 1e6f;
        data.has_trace_time_msec = true;

        // Dispatch to active control handler, which returns an Error if an abort is necessary.
        auto result = tick_active_control(data);

        if (!result.has_value()) {
            // Abort case may leave some actuators partially set, depending on when the error occurs.
            LOG_ERR("ABORTING: %s", result.error().build_message().c_str());

            abort_start_cycle = k_cycle_get_64();
            current_state = SystemState_STATE_ABORT;
        }
        else {
            // Transition to IDLE upon completion.
            if (data.trace_time_msec > trace_total_time_msec) {
                LOG_INF("Trace finished, entering IDLE");
                current_state = SystemState_STATE_IDLE;
            }
        }
        break;
    }

    default: {
        LOG_ERR("Invalid controller state: %d", current_state);
    }
    }

    // Dispatch commands to actuators
#if CONFIG_RANGER
    FuelValve::tick(data.fuel_valve_command);
    LoxValve::tick(data.lox_valve_command);
    // RCS valves, TVC actuators
#elif CONFIG_HORNET
    // TODO: do something if these return an error
    // TVC
    auto err = ServoX::tick(data.pitch_servo_command);
    err = ServoY::tick(data.yaw_servo_command);

    // RCS
    // TODO: Check which actually corresponds to what
    err = BetaTop::tick(data.rcs_propeller_cw_command);
    err = BetaCW::tick(data.rcs_propeller_cw_command);
    err = BetaBottom::tick(data.rcs_propeller_ccw_command);
    err = BetaCCW::tick(data.rcs_propeller_ccw_command);

    err = MotorTop::tick(data.main_propeller_command);
    err = MotorBottom::tick(data.main_propeller_command);


#endif

    // Update last-executed actuator commands
#ifdef CONFIG_RANGER
    prev_fuel_valve_command = data.fuel_valve_command;
    prev_lox_valve_command = data.lox_valve_command;
    prev_pitch_actuator_command = data.pitch_actuator_command;
    prev_yaw_actuator_command = data.yaw_actuator_command;
#elif CONFIG_HORNET
    prev_main_propeller_command = data.main_propeller_command;
    prev_rcs_propeller_cw_command = data.rcs_propeller_cw_command;
    prev_rcs_propeller_ccw_command = data.rcs_propeller_ccw_command;
    prev_pitch_servo_command = data.pitch_servo_command;
    prev_yaw_servo_command = data.yaw_servo_command;
#endif

    // Record how long controller tick calculations took.
    data.controller_timing.controller_tick_time_ns = nsec_since_cycle(start_cycle);

    // Transmit data packet. Log metrics on packet transmission success rates.
    recent_packets_attempted++;
    if (k_msgq_put(&telemetry_msgq, &data, K_NO_WAIT) != 0) {
        recent_packets_dropped++;
    }
    if (recent_packets_attempted == 10000 && recent_packets_dropped > 0) {
        float percent_packets_dropped = static_cast<float>(recent_packets_dropped) / recent_packets_attempted * 100.0f;
        LOG_WRN(
            "Packets were dropped in the past 10 sec: %f%% (%d dropped, %d attempted)",
            static_cast<double>(percent_packets_dropped),
            recent_packets_dropped,
            recent_packets_attempted);
        recent_packets_attempted = 0;
        recent_packets_dropped = 0;
    }
}

/// Retrieves a data packet from the telemetry message queue.
DataPacket Controller::get_next_data_packet()
{
    DataPacket packet;
    int err = k_msgq_get(&telemetry_msgq, &packet, K_FOREVER);
    if (err) {
        // Should be impossible, as this only fails if the queue is purged or if we timeout waiting.
        LOG_ERR("Failed to get packed from telemetry message queue: %d", err);
    }
    return packet;
}

// Request handlers.

/// Abort, returning the system to a safe state.
std::expected<void, Error> Controller::handle_abort(const AbortRequest& req)
{
    LOG_WRN("CLIENT TRIGGERED ABORT");
    MutexGuard guard{&controller_state_lock};

    abort_start_cycle = k_cycle_get_64();
    current_state = SystemState_STATE_ABORT;
    return {};
}

/// Client-triggered transition from any active control state to IDLE.
std::expected<void, Error> Controller::handle_halt(const HaltRequest& req)
{
    MutexGuard guard{&controller_state_lock};

    switch (current_state) {
    case SystemState_STATE_THROTTLE:
    case SystemState_STATE_TVC:
    case SystemState_STATE_RCS:
    case SystemState_STATE_FLIGHT:
    case SystemState_STATE_STATIC_FIRE:
    case SystemState_STATE_THROTTLE_VALVE:
    case SystemState_STATE_RCS_VALVE:
        break;
    default:
        return std::unexpected(Error::from_cause("must be in an active control state to halt"));
    }

    current_state = SystemState_STATE_IDLE;
    setup_idle();
    LOG_INF("Halting sequence");

    return {};
}

/// Client-triggered transition from any PRIMED state to IDLE.
std::expected<void, Error> Controller::handle_unprime(const UnprimeRequest& req)
{
    MutexGuard guard{&controller_state_lock};

    switch (current_state) {
    case SystemState_STATE_THROTTLE_PRIMED:
    case SystemState_STATE_TVC_PRIMED:
    case SystemState_STATE_RCS_PRIMED:
    case SystemState_STATE_FLIGHT_PRIMED:
    case SystemState_STATE_STATIC_FIRE_PRIMED:
    case SystemState_STATE_THROTTLE_VALVE_PRIMED:
    case SystemState_STATE_RCS_VALVE_PRIMED:
        break;
    default:
        return std::unexpected(Error::from_cause("must be in a primed state to unprime"));
    }

    current_state = SystemState_STATE_IDLE;
    setup_idle();
    LOG_INF("Unpriming sequence");

    return {};
}

/// Client-triggered transition from IDLE to CALIBRATE_THROTTLE_VALVE.
std::expected<void, Error> Controller::handle_calibrate_throttle_valve(const CalibrateThrottleValveRequest& req)
{
    MutexGuard guard{&controller_state_lock};
    if (current_state != SystemState_STATE_IDLE) {
        return std::unexpected(Error::from_cause("throttle calibration rejected unless system is idle"));
    }

#if CONFIG_HORNET
    return std::unexpected(Error::from_cause("must be ranger config for throttle valve calibration"));
#endif

    current_state = SystemState_STATE_CALIBRATE_THROTTLE_VALVE;

    return {};
}

/// Client-triggered transition from IDLE to THROTTLE_VALVE_PRIMED
std::expected<void, Error> Controller::handle_load_throttle_valve_sequence(const LoadThrottleValveSequenceRequest& req)
{
    MutexGuard guard{&controller_state_lock};

    if (current_state != SystemState_STATE_IDLE) {
        return std::unexpected(Error::from_cause("throttle valve sequence load rejected unless system is idle"));
    }

#if CONFIG_HORNET
    return std::unexpected(Error::from_cause("must be ranger config for throttle valve sequences"));
#endif

    // At least one trace must be specified
    if (!req.has_fuel_trace_deg && !req.has_lox_trace_deg) {
        return std::unexpected(Error::from_cause("must specify at least one valve trace"));
    }

    // Try loading fuel trace
    if (req.has_fuel_trace_deg) {
        auto ret = throttle_fuel_valve_trace_deg.load(req.fuel_trace_deg);
        if (!ret.has_value()) {
            return std::unexpected(ret.error().context("failed to load fuel valve trace"));
        }
        has_fuel_valve_trace = true;
        trace_total_time_msec = throttle_fuel_valve_trace_deg.get_total_time_ms();
    }
    else {
        has_fuel_valve_trace = false;
    }

    // Try loading lox trace
    if (req.has_lox_trace_deg) {
        auto ret = throttle_lox_valve_trace_deg.load(req.lox_trace_deg);
        if (!ret.has_value()) {
            return std::unexpected(ret.error().context("failed to load lox valve trace"));
        }
        has_lox_valve_trace = true;
        trace_total_time_msec = throttle_lox_valve_trace_deg.get_total_time_ms();
    }
    else {
        has_lox_valve_trace = false;
    }

    // All traces must be the same length
    if (req.has_fuel_trace_deg && req.has_lox_trace_deg) {
        if (throttle_fuel_valve_trace_deg.get_total_time_ms() != throttle_lox_valve_trace_deg.get_total_time_ms()) {
            return std::unexpected(
                Error::from_cause(
                    "both traces must be the same length, fuel was %f while lox was %f",
                    static_cast<double>(throttle_fuel_valve_trace_deg.get_total_time_ms()),
                    static_cast<double>(throttle_lox_valve_trace_deg.get_total_time_ms())));
        }
    }

    current_state = SystemState_STATE_THROTTLE_VALVE_PRIMED;
    LOG_INF("Primed throttle valve sequence");

    return {};
}

/// Client-triggered transition from THROTTLE_VALVE_PRIMED to THROTTLE_VALVE
std::expected<void, Error> Controller::handle_start_throttle_valve_sequence(const StartThrottleValveSequenceRequest& req)
{
    MutexGuard guard{&controller_state_lock};

    if (current_state != SystemState_STATE_THROTTLE_VALVE_PRIMED) {
        return std::unexpected(Error::from_cause("State must be THROTTLE_VALVE_PRIMED to enter THROTTLE_VALVE"));
    }

    trace_start_cycle = k_cycle_get_64();
    current_state = SystemState_STATE_THROTTLE_VALVE;

    LOG_INF("Starting throttle valve sequence");

    return {};
}

/// Client-triggered transition from IDLE to THROTTLE_PRIMED
std::expected<void, Error> Controller::handle_load_throttle_sequence(const LoadThrottleSequenceRequest& req)
{
    MutexGuard guard{&controller_state_lock};

    if (current_state != SystemState_STATE_IDLE) {
        return std::unexpected(Error::from_cause("throttle thrust sequence load rejected unless system is idle"));
    }

    // Try loading thrust trace
    auto ret = throttle_thrust_trace_N.load(req.thrust_trace_N);
    if (!ret.has_value()) {
        return std::unexpected(ret.error().context("failed to load thrust trace"));
    }
    trace_total_time_msec = throttle_thrust_trace_N.get_total_time_ms();

    current_state = SystemState_STATE_THROTTLE_PRIMED;
    LOG_INF("Primed throttle thrust sequence");

    return {};
}

/// Client-triggered transition from THROTTLE_PRIMED to THROTTLE
std::expected<void, Error> Controller::handle_start_throttle_sequence(const StartThrottleSequenceRequest& req)
{
    MutexGuard guard{&controller_state_lock};

    if (current_state != SystemState_STATE_THROTTLE_PRIMED) {
        return std::unexpected(Error::from_cause("State must be THROTTLE_PRIMED to enter THROTTLE"));
    }

    trace_start_cycle = k_cycle_get_64();
    current_state = SystemState_STATE_THROTTLE;

    LOG_INF("Starting throttle thrust sequence");

    return {};
}

// Client-triggered transition from IDLE to CALIRBATE_TVC
std::expected<void, Error> Controller::handle_calibrate_tvc(const CalibrateTvcRequest& req)
{
    MutexGuard guard{&controller_state_lock};

    if (current_state != SystemState_STATE_IDLE) {
        return std::unexpected(Error::from_cause("tvc load sequence rejected unless system is idle"));
    }
    // TODO

    current_state = SystemState_STATE_CALIBRATE_TVC;
    LOG_INF("Calibrating TVC");

    return {};
}

/// Client-triggered transition from IDLE to TVC_PRIMED
std::expected<void, Error> Controller::handle_load_tvc_sequence(const LoadTvcSequenceRequest& req)
{
    MutexGuard guard{&controller_state_lock};

    if (current_state != SystemState_STATE_IDLE) {
        return std::unexpected(Error::from_cause("tvc load sequence rejected unless system is idle"));
    }

    // Try loading pitch trace
    if (auto ret = tvc_pitch_trace_deg.load(req.pitch_trace_deg); !ret.has_value()) {
        return std::unexpected(ret.error().context("failed to load tvc pitch trace"));
    }

    // Try loading yaw trace
    if (auto ret = tvc_yaw_trace_deg.load(req.yaw_trace_deg); !ret.has_value()) {
        return std::unexpected(ret.error().context("failed to load tvc yaw trace"));
    }

    // Both traces must be the same length
    if (tvc_pitch_trace_deg.get_total_time_ms() != tvc_yaw_trace_deg.get_total_time_ms()) {
        return std::unexpected(
            Error::from_cause(
                "tvc traces must have the same length, pitch was %f while yaw was %f",
                static_cast<double>(tvc_pitch_trace_deg.get_total_time_ms()),
                static_cast<double>(tvc_yaw_trace_deg.get_total_time_ms())));
    }
    trace_total_time_msec = tvc_pitch_trace_deg.get_total_time_ms();

    current_state = SystemState_STATE_TVC_PRIMED;
    LOG_INF("Primed TVC sequence");

    return {};
}

/// Client-triggered transition from TVC_PRIMED to TVC
std::expected<void, Error> Controller::handle_start_tvc_sequence(const StartTvcSequenceRequest& req)
{
    MutexGuard guard{&controller_state_lock};

    if (current_state != SystemState_STATE_TVC_PRIMED) {
        return std::unexpected(Error::from_cause("State must be TVC_PRIMED to enter TVC"));
    }

    trace_start_cycle = k_cycle_get_64();
    current_state = SystemState_STATE_TVC;

    LOG_INF("Starting TVC sequence");

    return {};
}

/// Client-triggered transition from IDLE to RCS_VALVE_PRIMED
std::expected<void, Error> Controller::handle_load_rcs_valve_sequence(const LoadRcsValveSequenceRequest& req)
{
    MutexGuard guard{&controller_state_lock};

    if (current_state != SystemState_STATE_IDLE) {
        return std::unexpected(Error::from_cause("rcs valve sequence load rejected unless system is idle"));
    }

    // Try loading clockwise valve trace
    if (auto ret = rcs_cw_valve_trace.load(req.rcs_cw_valve_trace); !ret.has_value()) {
        return std::unexpected(ret.error().context("failed to load rcs cw valve trace"));
    }

    // Try loading coutner-clockwise valve trace
    if (auto ret = rcs_ccw_valve_trace.load(req.rcs_ccw_valve_trace); !ret.has_value()) {
        return std::unexpected(ret.error().context("failed to load rcs ccw valve trace"));
    }

    // Both traces must be the same length
    if (rcs_cw_valve_trace.get_total_time_ms() != rcs_ccw_valve_trace.get_total_time_ms()) {
        return std::unexpected(
            Error::from_cause(
                "rcs valve traces must have the same length, cw was %f while ccw was %f",
                static_cast<double>(rcs_cw_valve_trace.get_total_time_ms()),
                static_cast<double>(rcs_ccw_valve_trace.get_total_time_ms())));
    }
    trace_total_time_msec = rcs_cw_valve_trace.get_total_time_ms();

    current_state = SystemState_STATE_RCS_VALVE_PRIMED;
    LOG_INF("Primed RCS valve sequence");
    current_state = SystemState_STATE_RCS_VALVE_PRIMED;
    LOG_INF("Primed RCS valve sequence");

    return {};
}

/// Client-triggered transition from RCS_VALVE_PRIMED to RCS_VALVE
std::expected<void, Error> Controller::handle_start_rcs_valve_sequence(const StartRcsValveSequenceRequest& req)
{
    MutexGuard guard{&controller_state_lock};

    if (current_state != SystemState_STATE_RCS_VALVE_PRIMED) {
        return std::unexpected(Error::from_cause("State must be RCS_VALVE_PRIMED to enter RCS_VALVE"));
    }

    trace_start_cycle = k_cycle_get_64();
    current_state = SystemState_STATE_RCS_VALVE;

    LOG_INF("Starting RCS valve sequence");

    return {};
}

/// Client-triggered transition from IDLE to RCS_PRIMED
std::expected<void, Error> Controller::handle_load_rcs_sequence(const LoadRcsSequenceRequest& req)
{
    MutexGuard guard{&controller_state_lock};
    if (current_state != SystemState_STATE_IDLE) {
        return std::unexpected(Error::from_cause("rcs roll sequence load rejected unless system is idle"));
    }

    // Try loading roll trace
    if (auto ret = rcs_roll_trace_deg.load(req.trace_deg); !ret.has_value()) {
        return std::unexpected(ret.error().context("failed to load roll trace"));
    }
    trace_total_time_msec = rcs_roll_trace_deg.get_total_time_ms();

    current_state = SystemState_STATE_RCS_PRIMED;
    LOG_INF("Primed RCS roll sequence");

    return {};
}

/// Client-triggered transition from RCS_PRIMED to RCS
std::expected<void, Error> Controller::handle_start_rcs_sequence(const StartRcsSequenceRequest& req)
{
    MutexGuard guard{&controller_state_lock};

    if (current_state != SystemState_STATE_RCS_PRIMED) {
        return std::unexpected(Error::from_cause("State must be RCS_PRIMED to enter RCS"));
    }

    trace_start_cycle = k_cycle_get_64();
    current_state = SystemState_STATE_RCS;

    LOG_INF("Starting RCS roll sequence");

    return {};
}

/// Client-triggered transition from IDLE to STATIC_FIRE_PRIMED
std::expected<void, Error> Controller::handle_load_static_fire_sequence(const LoadStaticFireSequenceRequest& req)
{
    MutexGuard guard{&controller_state_lock};

    if (current_state != SystemState_STATE_IDLE) {
        return std::unexpected(Error::from_cause("flight load sequence rejected unless system is idle"));
    }

    // Try loading throttle trace
    if (auto ret = throttle_thrust_trace_N.load(req.thrust_trace_N); !ret.has_value()) {
        return std::unexpected(ret.error().context("failed to load static fire thrust trace"));
    }

    // Try loading tvc pitch trace
    if (auto ret = tvc_pitch_trace_deg.load(req.pitch_trace_deg); !ret.has_value()) {
        return std::unexpected(ret.error().context("failed to load static fire pitch trace"));
    }

    // Try loading tvc yaw trace
    if (auto ret = tvc_yaw_trace_deg.load(req.yaw_trace_deg); !ret.has_value()) {
        return std::unexpected(ret.error().context("failed to load static fire yaw trace"));
    }

    // All traces must be the same length
    if (!(throttle_thrust_trace_N.get_total_time_ms() == tvc_pitch_trace_deg.get_total_time_ms() &&
          tvc_pitch_trace_deg.get_total_time_ms() == tvc_yaw_trace_deg.get_total_time_ms())) {
        return std::unexpected(
            Error::from_cause(
                "static fire traces must have the same length, thrust was %f, pitch was %f, yaw was %f",
                static_cast<double>(throttle_thrust_trace_N.get_total_time_ms()),
                static_cast<double>(tvc_pitch_trace_deg.get_total_time_ms()),
                static_cast<double>(tvc_yaw_trace_deg.get_total_time_ms())));
    }
    trace_total_time_msec = flight_x_trace_m.get_total_time_ms();

    current_state = SystemState_STATE_STATIC_FIRE_PRIMED;
    LOG_INF("Primed static fire sequence");

    return {};
}

/// Client-triggered transition from STATIC_FIRE_PRIMED to STATIC_FIRE
std::expected<void, Error> Controller::handle_start_static_fire_sequence(const StartStaticFireSequenceRequest& req)
{
    MutexGuard guard{&controller_state_lock};

    if (current_state != SystemState_STATE_STATIC_FIRE_PRIMED) {
        return std::unexpected(Error::from_cause("State must be STATIC_FIRE_PRIMED to enter static fire"));
    }

    trace_start_cycle = k_cycle_get_64();
    current_state = SystemState_STATE_STATIC_FIRE;

    LOG_INF("Starting static fire sequence");

    return {};
}

/// Client-triggered transition from IDLE to FLIGHT_PRIMED
std::expected<void, Error> Controller::handle_load_flight_sequence(const LoadFlightSequenceRequest& req)
{
    MutexGuard guard{&controller_state_lock};

    if (current_state != SystemState_STATE_IDLE) {
        return std::unexpected(Error::from_cause("flight load sequence rejected unless system is idle"));
    }

    // Try loading x trace
    if (auto ret = flight_x_trace_m.load(req.x_position_trace_m); !ret.has_value()) {
        return std::unexpected(ret.error().context("failed to load flight x trace"));
    }

    // Try loading y trace
    if (auto ret = flight_y_trace_m.load(req.y_position_trace_m); !ret.has_value()) {
        return std::unexpected(ret.error().context("failed to load flight y trace"));
    }

    // Try loading z trace
    if (auto ret = flight_z_trace_m.load(req.z_position_trace_m); !ret.has_value()) {
        return std::unexpected(ret.error().context("failed to load flight z trace"));
    }

    // Try loading roll trace
    if (auto ret = flight_roll_trace_deg.load(req.roll_angle_trace_deg); !ret.has_value()) {
        return std::unexpected(ret.error().context("failed to load flight roll trace"));
    }

    // All traces must be the same length
    if (!(flight_x_trace_m.get_total_time_ms() == flight_y_trace_m.get_total_time_ms() &&
          flight_y_trace_m.get_total_time_ms() == flight_z_trace_m.get_total_time_ms() &&
          flight_z_trace_m.get_total_time_ms() == flight_roll_trace_deg.get_total_time_ms())) {
        return std::unexpected(
            Error::from_cause(
                "flight traces must have the same length, x was %f, y was %f, z was %f, roll was %f",
                static_cast<double>(flight_x_trace_m.get_total_time_ms()),
                static_cast<double>(flight_y_trace_m.get_total_time_ms()),
                static_cast<double>(flight_z_trace_m.get_total_time_ms()),
                static_cast<double>(flight_roll_trace_deg.get_total_time_ms())));
    }
    trace_total_time_msec = flight_x_trace_m.get_total_time_ms();

    current_state = SystemState_STATE_FLIGHT_PRIMED;
    LOG_INF("Primed flight sequence");

    return {};
}

/// Client-triggered transition from FLIGHT_PRIMED to FLIGHT
std::expected<void, Error> Controller::handle_start_flight_sequence(const StartFlightSequenceRequest& req)
{
    MutexGuard guard{&controller_state_lock};

    if (current_state != SystemState_STATE_FLIGHT_PRIMED) {
        return std::unexpected(Error::from_cause("State must be FLIGHT_PRIMED to enter FLIGHT"));
    }

    trace_start_cycle = k_cycle_get_64();
    FlightController::reset();
    current_state = SystemState_STATE_FLIGHT;

    LOG_INF("Starting flight sequence");

    return {};
}
