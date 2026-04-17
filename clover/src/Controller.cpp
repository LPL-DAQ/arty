#include "Controller.h"
#include "ControllerConfig.h"
#include "flight/FlightController.h"
#include "flight/StateEstimator.h"
#include "MutexGuard.h"
#include "server.h"
#include "config.h"

#include "sensors/AnalogSensors.h"
#include <zephyr/kernel.h>
#include <zephyr/kernel/thread_stack.h>
#include <zephyr/logging/log.h>

#if CONFIG_RANGER
#include "ranger/TVCRangerModule.h"
#include "ranger/TVCRangerActuator.h"
#include "ranger/RCSRangerModule.h"
#include "ranger/RCSRangerActuator.h"
#include "ranger/ThrottleRangerModule.h"
#include "ranger/ThrottleRangerActuator.h"
namespace ThrottleImpl = ThrottleRangerModule;
namespace TVCImpl = TVCRangerModule;
namespace RCSImpl = RCSRangerModule;

#elif CONFIG_HORNET
#include "hornet/TVCHornetModule.h"
#include "hornet/TVCHornetActuator.h"
#include "hornet/RCSHornetModule.h"
#include "hornet/RCSHornetActuator.h"
#include "hornet/ThrottleHornetModule.h"
#include "hornet/ThrottleHornetActuator.h"
#include "PwmActuator.h"
namespace ThrottleImpl = ThrottleHornetModule;
namespace TVCImpl = TVCHornetModule;
namespace RCSImpl = RCSHornetModule;
#else
    #error "No configuration defined. Please define CONFIG_RANGER or CONFIG_HORNET in your build configuration."
#endif


LOG_MODULE_REGISTER(Controller, LOG_LEVEL_INF);

K_MSGQ_DEFINE(telemetry_msgq, sizeof(DataPacket), 100, 1);

// Controller tick workqueue thread
K_THREAD_STACK_DEFINE(controller_step_thread_stack, 4096);
k_work_q controller_step_work_q;

/// Count of how many packets had to be dropped, used for logging. Owned by controller tick workqueue thread.
static int recent_packets_attempted = 0;
static int recent_packets_dropped = 0;

std::expected<void, Error> Controller::change_state(SystemState new_state)
{
    MutexGuard guard{&controller_state_lock};
    return Controller::current_state;
}

static std::expected<void, Error> change_state(SystemState new_state)
{
    if (Controller::current_state == new_state) {
        return {};
    }

    switch (new_state) {
    case SystemState_STATE_IDLE:
    case SystemState_STATE_ABORT:
        break;

    case SystemState_STATE_THROTTLE_PRIMED:
    case SystemState_STATE_TVC_PRIMED:
    case SystemState_STATE_RCS_PRIMED:
    case SystemState_STATE_FLIGHT_PRIMED:
    case SystemState_STATE_STATIC_FIRE_PRIMED:
        if (Controller::current_state != SystemState_STATE_IDLE) {
            return std::unexpected(Error::from_cause("%s request rejected unless system is idle", Controller::get_state_name(new_state)));
        }
        break;

    case SystemState_STATE_THROTTLE:
        if (Controller::current_state != SystemState_STATE_THROTTLE_PRIMED) {
            return std::unexpected(Error::from_cause("System state change to Throttle only allowed from Throttle Primed"));
        }
        break;

    case SystemState_STATE_TVC:
        if (Controller::current_state != SystemState_STATE_TVC_PRIMED) {
            return std::unexpected(Error::from_cause("System state change to TVC only allowed from TVC Primed"));
        }
        break;

    case SystemState_STATE_STATIC_FIRE:
        if (Controller::current_state != SystemState_STATE_STATIC_FIRE_PRIMED) {
            return std::unexpected(Error::from_cause("System state change to Static Fire only allowed from Static Fire Primed"));
        }
        break;

    case SystemState_STATE_RCS:
        if (Controller::current_state != SystemState_STATE_RCS_PRIMED) {
            return std::unexpected(Error::from_cause("System state change to RCS only allowed from RCS Primed"));
        }
        break;

    case SystemState_STATE_FLIGHT:
        if (Controller::current_state != SystemState_STATE_FLIGHT_PRIMED) {
            return std::unexpected(Error::from_cause("System state change to Flight only allowed from Flight Primed"));
        }
        break;

    default:
        return std::unexpected(Error::from_cause("System state change to unsupported state: %d", new_state));
    }

    Controller::current_state = new_state;
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
    StateEstimator::init();
    // Other sensors here...
    k_sched_unlock();

    // init actuators
#if CONFIG_RANGER
    LOG_INF("Initializing fuel throttle valve");
    if (auto result = FuelValve::init(); !result) {
        LOG_ERR("Failed to initialize fuel throttle valve: %s", result.error().build_message().c_str());
        return std::unexpected(result.error());
    }

    LOG_INF("Initializing lox throttle valve");
    if (auto result = LoxValve::init(); !result) {
        LOG_ERR("Failed to initialize lox throttle valve: %s", result.error().build_message().c_str());
        return std::unexpected(result.error());
    }
#elif CONFIG_HORNET
    LOG_INF("Initializing PWM Actuators");
    if (auto result = MotorBeta1::init(); !result) {
        LOG_ERR("Failed to initialize motor beta 1: %s", result.error().build_message().c_str());
        return std::unexpected(result.error());
    }
    if (auto result = MotorBeta2::init(); !result) {
        LOG_ERR("Failed to initialize motor beta 2: %s", result.error().build_message().c_str());
        return std::unexpected(result.error());
    }
    if (auto result = MotorBeta3::init(); !result) {
        LOG_ERR("Failed to initialize motor beta 3: %s", result.error().build_message().c_str());
        return std::unexpected(result.error());
    }
    if (auto result = MotorBeta4::init(); !result) {
        LOG_ERR("Failed to initialize motor beta 4: %s", result.error().build_message().c_str());
        return std::unexpected(result.error());
    }
    if (auto result = ServoL::init(); !result) {
        LOG_ERR("Failed to initialize motor servo x: %s", result.error().build_message().c_str());
        return std::unexpected(result.error());
    }
    if (auto result = ServoR::init(); !result) {
        LOG_ERR("Failed to initialize motor servo y: %s", result.error().build_message().c_str());
        return std::unexpected(result.error());
    }
    if (auto result = MotorCR::init(); !result) {
        LOG_ERR("Failed to initialize motor CR 1: %s", result.error().build_message().c_str());
        return std::unexpected(result.error());
    }
    // if (auto result = MotorBeta1::init(); !result) {
    //     LOG_ERR("Failed to initialize motor beta 1: %s", result.error().build_message().c_str());
    //     return std::unexpected(result.error());
    // }

#endif

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

void Controller::step_control_loop(k_work*)
{
    uint64_t start_cycle = k_cycle_get_64();
    DataPacket data = DataPacket_init_default;

    // Read sensors
    auto analog_sensors_readings = AnalogSensors::read();
    if (analog_sensors_readings) {
        std::tie(data.analog_sensors, data.controller_timing.analog_sensors_sense_time_ns) = *analog_sensors_readings;
    }
    else {
        // TODO: uncomment this
        // LOG_WRN("Analog sensor data is not yet ready, leaving defaults.");
    }

    // TODO: make this exist with real sensors
    GNCSensorReadings gnc_sensor_readings = GNCSensorReadings_init_default;
    // TODO: do the tie thing below but for each sensor. Maybe these aren't held together by one bit, and each sensor has its own thing?
    data.gnc_sensors = gnc_sensor_readings;


    daq_client_status daq_status = get_daq_client_status();

    // Dispatch to state estimation and peripheral controllers based on current state
    StateEstimator::step_control_loop(data);
    FlightController::step_control_loop(data);

    #if CONFIG_RANGER
        TVCRangerModule::step_control_loop(data);
        TVCRangerActuator::tick(data.tvc_state_output.tvc_ranger_state_output, data.tvc_actuator_data.tvc_ranger_data);

        RCSRangerModule::step_control_loop(data);
        RCSRangerActuator::tick(data.rcs_state_output.rcs_ranger_state_output, data.rcs_actuator_data.rcs_ranger_data);

        ThrottleRangerModule::step_control_loop(data);
        ThrottleRangerActuator::tick(data.throttle_state_output.throttle_ranger_state_output, data.throttle_actuator_data.throttle_ranger_data);
    #elif CONFIG_HORNET
        TVCHornetModule::step_control_loop(data);
        TVCHornetActuator::tick(data.tvc_state_output.tvc_hornet_state_output, data);

        RCSHornetModule::step_control_loop(data);
        RCSHornetActuator::tick(data.rcs_state_output.rcs_hornet_state_output, data.rcs_actuator_data.rcs_hornet_data);

        ThrottleHornetModule::step_control_loop(data);
        ThrottleHornetActuator::tick(data.throttle_state_output.throttle_hornet_state_output, data.throttle_actuator_data.throttle_hornet_data);
    #endif

    data.state = get_current_system_state();
    data.controller_timing.controller_tick_time_ns = static_cast<float>(k_cycle_get_64() - start_cycle) / sys_clock_hw_cycles_per_sec() * 1e9f;
    data.sequence_number = 0; // TODO: how is this supposed to be set?
    data.daq_last_pinged_ns = static_cast<float>(daq_status.last_pinged_ms) * 1e6f;

    // Log metrics on packet transmission success rates.
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
// TODO: Throttle valve seq vs throttle thrust seq (just check if it all works alr)
std::expected<void, Error> Controller::handle_abort(const AbortRequest& req)
{
    LOG_INF("Received abort request");
    MutexGuard guard{&controller_state_lock};
    Controller::abort_entry_time = k_uptime_get();
    auto ret = change_state(SystemState_STATE_ABORT);
    if (!ret.has_value()) {
        return std::unexpected(ret.error().context("Failed to change state to abort"));
    }
    return {};
}

std::expected<void, Error> Controller::handle_load_throttle_valve_sequence(const ThrottleLoadValveSequenceRequest& req)
{
    MutexGuard guard{&controller_state_lock};
    if (current_state != SystemState_STATE_IDLE) {
        return std::unexpected(Error::from_cause("Throttle valve sequence load rejected unless system is idle"));
    }

    if (ThrottleImpl::state() != ThrottleState_THROTTLE_STATE_IDLE) {
        return std::unexpected(Error::from_cause("Throttle must be idle before loading a valve sequence"));
    }
#if CONFIG_RANGER
    return ThrottleRangerModule::load_valve_sequence(req);
#else // CONFIG_HORNET
    return std::unexpected(Error::from_cause("Must be ranger config for valve sequences"));
#endif
}

std::expected<void, Error> Controller::handle_load_throttle_thrust_sequence(const ThrottleLoadThrustSequenceRequest& req)
{
    MutexGuard guard{&controller_state_lock};
    if (current_state != SystemState_STATE_IDLE) {
        return std::unexpected(Error::from_cause("Throttle thrust sequence load rejected unless system is idle"));
    }

    if (ThrottleImpl::state() != ThrottleState_THROTTLE_STATE_IDLE) {
        return std::unexpected(Error::from_cause("Throttle must be idle before loading a thrust sequence"));
    }

    return ThrottleImpl::load_thrust_sequence(req);
}

std::expected<void, Error> Controller::handle_load_rcs_valve_sequence(const RCSLoadValveSequenceRequest& req)
{
    MutexGuard guard{&controller_state_lock};
    if (current_state != SystemState_STATE_IDLE) {
        return std::unexpected(Error::from_cause("RCS valve sequence load rejected unless system is idle"));
    }

    if (RCSImpl::state() != RCSState_RCS_STATE_IDLE) {
        return std::unexpected(Error::from_cause("RCS must be idle before loading a valve or motor sequence"));
    }

#if CONFIG_RANGER
    return RCSRangerModule::load_motor_sequence(req);
#else // CONFIG_HORNET
    return RCSHornetModule::load_valve_sequence(req);
#endif
}

std::expected<void, Error> Controller::handle_load_rcs_roll_sequence(const RCSLoadRollSequenceRequest& req)
{
    MutexGuard guard{&controller_state_lock};
    if (current_state != SystemState_STATE_IDLE) {
        return std::unexpected(Error::from_cause("RCS roll sequence load rejected unless system is idle"));
    }

    if (RCSImpl::state() != RCSState_RCS_STATE_IDLE) {
        return std::unexpected(Error::from_cause("RCS must be idle before loading a roll sequence"));
    }

    return RCSImpl::load_roll_sequence(req);
}

std::expected<void, Error> Controller::handle_load_tvc_sequence(const TVCLoadSequenceRequest& req)
{
    MutexGuard guard{&controller_state_lock};
    if (current_state != SystemState_STATE_IDLE) {
        return std::unexpected(Error::from_cause("TVC load sequence rejected unless system is idle"));
    }

    if (TVCImpl::state() != TVCState_TVC_STATE_IDLE) {
        return std::unexpected(Error::from_cause("TVC must be idle before loading a sequence"));
    }

    return TVCImpl::load_sequence(req);
}

std::expected<void, Error> Controller::handle_load_flight_sequence(const FlightLoadSequenceRequest& req)
{
    MutexGuard guard{&controller_state_lock};
    if (current_state != SystemState_STATE_IDLE) {
        return std::unexpected(Error::from_cause("Flight load sequence rejected unless system is idle"));
    }
    if (FlightController::state() != FlightState_FLIGHT_STATE_IDLE) {
        return std::unexpected(Error::from_cause("Flight controller must be idle before loading a sequence"));
    }
    return FlightController::load_sequence(req);
}

std::expected<void, Error> Controller::handle_start_throttle_valve_sequence(const ThrottleStartValveSequenceRequest& req)
{
    MutexGuard guard{&controller_state_lock};
#if CONFIG_RANGER
    auto ret = ThrottleRangerModule::start_valve_sequence();
    if (!ret.has_value()) {
        return ret;
    }
    return change_state(SystemState_STATE_THROTTLE);
#else
    return std::unexpected(Error::from_cause("Throttle valve sequence start unsupported on Hornet"));
#endif
}

std::expected<void, Error> Controller::handle_start_throttle_thrust_sequence(const ThrottleStartThrustSequenceRequest& req)
{
    MutexGuard guard{&controller_state_lock};
    auto ret = ThrottleImpl::start_thrust_sequence();
    if (!ret.has_value()) {
        return ret;
    }

    return change_state(SystemState_STATE_THROTTLE);
}

std::expected<void, Error> Controller::handle_start_rcs_valve_sequence(const RCSStartValveSequenceRequest& req)
{
    MutexGuard guard{&controller_state_lock};
#if CONFIG_RANGER
    auto ret = RCSRangerModule::start_motor_sequence();
#elif CONFIG_HORNET
    auto ret = RCSHornetModule::start_valve_sequence();
#endif
    if (!ret.has_value()) {
        return ret;
    }

    return change_state(SystemState_STATE_RCS);
}

std::expected<void, Error> Controller::handle_start_rcs_roll_sequence(const RCSStartRollSequenceRequest& req)
{
    MutexGuard guard{&controller_state_lock};
    auto ret = RCSImpl::start_roll_sequence();
    if (!ret.has_value()) {
        return ret;
    }

    return change_state(SystemState_STATE_RCS);
}

std::expected<void, Error> Controller::handle_start_tvc_sequence(const TVCStartSequenceRequest& req)
{
    MutexGuard guard{&controller_state_lock};
    auto ret = TVCImpl::start_sequence();
    if (!ret.has_value()) {
        return ret;
    }

    return change_state(current_state == SystemState_STATE_STATIC_FIRE_PRIMED ? SystemState_STATE_STATIC_FIRE : SystemState_STATE_TVC);
}

std::expected<void, Error> Controller::handle_start_flight_sequence(const FlightStartSequenceRequest& req)
{
    MutexGuard guard{&controller_state_lock};
    auto ret = FlightController::start_sequence();
    if (!ret.has_value()) {
        return ret;
    }

    return change_state(SystemState_STATE_FLIGHT);
}

std::expected<void, Error> Controller::handle_prime(const PrimeRequest& req)
{
    MutexGuard guard{&controller_state_lock};
    switch (req.target) {
    case PrimeTarget_PRIME_TARGET_THROTTLE:
        if (ThrottleImpl::state() != ThrottleState_THROTTLE_STATE_THRUST_PRIMED) {
            return std::unexpected(Error::from_cause("Throttle must be primed before system prime"));
        }
        return change_state(SystemState_STATE_THROTTLE_PRIMED);

    case PrimeTarget_PRIME_TARGET_TVC:
        if (TVCImpl::state() != TVCState_TVC_STATE_TRACE_PRIMED) {
            return std::unexpected(Error::from_cause("TVC must be primed before system prime"));
        }
        return change_state(SystemState_STATE_TVC_PRIMED);

    case PrimeTarget_PRIME_TARGET_RCS:
        if (RCSImpl::state() != RCSState_RCS_STATE_ROLL_PRIMED) {
            return std::unexpected(Error::from_cause("RCS must be primed before system prime"));
        }
        return change_state(SystemState_STATE_RCS_PRIMED);

    case PrimeTarget_PRIME_TARGET_STATIC_FIRE:
        if (ThrottleImpl::state() != ThrottleState_THROTTLE_STATE_THRUST_PRIMED || TVCImpl::state() != TVCState_TVC_STATE_TRACE_PRIMED) {
            return std::unexpected(Error::from_cause("Static fire prime requires both Throttle and TVC to be primed"));
        }
        return change_state(SystemState_STATE_STATIC_FIRE_PRIMED);

    default:
        return std::unexpected(Error::from_cause("Unknown prime target: %d", req.target));
    }
}

std::expected<void, Error> Controller::handle_halt(const HaltRequest& req)
{
    MutexGuard guard{&controller_state_lock};
    auto ret = change_state(SystemState_STATE_IDLE);
    if (!ret.has_value()) {
        return std::unexpected(ret.error().context("Failed to change state to idle"));
    }

    HaltTarget target = req.has_target ? req.target : HaltTarget_HALT_TARGET_ALL;

    if (target == HaltTarget_HALT_TARGET_ALL) {
        FlightController::change_state(FlightState_FLIGHT_STATE_IDLE);
        ThrottleImpl::change_state(ThrottleState_THROTTLE_STATE_IDLE);
        TVCImpl::change_state(TVCState_TVC_STATE_IDLE);
        RCSImpl::change_state(RCSState_RCS_STATE_IDLE);

        return {};
    }

    switch (target) {
    case HaltTarget_HALT_TARGET_THROTTLE:
        ThrottleImpl::change_state(ThrottleState_THROTTLE_STATE_IDLE);
        return {};

    case HaltTarget_HALT_TARGET_TVC:
        TVCImpl::change_state(TVCState_TVC_STATE_IDLE);
        return {};

    case HaltTarget_HALT_TARGET_RCS:
        RCSImpl::change_state(RCSState_RCS_STATE_IDLE);
        return {};

    case HaltTarget_HALT_TARGET_FLIGHT: {
        FlightController::change_state(FlightState_FLIGHT_STATE_IDLE);
        return {};
    }
    default:
        return std::unexpected(Error::from_cause("Unknown halt target: %d", target));
    }
}

std::expected<void, Error> Controller::handle_calibrate_throttle(const ThrottleCalibrateValveRequest& req)
{
    MutexGuard guard{&controller_state_lock};
    if (current_state != SystemState_STATE_IDLE) {
        return std::unexpected(Error::from_cause("Throttle calibration rejected unless system is idle"));
    }

    return ThrottleImpl::change_state(ThrottleState_THROTTLE_STATE_CALIBRATE_VALVE);
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
    if (state == SystemState_STATE_FLIGHT_PRIMED)
        return "Flight_Primed";
    if (state == SystemState_STATE_THROTTLE_PRIMED)
        return "Throttle_Primed";
    if (state == SystemState_STATE_TVC_PRIMED)
        return "TVC_Primed";
    if (state == SystemState_STATE_RCS_PRIMED)
        return "RCS_Primed";
    if (state == SystemState_STATE_STATIC_FIRE_PRIMED)
        return "Static_Fire_Primed";
    if (state == SystemState_STATE_STATIC_FIRE)
        return "Static_Fire";
    return "Unknown State";  // Unknown state
}

