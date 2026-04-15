#include "FlightController.h"
#include "MutexGuard.h"
#include "sensors/AnalogSensors.h"
#include "ControllerConfig.h"
#include "config.h"
#include "PID.h"
#include <zephyr/kernel.h>
#include <zephyr/kernel/thread_stack.h>
#include <zephyr/logging/log.h>

LOG_MODULE_REGISTER(FlightController, LOG_LEVEL_INF);

K_MUTEX_DEFINE(flight_controller_lock);

static FlightState get_flight_controller_state()
{
    MutexGuard guard{&flight_controller_lock};
    return FlightController::current_state;
}

static uint32_t get_flight_controller_sequence_start_time()
{
    MutexGuard guard{&flight_controller_lock};
    return FlightController::sequence_start_time;
}

static uint32_t get_flight_controller_abort_entry_time()
{
    MutexGuard guard{&flight_controller_lock};
    return FlightController::abort_entry_time;
}

// TODO: implement fr with varrying mass estimate
static float get_weight_lbf(){
    MutexGuard guard{&flight_controller_lock};
    return
}

namespace {
    Trace x_trace;
    Trace y_trace;
    Trace z_trace;
    Trace roll_trace;
    bool flight_sequence_has_trace = false;
    float flight_sequence_total_time = 0.0f;
    // TODO: Tune, including adding integral terms (is requried)
    PID pidYaw(0.385, 0.0, 0.44);   // about the X axis
    PID pidPitch(0.385, 0.0, 0.44);   // about the Y axis
    PID pidX(0, 0, 0);              // needs tuning
    PID pidY(0, 0, 0);              // needs tuning
    PID pidZ(0.075, 0.01, 0);
    PID pidZVelocity(0, 0, 0);      // needs tuning

    // Mounting offset between IMU sensor frame and rocket body frame (deg)
    constexpr float IMU_TO_BODY_YAW_DEG = 0.0f;
    constexpr float IMU_TO_BODY_PITCH_DEG = 0.0f;
    constexpr float IMU_TO_BODY_ROLL_DEG = 0.0f;

    const float maxGimble = 12.0;   // degrees, this is an estimate
    const float maxTiltDeg = 15.0;  // What we dont want the rocket to tilt more than (deg)
}

void resetPIDs()
{
    pidYaw.reset();
    pidPitch.reset();
    pidX.reset();
    pidY.reset();
    pidZ.reset();
    pidZVelocity.reset();
}

void FlightController::step_control_loop(DataPacket& data)
{
    int64_t current_time = k_uptime_get();

    FlightState local_state;
    uint32_t local_sequence_start_time;
    uint32_t local_abort_entry_time;
    {
        MutexGuard guard{&flight_controller_lock};
        local_state = current_state;
        local_sequence_start_time = sequence_start_time;
        local_abort_entry_time = abort_entry_time;
    }

    FlightStateOutput local_output{};

    switch (local_state) {
    case FlightState_FLIGHT_STATE_IDLE: {
        auto [idle_out, idle_data] = idle_tick();
        data.which_flight_state_data = DataPacket_flight_idle_data_tag;
        data.flight_state_data.flight_idle_data = idle_data;
        local_output = idle_out;
        break;
    }
    case FlightState_FLIGHT_STATE_TAKEOFF: {
        auto [takeoff_out, takeoff_data] = takeoff_tick(current_time, local_sequence_start_time);
        data.which_flight_state_data = DataPacket_flight_takeoff_data_tag;
        data.flight_state_data.flight_takeoff_data = takeoff_data;
        local_output = takeoff_out;
        break;
    }
    case FlightState_FLIGHT_STATE_FLIGHT_SEQ: {
        auto [seq_out, seq_data] = flight_seq_tick(data.analog_sensors, current_time, local_sequence_start_time);
        data.which_flight_state_data = DataPacket_flight_sequence_data_tag;
        data.flight_state_data.flight_sequence_data = seq_data;
        local_output = seq_out;
        break;
    }
    case FlightState_FLIGHT_STATE_LANDING: {
        auto [landing_out, landing_data] = landing_tick(current_time, local_sequence_start_time);
        data.which_flight_state_data = DataPacket_flight_landing_data_tag;
        data.flight_state_data.flight_landing_data = landing_data;
        local_output = landing_out;
        break;
    }
    case FlightState_FLIGHT_STATE_ABORT: {
        auto [abort_out, abort_data] = abort_tick(current_time, local_abort_entry_time);
        data.which_flight_state_data = DataPacket_flight_abort_data_tag;
        data.flight_state_data.flight_abort_data = abort_data;
        local_output = abort_out;
        break;
    }

    default: {
        auto [idle_out, idle_data] = idle_tick();
        data.which_flight_state_data = DataPacket_flight_idle_data_tag;
        data.flight_state_data.flight_idle_data = idle_data;
        local_output = idle_out;
        break;
    }
    }

    auto ret = change_state(local_output.next_state);
    if (!ret.has_value()) {
        LOG_ERR("Error while changing flight state: %s", ret.error().build_message().c_str());
    }

    data.flight_state_output = local_output;
    {
        MutexGuard guard{&flight_controller_lock};
        current_output = local_output;
        data.flight_state = current_state;
    }

}
// TODO: remove takeoff and landing, move them into system states. They can be filled in later
std::expected<void, Error> FlightController::change_state(FlightState new_state)
{
    MutexGuard guard{&flight_controller_lock};
    if (current_state == new_state)
        return {};

    if (new_state == FlightState_FLIGHT_STATE_TAKEOFF ||
        new_state == FlightState_FLIGHT_STATE_FLIGHT_SEQ ||
        new_state == FlightState_FLIGHT_STATE_LANDING) {
        sequence_start_time = k_uptime_get();
    }
    if (new_state == FlightState_FLIGHT_STATE_ABORT) {
        abort_entry_time = k_uptime_get();
    }

    current_state = new_state;
    LOG_INF("Changed Flight State to %s", get_state_name(current_state));
    return {};
}

std::pair<FlightStateOutput, FlightIdleData> FlightController::idle_tick()
{
    FlightStateOutput out{};
    FlightIdleData data{};
    out.next_state = FlightState_FLIGHT_STATE_IDLE;
    return {out, data};
}

std::pair<FlightStateOutput, FlightTakeoffData> FlightController::takeoff_tick(int64_t current_time, int64_t start_time)
{
    FlightStateOutput out{};
    FlightTakeoffData data{};

    if (current_time - start_time > 1000) {
        out.next_state = FlightState_FLIGHT_STATE_FLIGHT_SEQ;
    } else {
        out.next_state = FlightState_FLIGHT_STATE_TAKEOFF;
    }

    return {out, data};
}

std::pair<FlightStateOutput, FlightSequenceData> FlightController::flight_seq_tick(const AnalogSensorReadings& analog_sensors, int64_t current_time, int64_t start_time)
{
    FlightStateOutput out{};
    FlightSequenceData data{};

    float local_flight_sequence_total_time;
    bool local_flight_sequence_has_trace;
    {
        MutexGuard guard{&flight_controller_lock};
        local_flight_sequence_total_time = flight_sequence_total_time;
        local_flight_sequence_has_trace = flight_sequence_has_trace;
    }

    float elapsed_time = static_cast<float>(current_time - start_time);
    if (local_flight_sequence_total_time > 0.0f && elapsed_time > local_flight_sequence_total_time) {
        out.next_state = FlightState_FLIGHT_STATE_LANDING;
        return {out, data};
    }

    if (local_flight_sequence_has_trace) {
        auto x_target = x_trace.sample(elapsed_time);
        if (!x_target) {
            LOG_ERR("Flight sequence X trace sampling failed: %s", x_target.error().build_message().c_str());
            out.next_state = FlightState_FLIGHT_STATE_ABORT;
            return {out, data};
        }

        auto y_target = y_trace.sample(elapsed_time);
        if (!y_target) {
            LOG_ERR("Flight sequence Y trace sampling failed: %s", y_target.error().build_message().c_str());
            out.next_state = FlightState_FLIGHT_STATE_ABORT;
            return {out, data};
        }

        auto z_target = z_trace.sample(elapsed_time);
        if (!z_target) {
            LOG_ERR("Flight sequence Z trace sampling failed: %s", z_target.error().build_message().c_str());
            out.next_state = FlightState_FLIGHT_STATE_ABORT;
            return {out, data};
        }

        auto roll_target = roll_trace.sample(elapsed_time);
        if (!roll_target) {
            LOG_ERR("Flight sequence roll trace sampling failed: %s", roll_target.error().build_message().c_str());
            out.next_state = FlightState_FLIGHT_STATE_ABORT;
            return {out, data};
        }

        out.z_acceleration = *z_target;
        out.x_angular_acceleration = *x_target;
        out.y_angular_acceleration = *y_target;
        out.roll_position = *roll_target;
    }
    out.next_state = FlightState_FLIGHT_STATE_FLIGHT_SEQ;
    return {out, data};
}

std::pair<FlightStateOutput, FlightLandingData> FlightController::landing_tick(int64_t current_time, int64_t start_time)
{
    FlightStateOutput out{};
    FlightLandingData data{};

    if (current_time - start_time > 1000) {
        out.next_state = FlightState_FLIGHT_STATE_IDLE;
    } else {
        out.next_state = FlightState_FLIGHT_STATE_LANDING;
    }

    return {out, data};
}

std::pair<FlightStateOutput, FlightAbortData> FlightController::abort_tick(int64_t current_time, int64_t entry_time)
{
    FlightStateOutput out{};
    FlightAbortData data{};

    if (current_time - entry_time > 500) {
        out.next_state = FlightState_FLIGHT_STATE_IDLE;
    } else {
        out.next_state = FlightState_FLIGHT_STATE_ABORT;
    }

    return {out, data};
}


std::expected<void, Error> FlightController::init()
{
    LOG_INF("Initializing FlightController state");
    change_state(FlightState_FLIGHT_STATE_IDLE);

    // integral cant command more than 1/3rd output range
    pidYaw.setIntegralLimits(-maxGimble / 3, maxGimble / 3);
    pidPitch.setIntegralLimits(-maxGimble / 3, maxGimble / 3);
    pidYaw.setDerivativeLowPass(10.0);  // 10 Hz
    pidPitch.setDerivativeLowPass(10.0); // 10 Hz

    pidZ.setIntegralZone(0.05); // only use intelgral within 5 cm of target
    pidZ.setOutputLimits(-0.075, 0.075); // Max thrust variance from weight

    return {};
}

std::expected<void, Error> FlightController::load_sequence(const FlightLoadSequenceRequest& req)
{
    LOG_INF("Received load flight sequence request");

    if (req.y_position_trace.total_time_ms != req.x_position_trace.total_time_ms ||
        req.z_position_trace.total_time_ms != req.x_position_trace.total_time_ms ||
        req.roll_angle_trace.total_time_ms != req.x_position_trace.total_time_ms) {
        return std::unexpected(Error::from_cause("Flight sequence traces must have the same total_time_ms"));
    }

    auto result_x = x_trace.load(req.x_position_trace);
    if (!result_x)
        return std::unexpected(result_x.error().context("%s", "Invalid X position trace"));

    auto result_y = y_trace.load(req.y_position_trace);
    if (!result_y)
        return std::unexpected(result_y.error().context("%s", "Invalid Y position trace"));

    auto result_z = z_trace.load(req.z_position_trace);
    if (!result_z)
        return std::unexpected(result_z.error().context("%s", "Invalid Z position trace"));

    auto result_roll = roll_trace.load(req.roll_angle_trace);
    if (!result_roll)
        return std::unexpected(result_roll.error().context("%s", "Invalid roll angle trace"));

    {
        MutexGuard guard{&flight_controller_lock};
        flight_sequence_has_trace = true;
        flight_sequence_total_time = req.x_position_trace.total_time_ms;
    }

    return {};
}

std::expected<void, Error> FlightController::start_sequence()
{
    LOG_INF("Received start flight sequence request");
    {
        MutexGuard guard{&flight_controller_lock};
        sequence_start_time = k_uptime_get();
    }
    auto ret = change_state(FlightState_FLIGHT_STATE_TAKEOFF);
    if (!ret.has_value()) {
        return std::unexpected(ret.error().context("Failed to change state to takeoff"));
    }
    return {};
}

FlightState FlightController::state()
{
    MutexGuard guard{&flight_controller_lock};
    return current_state;
}

const char* FlightController::get_state_name(FlightState state)
{
    if (state == FlightState_FLIGHT_STATE_IDLE)
        return "Idle";
    if (state == FlightState_FLIGHT_STATE_ABORT)
        return "Abort";
    if (state == FlightState_FLIGHT_STATE_TAKEOFF)
        return "Takeoff";
    if (state == FlightState_FLIGHT_STATE_FLIGHT_SEQ)
        return "FlightSeq";
    if (state == FlightState_FLIGHT_STATE_LANDING)
        return "Landing";
    return "Unknown State";
}
