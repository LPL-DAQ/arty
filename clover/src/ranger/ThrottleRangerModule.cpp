#include "ThrottleRangerModule.h"
#include "MutexGuard.h"
#include "../ControllerConfig.h"
#include "../sensors/AnalogSensors.h"
#include "../server.h"
#include "../config.h"
#include <zephyr/kernel.h>
#include <zephyr/kernel/thread_stack.h>
#include <zephyr/logging/log.h>
#include <utility>


LOG_MODULE_REGISTER(ThrottleRangerModule, LOG_LEVEL_INF);

K_MUTEX_DEFINE(throttle_ranger_module_lock);



std::expected<void, Error> ThrottleRangerModule::change_state(ThrottleState new_state)
{
    MutexGuard guard{&throttle_ranger_module_lock};
    if (current_state == new_state)
        return {};

    if (new_state == ThrottleState_THROTTLE_STATE_VALVE_SEQ) {
        sequence_start_time = k_uptime_get();
    } else if (new_state == ThrottleState_THROTTLE_STATE_THRUST_SEQ) {
        sequence_start_time = k_uptime_get();
    } else if (new_state == ThrottleState_THROTTLE_STATE_ABORT) {
        abort_entry_time = k_uptime_get();
    }

    current_state = new_state;

    LOG_INF("Changed State to %s", get_state_name(current_state));

    return {};
}


// std::optional<std::pair<AnalogSensorReadings, float>> analog_sensors_readings
void ThrottleRangerModule::step_control_loop(DataPacket& data)
{
    int64_t current_time = k_uptime_get();
    uint64_t start_cycle = k_cycle_get_64();
    ThrottleRangerStateOutput out{};

    ThrottleState local_state;
    uint32_t local_sequence_start_time;
    uint32_t local_abort_entry_time;

    {
        MutexGuard guard{&throttle_ranger_module_lock};
        local_state = current_state;
        local_sequence_start_time = sequence_start_time;
        local_abort_entry_time = abort_entry_time;

    }

    // --- PROCEDURAL LOGIC DISPATCHER ---
    switch (local_state) {
    case ThrottleState_THROTTLE_STATE_IDLE: {
        auto [idle_out, idle_data] = idle_tick();
        data.which_throttle_state_data = DataPacket_throttle_idle_data_tag;
        data.throttle_state_data.throttle_idle_data = idle_data;
        out = idle_out;
        break;
    }
    case ThrottleState_THROTTLE_STATE_CALIBRATE_VALVE: {
        // TODO: Pass through valve data
        auto [cal_out, cal_data] = calibrate_valve_tick(current_time, 0.0f,0.0f,0.0f,0.0f);
        data.which_throttle_state_data = DataPacket_throttle_valve_calibration_data_tag;
        data.throttle_state_data.throttle_valve_calibration_data = cal_data;
        out = cal_out;
        break;
    }
    case ThrottleState_THROTTLE_STATE_VALVE_PRIMED: {
        auto [primed_out, primed_data] = idle_tick();
        primed_out.next_state = ThrottleState_THROTTLE_STATE_VALVE_PRIMED;
        data.which_throttle_state_data = DataPacket_throttle_idle_data_tag;
        data.throttle_state_data.throttle_idle_data = primed_data;
        out = primed_out;
        break;
    }
    case ThrottleState_THROTTLE_STATE_VALVE_SEQ: {
        auto [seq_out, seq_data] = valve_sequence_tick(current_time);
        data.which_throttle_state_data = DataPacket_throttle_valve_sequence_data_tag;
        data.throttle_state_data.throttle_valve_sequence_data = seq_data;
        out = seq_out;
        break;
    }
    case ThrottleState_THROTTLE_STATE_THRUST_PRIMED: {
        auto [thrust_primed_out, thrust_primed_data] = idle_tick();
        thrust_primed_out.next_state = ThrottleState_THROTTLE_STATE_THRUST_PRIMED;
        data.which_throttle_state_data = DataPacket_throttle_idle_data_tag;
        data.throttle_state_data.throttle_idle_data = thrust_primed_data;
        out = thrust_primed_out;
        break;
    }
    case ThrottleState_THROTTLE_STATE_THRUST_SEQ: {
        auto [thrust_out, thrust_data] = thrust_sequence_tick(data.analog_sensors, current_time);
        data.which_throttle_state_data = DataPacket_throttle_ranger_thrust_sequence_data_tag;
        data.throttle_state_data.throttle_ranger_thrust_sequence_data = thrust_data;
        out = thrust_out;
        break;
    }
    case ThrottleState_THROTTLE_STATE_FLIGHT: {
        auto [flight_out, flight_data] = flight_tick(data.analog_sensors, data.flight_state_output);
        data.which_throttle_state_data = DataPacket_throttle_flight_data_tag;
        data.throttle_state_data.throttle_flight_data = flight_data;
        out = flight_out;
        break;
    }
    case ThrottleState_THROTTLE_STATE_ABORT: {
        auto [abort_out, abort_data] = abort_tick(current_time, local_abort_entry_time);
        data.which_throttle_state_data = DataPacket_throttle_abort_data_tag;
        data.throttle_state_data.throttle_abort_data = abort_data;
        out = abort_out;
        break;
    }
    default: {
        auto [idle_out, idle_data] = idle_tick();
        data.which_throttle_state_data = DataPacket_throttle_idle_data_tag;
        data.throttle_state_data.throttle_idle_data = idle_data;
        out = idle_out;
        break;
    }
    }

    data.which_throttle_state_output = DataPacket_throttle_ranger_state_output_tag;
    data.throttle_state_output.throttle_ranger_state_output = out;
    data.which_throttle_actuator_data = DataPacket_throttle_ranger_data_tag;

    change_state(out.next_state);

    data.throttle_state = state();
}


std::pair<ThrottleRangerStateOutput, ThrottleIdleData> ThrottleRangerModule::idle_tick()
{
    ThrottleRangerStateOutput out{};
    ThrottleIdleData data{};
    out.fuel_on = true;
    out.lox_on = true;
    out.next_state = ThrottleState_THROTTLE_STATE_IDLE;
    return {out, data};
}

std::pair<ThrottleRangerStateOutput, ThrottleFlightData> ThrottleRangerModule::flight_tick(const AnalogSensorReadings& analog_sensors, FlightStateOutput& flight_output)
{
    ThrottleRangerStateOutput out{};
    ThrottleFlightData data{};
    float target_thrust = flight_output.z_acceleration;
    out.fuel_on = true;
    out.lox_on = true;
    out.next_state = ThrottleState_THROTTLE_STATE_FLIGHT;

    return {out, data};
}


std::pair<ThrottleRangerStateOutput, ThrottleValveSequenceData> ThrottleRangerModule::valve_sequence_tick(int64_t current_time)
{
    ThrottleRangerStateOutput out{};
    ThrottleValveSequenceData data{};
    out.fuel_on = true;
    out.lox_on = true;
    bool local_valve_sequence_has_fuel;
    bool local_valve_sequence_has_lox;
    float local_valve_sequence_fuel_total_time_ms;
    float local_valve_sequence_lox_total_time_ms;

    uint32_t local_sequence_start_time;
    {
        MutexGuard guard{&throttle_ranger_module_lock};
        local_sequence_start_time = sequence_start_time;
        local_valve_sequence_has_fuel = valve_sequence_has_fuel;
        local_valve_sequence_has_lox = valve_sequence_has_lox;
        local_valve_sequence_fuel_total_time_ms = valve_sequence_fuel_total_time_ms;
        local_valve_sequence_lox_total_time_ms = valve_sequence_lox_total_time_ms;
    }

    out.next_state = ThrottleState_THROTTLE_STATE_VALVE_SEQ;  // Assume we stay in this state by default

    float dt = current_time - local_sequence_start_time;

    if (local_valve_sequence_has_fuel) {
        auto f_target = fuel_trace.sample(dt);
        if (!f_target) {
            LOG_ERR("Failed to sample fuel trace: %s", f_target.error().build_message().c_str());
            out.next_state = ThrottleState_THROTTLE_STATE_IDLE;
            return {out, data};
        }
        out.has_fuel_pos = true;
        out.fuel_pos = *f_target;
    }

    if (local_valve_sequence_has_lox) {
        auto l_target = lox_trace.sample(dt);
        if (!l_target) {
            LOG_ERR("Failed to sample lox trace: %s", l_target.error().build_message().c_str());
            out.next_state = ThrottleState_THROTTLE_STATE_IDLE;
            return {out, data};
        }
        out.has_lox_pos = true;
        out.lox_pos = *l_target;
    }

    bool done_fuel = local_valve_sequence_has_fuel ? dt >= local_valve_sequence_fuel_total_time_ms : true;
    bool done_lox = local_valve_sequence_has_lox ? dt >= local_valve_sequence_lox_total_time_ms : true;

    if (done_fuel && done_lox) {
        LOG_INF("Done open loop seq, dt was %f", static_cast<double>(dt));
        out.next_state = ThrottleState_THROTTLE_STATE_IDLE;
    }

    return std::make_pair(out, data);}

std::pair<ThrottleRangerStateOutput, ThrottleRangerThrustSequenceData> ThrottleRangerModule::thrust_sequence_tick(const AnalogSensorReadings& analog_sensors, int64_t current_time)
{
    ThrottleRangerStateOutput out{};
    ThrottleRangerThrustSequenceData data{};

    uint32_t local_sequence_start_time;
    float thrust_sequence_total_time_ms;
    {
        MutexGuard guard{&throttle_ranger_module_lock};
        local_sequence_start_time = sequence_start_time;
    }

    float elapsed_time = current_time - local_sequence_start_time;
    if (elapsed_time > thrust_sequence_total_time_ms) {
        out.next_state = ThrottleState_THROTTLE_STATE_IDLE;
        return {out, data};
    }

    auto target_result = throttle_thrust_trace.sample(elapsed_time);
    if (!target_result) {
        LOG_ERR("Failed to sample thrust_trace: %s", target_result.error().build_message().c_str());
        out.next_state = ThrottleState_THROTTLE_STATE_ABORT;
        return {out, data};
    }

    float target_thrust_lbf = *target_result;
    data.target_thrust = target_thrust_lbf;

    // TODO: Tripple check that the abort stuff works
    // 1. Safety: abort if PTC401 is below threshold for some time
    // TODO: is battery voltage the right thing to check here? that seems wrong
    if (analog_sensors.battery_voltage <= PTC401_ABORT_THRESHOLD) {
        if (low_ptc_start_time_ms == 0) {
            low_ptc_start_time_ms = current_time;
        }
        else if (current_time - low_ptc_start_time_ms > PTC401_ABORT_THRESHOLD_TIME_MS) {
            // TODO: Fix float to double cast (From noah: is this still an issue?)
            LOG_ERR("PTC401 < %f for >%u ms in THRUST_SEQ, aborting.", (double)PTC401_ABORT_THRESHOLD, PTC401_ABORT_THRESHOLD_TIME_MS);
            out.next_state = ThrottleState_THROTTLE_STATE_ABORT;
            return {};
        }
    }
    else {
        low_ptc_start_time_ms = 0;
    }

    // 2. Read pressures
    float ptc401_val = analog_sensors.battery_voltage;                                    // Adjusted value
    float pto401_val = analog_sensors.battery_voltage + LOX_ENGINE_INLET_LINE_LOSS_PSI;   // Adjusted value
    float pt103_val = analog_sensors.battery_voltage;                                      // Adjusted value
    float ptf401_val = analog_sensors.battery_voltage + FUEL_ENGINE_INLET_LINE_LOSS_PSI;  // Adjusted value
    float pt203_val = analog_sensors.battery_voltage;                                      // Adjusted value
    float ptc402_val = analog_sensors.battery_voltage;                                    // Adjusted value
    bool pt203_valid = (analog_sensors.battery_voltage >= MIN_threshold && analog_sensors.battery_voltage <= MAX_threshold_PT2k);
    bool ptf401_valid = (analog_sensors.battery_voltage >= MIN_threshold && analog_sensors.battery_voltage <= MAX_threshold_PT2k);
    bool pt103_valid = (analog_sensors.battery_voltage >= MIN_threshold && analog_sensors.battery_voltage <= MAX_threshold_PT2k);
    bool pto401_valid = (analog_sensors.battery_voltage >= MIN_threshold && analog_sensors.battery_voltage <= MAX_threshold_PT2k);
    bool ptc401_valid = (analog_sensors.battery_voltage >= MIN_threshold && analog_sensors.battery_voltage <= MAX_threshold_PT1k);
    bool ptc402_valid = (analog_sensors.battery_voltage >= MIN_threshold && analog_sensors.battery_voltage <= MAX_threshold_PT1k);
    float p_inj_fuel;
    float p_inj_lox;
    float p_ch;
    if (ptc401_valid && ptc402_valid) {
        // Both are healthy: Take the average
        p_ch = (ptc401_val + ptc402_val) / 2.0f;
    }
    else if (ptc401_valid) {
        // Only primary is healthy
        p_ch = ptc401_val;
    }
    else if (ptc402_valid) {
        // Only backup is healthy
        p_ch = ptc402_val;
    }
    else {
        LOG_ERR("NO PTC 401 / PTC402");

        // Both failed: Trigger Abort
        out.next_state = ThrottleState_THROTTLE_STATE_ABORT;
        return {};
    }
    if (pt203_valid && ptf401_valid) {
        // Both are healthy: Take the average
        p_inj_fuel = (pt203_val + ptf401_val) / 2.0f;
    }
    else if (pt203_valid) {
        // Only primary is healthy
        p_inj_fuel = pt203_val;
    }
    else if (ptf401_valid) {
        // Only backup is healthy
        p_inj_fuel = ptf401_val;
    }
    else {
        // Both failed: Trigger Abort
        LOG_ERR("NO PT 203 / ptf401");

        out.next_state = ThrottleState_THROTTLE_STATE_ABORT;
        return {};
    }
    if (pt103_valid && pto401_valid) {
        // Both are healthy: Take the average
        p_inj_lox = (pt103_val + pto401_val) / 2.0f;
    }
    else if (pt103_valid) {
        // Only primary is healthy
        p_inj_lox = pt103_val;
    }
    else if (pto401_valid) {
        // Only backup is healthy
        p_inj_lox = pto401_val;
    }
    else {
        // Both failed: Trigger Abort
        LOG_ERR("NO PT 203 / 401");

        out.next_state = ThrottleState_THROTTLE_STATE_ABORT;
        return {};
    }

    // 3. Calculate mass flows
    float mdot_f = calculate_fuel_mass_flow(p_inj_fuel, p_ch);
    float mdot_lox = calculate_lox_mass_flow(p_inj_lox, p_ch);

    // 4. Clamp fuel mass flow to avoid division by zero
    float mdot_f_safe = std::max(mdot_f, 0.001f);

    // 5. Calculate O/F
    float predicted_of = mdot_lox / mdot_f_safe;

    // 6. Clamp O/F for lookup
    float of_safe = std::clamp(predicted_of, MIN_SAFE_OF, MAX_SAFE_OF);

    // 7. Predict Isp using chamber pressure and O/F
    float predicted_isp = interp2D(isp_pc_axis_internal, 29, isp_of_axis_internal, 34, isp_data_internal, p_ch, of_safe);

    // 8. Predict thrust (convert to lbf-equivalent)
    float predicted_thrust = (mdot_f + mdot_lox) * predicted_isp * EFFICIENCY * LBF_CONVERSION;

    // Clamp requested O/F into safe range as well
    float target_of_safe = std::clamp(target_of, MIN_SAFE_OF, MAX_SAFE_OF);

    // 9. Compute PID
    float dt = SEC_PER_CONTROL_TICK;
    float thrust_error = target_thrust_lbf - predicted_thrust;
    float change_alpha_cmd = THRUST_KP * thrust_error;
    change_alpha_cmd *= dt;
    float clamped_change_alpha_cmd = std::clamp(change_alpha_cmd, MIN_CHANGE_ALPHA, MAX_CHANGE_ALPHA);

    // 10. Integrate PID to get alpha
    if (alpha == -1.0f) {
        // Initialize alpha to starting guess based on Mprime
        alpha = (target_thrust_lbf - thrust_axis_internal[0]) / (thrust_axis_internal[100 - 1] - thrust_axis_internal[0]);
    }
    alpha += clamped_change_alpha_cmd;
    alpha = std::clamp(alpha, MIN_ALPHA, MAX_ALPHA);

    // 11. Plug alpha into Mprime contour
    float thrust_from_alpha = alpha * (thrust_axis_internal[100 - 1] - thrust_axis_internal[0]) + thrust_axis_internal[0];
    float fuel_valve_cmd = interp2D(thrust_axis_internal, 100, of_axis_internal, 100, fuel_valve_grid_internal, thrust_from_alpha, target_of_safe);

    float lox_valve_cmd = interp2D(thrust_axis_internal, 100, of_axis_internal, 100, lox_valve_grid_internal, thrust_from_alpha, target_of_safe);

    // 12. Clamp valve commands to safe ranges
    fuel_valve_cmd = std::clamp(fuel_valve_cmd, MIN_VALVE_POS, MAX_VALVE_POS);
    lox_valve_cmd = std::clamp(lox_valve_cmd, MIN_VALVE_POS, MAX_VALVE_POS);

    // Populate telemetry datadata.has_predicted_thrust = true;
    data.predicted_thrust = predicted_thrust;
    data.predicted_of = predicted_of;
    data.mdot_fuel = mdot_f;
    data.mdot_lox = mdot_lox;
    data.change_alpha_cmd = change_alpha_cmd;
    data.clamped_change_alpha_cmd = clamped_change_alpha_cmd;
    data.alpha = alpha;
    data.thrust_from_alpha = thrust_from_alpha;


    // 10. Populate ThrottleControllerOutput
    out.has_fuel_pos = true;
    out.fuel_pos = fuel_valve_cmd;
    out.has_lox_pos = true;
    out.lox_pos = lox_valve_cmd;
    out.fuel_on  = true;
    out.lox_on = true;

    out.next_state = ThrottleState_THROTTLE_STATE_THRUST_SEQ;
    return {out, data};
}

std::pair<ThrottleRangerStateOutput, ThrottleAbortData> ThrottleRangerModule::abort_tick(uint32_t current_time, uint32_t entry_time)
{
    ThrottleRangerStateOutput out{};
    ThrottleAbortData data{};

    out.fuel_on = true;
    out.lox_on = true;
    out.has_fuel_pos = true;
    out.fuel_pos = DEFAULT_FUEL_POS;
    out.has_lox_pos = true;
    out.lox_pos = DEFAULT_LOX_POS;

    if (current_time - entry_time > 500) {
        out.next_state = ThrottleState_THROTTLE_STATE_IDLE;
    } else {
        out.next_state = ThrottleState_THROTTLE_STATE_ABORT;
    }

    return {out, data};
}

std::pair<ThrottleRangerStateOutput, ThrottleValveCalibrationData> ThrottleRangerModule::calibrate_valve_tick(uint32_t timestamp,float fuel_pos, float lox_pos,float fuel_pos_enc, float lox_pos_enc) {
    ThrottleRangerStateOutput out{};
    ThrottleValveCalibrationData data{};

    switch (cal_phase) {
        case CalPhase::SEEK_HARDSTOP:
            calibration_seek_hardstop(out, fuel_pos, fuel_pos_enc,
                            lox_pos, lox_pos_enc);
            break;
        case CalPhase::BACK_OFF:
            break;
        case CalPhase::END_MOVEMENT:
            calibration_end_movement(out, timestamp);
            break;
        case CalPhase::POWER_OFF:
            calibration_power_off(out, timestamp);
            break;
        case CalPhase::REPOWER:
            calibration_repower(out, timestamp);
            break;
        case CalPhase::COMPLETE:
            calibration_complete(out, timestamp);
            break;
        case CalPhase::MEASURE:
            calibration_measure(out, fuel_pos, fuel_pos_enc, lox_pos, lox_pos_enc);
            break;
        case CalPhase::ERROR:
            calibration_error(out, timestamp);
            break;
        default:
            break;
    }
    data.fuel_found_hardstop = cal_fuel_found_stop;
    data.fuel_hardstop_pos = cal_fuel_hardstop_position;
    data.lox_found_hardstop = cal_lox_found_stop;
    data.lox_hardstop_pos = cal_lox_hardstop_position;
    data.fuel_err = fuel_pos - (fuel_pos_enc+ cal_fuel_starting_error);
    data.lox_err = lox_pos - (lox_pos_enc + cal_lox_starting_error);
    data.cal_phase = calibration_get_phase_id();

    return std::make_pair(out, data);
}

void ThrottleRangerModule::start_calibration(float fuel_pos, float fuel_pos_enc, float lox_pos, float lox_pos_enc) {
    // Controller handles actuation now
    MutexGuard guard{&throttle_ranger_module_lock};
    cal_phase = CalPhase::SEEK_HARDSTOP;
    cal_rep_counter = 0;
    cal_fuel_found_stop = false;
    cal_lox_found_stop = false;
    cal_fuel_hardstop_position = 0.0f;
    cal_lox_hardstop_position = 0.0f;
    cal_power_cycle_timestamp = 0;

    cal_fuel_target_position = fuel_pos_enc;
    cal_lox_target_position = lox_pos_enc;

    cal_fuel_starting_error = fuel_pos - fuel_pos_enc;
    cal_lox_starting_error = lox_pos - lox_pos_enc;
}


std::expected<void, Error> ThrottleRangerModule::load_thrust_sequence(const ThrottleLoadThrustSequenceRequest& req)
{
    LOG_INF("Received load thrust sequence request");

    auto result = throttle_thrust_trace.load(req.thrust_trace_lbf);
    if (!result)
        return std::unexpected(result.error().context("%s", "Invalid thrust trace"));

    {
        MutexGuard guard{&throttle_ranger_module_lock};
        thrust_sequence_total_time_ms = req.thrust_trace_lbf.total_time_ms;
    }

    change_state(ThrottleState_THROTTLE_STATE_THRUST_PRIMED);


    return {};
}

std::expected<void, Error> ThrottleRangerModule::start_thrust_sequence()
{
    change_state(ThrottleState_THROTTLE_STATE_THRUST_SEQ);

    {
        MutexGuard guard{&throttle_ranger_module_lock};
        low_ptc_start_time_ms = 0;
        alpha = -1.0f;
    }

    return {};
}

std::expected<void, Error> ThrottleRangerModule::load_valve_sequence(const ThrottleLoadValveSequenceRequest& req)
{
    bool has_fuel = req.has_fuel_trace_deg;
    bool has_lox = req.has_lox_trace_deg;
    if (!has_fuel && !has_lox) {
        return std::unexpected(Error::from_cause("No sequences provided in load request"));
    }

    {
        MutexGuard guard{&throttle_ranger_module_lock};
        valve_sequence_has_fuel = has_fuel;
        valve_sequence_has_lox = has_lox;
        valve_sequence_fuel_total_time_ms = -1.0f;
        valve_sequence_lox_total_time_ms = -1.0f;
    }

    if (has_fuel) {
        auto result = fuel_trace.load(req.fuel_trace_deg);
        if (!result)
            return std::unexpected(result.error().context("%s", "Invalid fuel trace"));
        {
            MutexGuard guard{&throttle_ranger_module_lock};
            valve_sequence_fuel_total_time_ms = req.fuel_trace_deg.total_time_ms;
        }
    }

    if (has_lox) {
        auto result = lox_trace.load(req.lox_trace_deg);
        if (!result)
            return std::unexpected(result.error().context("%s", "Invalid lox trace"));
        {
            MutexGuard guard{&throttle_ranger_module_lock};
            valve_sequence_lox_total_time_ms = req.lox_trace_deg.total_time_ms;
        }
    }

    return {};
}

std::expected<void, Error> ThrottleRangerModule::start_valve_sequence()
{
    return change_state(ThrottleState_THROTTLE_STATE_VALVE_SEQ);
}

ThrottleState ThrottleRangerModule::state()
{
    MutexGuard guard{&throttle_ranger_module_lock};
    return current_state;
}

// TODO: currently completely dysfunctional idk who or why this was made like this
std::expected<void, Error> ThrottleRangerModule::power_on(const ThrottlePowerOnRequest& req)
{
    LOG_INF("Received power on valve request");

    MutexGuard guard{&throttle_ranger_module_lock};
    if (current_state != ThrottleState_THROTTLE_STATE_IDLE) {
        return std::unexpected(Error::from_cause("Cannot turn valve on unless system is IDLE"));
    }

    switch (req.valve) {
    case Valve_FUEL:
        LOG_INF("Turning fuel valve on");
        fuel_powered = true;
        break;
    case Valve_LOX:
        LOG_INF("Turning lox valve on");
        lox_powered = true;
        break;
    default:
        return std::unexpected(Error::from_cause("Unknown valve identifier provided to power on command"));
    }
    return {};
}
// TODO: also dysfunctional, see above
std::expected<void, Error> ThrottleRangerModule::power_off(const ThrottlePowerOffRequest& req)
{
    LOG_INF("Received power off valve request");

    MutexGuard guard{&throttle_ranger_module_lock};
    if (current_state != ThrottleState_THROTTLE_STATE_IDLE) {
        return std::unexpected(Error::from_cause("Cannot turn valve off unless system is IDLE"));
    }

    switch (req.valve) {
    case Valve_FUEL:
        fuel_powered = false;
        LOG_INF("Turning fuel valve off");
        break;
    case Valve_LOX:
        lox_powered = false;
        LOG_INF("Turning lox valve off");
        break;
    default:
        return std::unexpected(Error::from_cause("Unknown valve identifier provided to power off command"));
    }
    return {};
}


void ThrottleRangerModule::calibration_seek_hardstop(ThrottleRangerStateOutput& out, float fuel_pos,float fuel_pos_enc,float lox_pos, float lox_pos_enc) {
    out.fuel_on = true;
    out.lox_on = true;

    if (!cal_lox_found_stop
        && std::abs(lox_pos - (cal_lox_starting_error + lox_pos_enc)) <= cal_pos_error_limit
    ) {
            cal_lox_target_position += cal_step_size / (cal_rep_counter+1);
            out.has_lox_pos = true;
            out.lox_pos = cal_lox_target_position; // move towards stop, but slow down in later loops
    } else { // when it reaches
        cal_lox_found_stop = true;
        cal_lox_hardstop_position = lox_pos_enc;
        out.has_lox_pos = true;
        out.lox_pos = lox_pos_enc; // hold position once we find the hardstop
    }


    if (!cal_fuel_found_stop
        && std::abs(fuel_pos - (cal_fuel_starting_error + fuel_pos_enc)) <= cal_pos_error_limit
    ) {
            cal_fuel_target_position += cal_step_size / (cal_rep_counter+1);
            out.has_fuel_pos = true;
            out.fuel_pos = cal_fuel_target_position; // move towards stop, but slow down in later loops
    } else { // when it reaches
        cal_fuel_found_stop = true;
        cal_fuel_hardstop_position = fuel_pos_enc;
        out.has_fuel_pos = true;
        out.fuel_pos = fuel_pos_enc; // hold position once we find the hardstop
    }

    // if both reached, move away from stop
    if (cal_fuel_found_stop && cal_lox_found_stop) {
        cal_fuel_found_stop = false;
        cal_lox_found_stop = false;
        cal_rep_counter++;
        cal_phase = CalPhase::END_MOVEMENT;
    }
    out.next_state = ThrottleState_THROTTLE_STATE_CALIBRATE_VALVE;
}


void ThrottleRangerModule::calibration_end_movement(ThrottleRangerStateOutput& out, uint32_t timestamp) {
    out.has_reset_fuel_pos = true;
    out.reset_fuel_pos = cal_fuel_hardstop_position;
    out.has_reset_lox_pos = true;
    out.reset_lox_pos = cal_lox_hardstop_position;


    out.fuel_on = false;
    out.lox_on = false;
    if (cal_power_cycle_timestamp == 0){
        cal_power_cycle_timestamp = timestamp;
    }
    else if (timestamp - cal_power_cycle_timestamp >= 1000) {
        cal_phase = CalPhase::POWER_OFF;
    }
    out.next_state = ThrottleState_THROTTLE_STATE_CALIBRATE_VALVE;

}

void ThrottleRangerModule::calibration_power_off(ThrottleRangerStateOutput& out, uint32_t timestamp) {
    out.fuel_on = false;
    out.lox_on = false;
    if (timestamp - cal_power_cycle_timestamp >= 4000) {
        cal_phase = CalPhase::REPOWER;
    }
    out.next_state = ThrottleState_THROTTLE_STATE_CALIBRATE_VALVE;
}

void ThrottleRangerModule::calibration_repower(ThrottleRangerStateOutput& out, uint32_t timestamp) {
    out.fuel_on = true;
    out.lox_on = true;
    if (timestamp - cal_power_cycle_timestamp >= 5000) {
        cal_phase = CalPhase::COMPLETE;
    }
    out.next_state = ThrottleState_THROTTLE_STATE_CALIBRATE_VALVE;
}

void ThrottleRangerModule::calibration_complete(ThrottleRangerStateOutput& out, uint32_t timestamp) {
    out.fuel_on = true;
    out.lox_on = true;
    out.has_fuel_pos = true;
    out.fuel_pos = 95.0f;
    out.has_lox_pos = true;
    out.lox_pos = 95.0f;
    cal_fuel_found_stop = false;
    cal_lox_found_stop = false;
    cal_fuel_starting_error = 0;
    cal_lox_starting_error = 0;
    cal_fuel_target_position = 95;
    cal_lox_target_position = 95;

    // should be idle, but this is for testing
    out.next_state = ThrottleState_THROTTLE_STATE_CALIBRATE_VALVE;
    if (timestamp - cal_power_cycle_timestamp >= 6500) {
        out.has_reset_fuel_pos = true;
        out.reset_fuel_pos = 95.0f;
        out.has_reset_lox_pos = true;
        out.reset_lox_pos = 95.0f;
        out.next_state = ThrottleState_THROTTLE_STATE_IDLE;
    }

}

void ThrottleRangerModule::calibration_error(ThrottleRangerStateOutput& out, uint32_t timestamp) {
    // In error, turn off drivers and do not try to move
    out.fuel_on = false;
    out.lox_on = false;

    if (cal_power_cycle_timestamp == 0){
        cal_power_cycle_timestamp = timestamp;
    }
}

void ThrottleRangerModule::calibration_measure(ThrottleRangerStateOutput& out, float fuel_pos,float fuel_pos_enc,float lox_pos, float lox_pos_enc) {
    out.fuel_on = true;
    out.lox_on = true;

    if (lox_pos_enc > 10){
        cal_lox_target_position -= cal_step_size*3;
        out.has_lox_pos = true;
        out.lox_pos = cal_lox_target_position; // move towards stop, but slow down in later loops
    }
    else if (!cal_lox_found_stop
        && std::abs(lox_pos - (cal_lox_starting_error + lox_pos_enc)) <= cal_pos_error_limit
    ) {
        cal_lox_target_position -= cal_step_size;
        out.has_lox_pos = true;
        out.lox_pos = cal_lox_target_position; // move towards stop, but slow down in later loops
    } else { // when it reaches
        cal_lox_found_stop = true;
        cal_lox_hardstop_position = lox_pos_enc;
        out.has_lox_pos = true;
        out.lox_pos = lox_pos_enc; // hold position once we find the hardstop
    }

    cal_fuel_found_stop = true;
    cal_fuel_hardstop_position = fuel_pos_enc;
    out.has_fuel_pos = true;
    out.fuel_pos = fuel_pos_enc; // hold position once we find the hardstop

    // if both reached, move away from stop
    out.next_state = ThrottleState_THROTTLE_STATE_CALIBRATE_VALVE;

    if (cal_fuel_found_stop && cal_lox_found_stop) {
        cal_fuel_target_position = cal_fuel_hardstop_position;
        cal_lox_target_position = cal_lox_hardstop_position;
        // LOG_INF("err: %f, pos %f, enc %f",  std::abs(lox_pos - (cal_lox_starting_error + lox_pos_enc)), lox_pos, lox_pos_enc);
        // LOG_INF("Fuel hardstop at %f, Lox hardstop at %f", cal_fuel_hardstop_position, cal_lox_hardstop_position);
        out.next_state = ThrottleState_THROTTLE_STATE_IDLE;
    }
}

int ThrottleRangerModule::calibration_get_phase_id() {

    switch (cal_phase) {
        case CalPhase::SEEK_HARDSTOP:
            return 0;
        case CalPhase::BACK_OFF:
            return 1;
        case CalPhase::END_MOVEMENT:
            return 2;
        case CalPhase::POWER_OFF:
            return 3;
        case CalPhase::REPOWER:
            return 4;
        case CalPhase::COMPLETE:
            return 5;
        case CalPhase::MEASURE:
            return 6;
        case CalPhase::ERROR:
            return 7;
        default:
            return -1; // Unknown phase
    }
}



const char* ThrottleRangerModule::get_state_name(ThrottleState state)
{
    if (state == ThrottleState_THROTTLE_STATE_IDLE)
        return "Idle";
    if (state == ThrottleState_THROTTLE_STATE_CALIBRATE_VALVE)
        return "Calibrate Valve";
    if (state == ThrottleState_THROTTLE_STATE_VALVE_PRIMED)
        return "Valve Primed";
    if (state == ThrottleState_THROTTLE_STATE_VALVE_SEQ)
        return "Valve Seq";
    if (state == ThrottleState_THROTTLE_STATE_THRUST_PRIMED)
        return "Thrust Primed";
    if (state == ThrottleState_THROTTLE_STATE_THRUST_SEQ)
        return "Thrust Seq";
    if (state == ThrottleState_THROTTLE_STATE_FLIGHT)
        return "Flight";
    if (state == ThrottleState_THROTTLE_STATE_ABORT)
        return "Abort";
    return "Unknown State";  // Unknown state
}
