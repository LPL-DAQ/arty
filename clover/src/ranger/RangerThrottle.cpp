#include "RangerThrottle.h"
#include "Controller.h"
#include "LookupTable1D.h"
#include "MutexGuard.h"
// #include "../lut/thrust_to_fuel_1.5.h"
// #include "../lut/thrust_to_lox_1.5.h"
#include "../lut/cea_lut.h"
#include "../lut/thrust_to_fuel.h"
#include "../lut/thrust_to_lox.h"
#include <cmath>
#include <zephyr/kernel.h>
#include <zephyr/logging/log.h>

LOG_MODULE_REGISTER(RangerThrottle, CONFIG_LOG_DEFAULT_LEVEL);

K_MUTEX_DEFINE(ranger_throttle_lock);

static constexpr float DEFAULT_FUEL_POS = 81.0f;
static constexpr float DEFAULT_LOX_POS = 74.0f;

static constexpr float FUEL_ENGINE_INLET_LINE_LOSS_PSI = 21.0f;
static constexpr float LOX_ENGINE_INLET_LINE_LOSS_PSI = 41.0f;

// Physics constants
static constexpr float EFFICIENCY = 0.93f;
static constexpr float LBF_CONVERSION = 0.224809f;
static constexpr float K_SLOPE = -2.438665784714502e-04f;
static constexpr float K_OFFSET = 0.233994229898658f;
static constexpr float ALPHA = 1.183054065574921e+02f;
static constexpr float LOX_AREA_SI = 1.39154e-5f;
static constexpr float PSI_TO_PA = 6894.76f;
static constexpr float FUEL_CV_INJ = 0.5f;
static constexpr float FUEL_SG = 0.806f;
static constexpr float MIN_SAFE_OF = 0.5f;
static constexpr float MAX_SAFE_OF = 3.0f;
static constexpr float PTC401_ABORT_THRESHOLD = 10.0f;  //
static constexpr uint32_t PTC401_ABORT_THRESHOLD_TIME_MS = 500U;

// Controller constants (NOTE: Currently using OF = 1.4)
static inline float alpha = -1.0f;
// 0.008283 this is for OF 1.5
static constexpr float THRUST_KP = 0.0094f;
static constexpr float MAX_CHANGE_ALPHA_PER_SEC = 6.141f;
static constexpr float MAX_CHANGE_ALPHA_PER_TICK =
    MAX_CHANGE_ALPHA_PER_SEC * Controller::SEC_PER_CONTROL_TICK;
static constexpr float MIN_ALPHA = 0.0f;
static constexpr float MAX_ALPHA = 0.96f;
static constexpr float MIN_VALVE_POS = 25.0f;
static constexpr float MAX_VALVE_POS = 90.0f;
static constexpr float thrust_axis_internal[] = {
    400.0000f, 405.6122f, 411.2245f, 416.8367f, 422.4490f, 428.0612f, 433.6735f, 439.2857f, 444.8980f, 450.5102f, 456.1224f, 461.7347f, 
    467.3469f, 472.9592f, 478.5714f, 484.1837f, 489.7959f, 495.4082f, 501.0204f, 506.6327f, 512.2449f, 517.8571f, 523.4694f, 529.0816f, 
    534.6939f, 540.3061f, 545.9184f, 551.5306f, 557.1429f, 562.7551f, 568.3673f, 573.9796f, 579.5918f, 585.2041f, 590.8163f, 596.4286f, 
    602.0408f, 607.6531f, 613.2653f, 618.8776f, 624.4898f, 630.1020f, 635.7143f, 641.3265f, 646.9388f, 652.5510f, 658.1633f, 663.7755f, 
    669.3878f, 675.0000f
};

// Controller state variables
// TODO: James pls check
static constexpr float MAX_threshold_PT2k = 1900.0f;  // Define a maximum value for sensor validation
static constexpr float MAX_threshold_PT1k = 950.0f;   // Define a maximum value for sensor validation
static constexpr float MIN_threshold = 50.0f;         // Define a maximum value for sensor validation

enum class CalPhase {
    SEEK_HARDSTOP,
    BACK_OFF,
    END_MOVEMENT,
    POWER_OFF,
    REPOWER,
    COMPLETE,
    MEASURE,
    ERROR,
};

static inline CalPhase cal_phase = CalPhase::SEEK_HARDSTOP;

static inline float cal_fuel_target_position = 0.0f;
static inline float cal_fuel_hardstop_position = 0.0f;
static inline bool cal_fuel_found_stop = false;
static inline float cal_lox_target_position = 0.0f;
static inline float cal_lox_hardstop_position = 0.0f;
static inline bool cal_lox_found_stop = false;
static inline float cal_step_size = 0.002f;
static inline int cal_rep_counter = 0;
static inline float cal_pos_error_limit = 0.2f;
static inline float cal_fuel_starting_error = 0.0f;
static inline float cal_lox_starting_error = 0.0f;
static inline uint32_t cal_power_cycle_timestamp = 0;

struct CalibrationValveRefs {
    float& target_position;
    float& hardstop_position;
    bool& found_stop;
    float& starting_error;
};

static bool is_supported_valve(ThrottleValveType valve)
{
    return valve == ThrottleValveType_FUEL || valve == ThrottleValveType_LOX;
}

static CalibrationValveRefs get_valve_refs(ThrottleValveType valve)
{
    if (valve == ThrottleValveType_FUEL) {
        return CalibrationValveRefs{cal_fuel_target_position, cal_fuel_hardstop_position, cal_fuel_found_stop, cal_fuel_starting_error};
    }
    return CalibrationValveRefs{cal_lox_target_position, cal_lox_hardstop_position, cal_lox_found_stop, cal_lox_starting_error};
}

static void calibration_seek_hardstop(ThrottleValveCommand& command, float valve_pos, float valve_pos_enc, CalibrationValveRefs refs)
{
    command.enable = true;

    if (!refs.found_stop && std::abs(valve_pos - (refs.starting_error + valve_pos_enc)) <= cal_pos_error_limit) {
        refs.target_position += cal_step_size / (cal_rep_counter + 1);
        command.target_deg = refs.target_position;
    }
    else {
        refs.found_stop = true;
        refs.hardstop_position = valve_pos_enc;
        command.target_deg = valve_pos_enc;
    }

    if (refs.found_stop) {
        refs.found_stop = false;
        cal_rep_counter++;
        cal_phase = CalPhase::END_MOVEMENT;
    }
}

static void calibration_end_movement(ThrottleValveCommand& command, uint32_t timestamp)
{
    command.enable = false;
    if (cal_power_cycle_timestamp == 0) {
        cal_power_cycle_timestamp = timestamp;
    }
    else if (timestamp - cal_power_cycle_timestamp >= 1000) {
        cal_phase = CalPhase::POWER_OFF;
    }
}

static void calibration_power_off(ThrottleValveCommand& command, uint32_t timestamp)
{
    command.enable = false;
    if (timestamp - cal_power_cycle_timestamp >= 4000) {
        cal_phase = CalPhase::REPOWER;
    }
}

static void calibration_repower(ThrottleValveCommand& command, uint32_t timestamp)
{
    command.enable = true;
    if (timestamp - cal_power_cycle_timestamp >= 5000) {
        cal_phase = CalPhase::COMPLETE;
    }
}

static void calibration_complete(ThrottleValveCommand& command, uint32_t timestamp, CalibrationValveRefs refs)
{
    command.enable = true;
    command.target_deg = 95.0f;

    refs.found_stop = false;
    refs.starting_error = 0.0f;
    refs.target_position = 95.0f;

    if (timestamp - cal_power_cycle_timestamp >= 6500) {
        cal_phase = CalPhase::SEEK_HARDSTOP;
    }
}

static void calibration_error(ThrottleValveCommand& command, uint32_t timestamp)
{
    command.enable = false;

    if (cal_power_cycle_timestamp == 0) {
        cal_power_cycle_timestamp = timestamp;
    }
}

static void calibration_measure(ThrottleValveCommand& command, float valve_pos, float valve_pos_enc, CalibrationValveRefs refs)
{
    command.enable = true;

    if (valve_pos_enc > 10.0f) {
        refs.target_position -= cal_step_size * 3;
        command.target_deg = refs.target_position;
    }
    else if (!refs.found_stop && std::abs(valve_pos - (refs.starting_error + valve_pos_enc)) <= cal_pos_error_limit) {
        refs.target_position -= cal_step_size;
        command.target_deg = refs.target_position;
    }
    else {
        refs.found_stop = true;
        refs.hardstop_position = valve_pos_enc;
        command.target_deg = valve_pos_enc;
    }

    if (refs.found_stop) {
        refs.target_position = refs.hardstop_position;
        cal_phase = CalPhase::SEEK_HARDSTOP;
    }
}

// Track duration of low chamber pressure for abort logic.
static inline float calculate_fuel_mass_flow(float p_inj_fuel, float p_ch)
{
    float dP = std::max(0.0f, p_inj_fuel - p_ch);
    return 0.06309f * FUEL_CV_INJ * std::sqrt(dP * FUEL_SG);
}

static inline float calculate_lox_mass_flow(float p_inj_lox, float p_ch)
{
    float dP_psi = std::max(0.0f, p_inj_lox - p_ch);
    float dP_Pa = dP_psi * PSI_TO_PA;
    float p_inj_safe = std::max(0.0f, p_inj_lox);
    float K_var = (K_SLOPE * p_inj_safe) + K_OFFSET;
    float rho_syn = 1141.0f + (ALPHA * p_inj_safe);
    return K_var * LOX_AREA_SI * std::sqrt(2.0f * rho_syn * dP_Pa);
}

/// Reset internal state before an active control trace
void RangerThrottle::reset()
{
    MutexGuard ranger_throttle_guard{&ranger_throttle_lock};
    alpha = -1.0f;
}

// TODO: test the re-done single valve calibration. Also, make it not super slow
std::expected<ThrottleValveCommand, Error> RangerThrottle::calibration_tick(ThrottleValveType valve, uint32_t timestamp, float valve_pos, float valve_pos_enc)
{
    MutexGuard ranger_throttle_guard{&ranger_throttle_lock};

    if (!is_supported_valve(valve)) {
        return std::unexpected(Error::from_cause("unknown valve passed to calibration_tick"));
    }

    auto refs = get_valve_refs(valve);
    ThrottleValveCommand command = ThrottleValveCommand_init_default;
    command.enable = true;

    switch (cal_phase) {
    case CalPhase::SEEK_HARDSTOP:
        calibration_seek_hardstop(command, valve_pos, valve_pos_enc, refs);
        break;
    case CalPhase::BACK_OFF:
        break;
    case CalPhase::END_MOVEMENT:
        calibration_end_movement(command, timestamp);
        break;
    case CalPhase::POWER_OFF:
        calibration_power_off(command, timestamp);
        break;
    case CalPhase::REPOWER:
        calibration_repower(command, timestamp);
        break;
    case CalPhase::COMPLETE:
        calibration_complete(command, timestamp, refs);
        break;
    case CalPhase::MEASURE:
        calibration_measure(command, valve_pos, valve_pos_enc, refs);
        break;
    case CalPhase::ERROR:
        calibration_error(command, timestamp);
        break;
    default:
        break;
    }

    return command;
}

void RangerThrottle::calibration_reset(ThrottleValveType valve, float valve_pos, float valve_pos_enc)
{
    MutexGuard ranger_throttle_guard{&ranger_throttle_lock};

    if (!is_supported_valve(valve)) {
        return;
    }

    auto refs = get_valve_refs(valve);

    cal_phase = CalPhase::SEEK_HARDSTOP;
    cal_rep_counter = 0;
    refs.found_stop = false;
    refs.hardstop_position = 0.0f;
    cal_power_cycle_timestamp = 0;

    refs.target_position = valve_pos_enc;

    refs.starting_error = valve_pos - valve_pos_enc;
}

static std::expected<float, Error> thrust_predictor(AnalogSensorReadings& analog_sensors, RangerThrottleMetrics& metrics)
{

    // 2. Read pressures
    // TODO: these were also battery voltage? That seems wrong
    // TODO: ARRE THESE THE CORRECT PTS? NEEDS TO BE UPDATED FOR RANGER
    // TODO: james, rupin says he doesnt know what to do - He cant find the P&ID
    float ptc401_val = analog_sensors.ptc401;
    float ptc402_val = analog_sensors.ptc402;
    float pto401_val = analog_sensors.pto401 + LOX_ENGINE_INLET_LINE_LOSS_PSI;
    float pt103_val = analog_sensors.pt103;
    float ptf401_val = analog_sensors.ptf401 + FUEL_ENGINE_INLET_LINE_LOSS_PSI;  // Adjusted value
    float pt203_val = analog_sensors.pt203;

    bool ptc401_valid = (analog_sensors.ptc401 >= MIN_threshold && analog_sensors.ptc401 <= MAX_threshold_PT2k) && analog_sensors.has_ptc401;
    bool ptc402_valid = (analog_sensors.ptc402 >= MIN_threshold && analog_sensors.ptc402 <= MAX_threshold_PT2k) && analog_sensors.has_ptc402;
    bool pto401_valid = (analog_sensors.pto401 >= MIN_threshold && analog_sensors.pto401 <= MAX_threshold_PT2k) && analog_sensors.has_pto401;
    bool pt103_valid = (analog_sensors.pt103 >= MIN_threshold && analog_sensors.pt103 <= MAX_threshold_PT2k) && analog_sensors.has_pt103;
    bool ptf401_valid = (analog_sensors.ptf401 >= MIN_threshold && analog_sensors.ptf401 <= MAX_threshold_PT2k) && analog_sensors.has_ptf401;
    bool pt203_valid = (analog_sensors.pt203 >= MIN_threshold && analog_sensors.pt203 <= MAX_threshold_PT2k) && analog_sensors.has_pt203;

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
        // return std::unexpected(
        //     Error::from_cause("missing PTC-401 / PTC-402 -- PTC-401 has %f, PTC-402 has %f", static_cast<double>(ptc401_val),
        //     static_cast<double>(ptc402_val)));
    }

    float p_inj_fuel;
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
        // return std::unexpected(
        //     Error::from_cause("missing PT-203 / PTF-401 -- PT-203 has %f, PTF-401 has %f", static_cast<double>(pt203_val), static_cast<double>(ptf401_val)));
    }

    float p_inj_lox;
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
        // return std::unexpected(
        //     Error::from_cause("missing PT-103 / PTO-401 -- PT-103 has %f, PTO-401 has %f", static_cast<double>(pt103_val), static_cast<double>(pto401_val)));
    }

    // 3. Calculate mass flows
    float mdot_f = calculate_fuel_mass_flow(p_inj_fuel, p_ch);
    float mdot_lox = calculate_lox_mass_flow(p_inj_lox, p_ch);

    // 4. Clamp fuel mass flow to avoid division by zero
    float mdot_f_safe = std::max(mdot_f, 0.001f);

    // 5. Calculate O/F
    float predicted_of = mdot_lox / mdot_f_safe;

    // 6. Clamp O/F for lookup
    [[maybe_unused]] float of_safe = std::clamp(predicted_of, MIN_SAFE_OF, MAX_SAFE_OF);

    // TODO: add lut for cea
    // 7. Predict Isp using chamber pressure and O/F
    float predicted_isp = PcOfCea::sample(p_ch, predicted_of);

    // 8. Predict thrust (convert to lbf-equivalent)
    float predicted_thrust_lbf = (mdot_f + mdot_lox) * predicted_isp * EFFICIENCY * LBF_CONVERSION;

    metrics.predicted_thrust_lbf = 0;
    metrics.predicted_thrust_lbf = predicted_thrust_lbf;

    metrics.predicted_of = predicted_of;
    metrics.mdot_fuel = mdot_f;
    metrics.mdot_lox = mdot_lox;

    return predicted_thrust_lbf;
    // return 0.0f;
}

static std::tuple<ThrottleValveCommand, ThrottleValveCommand>
active_control(float& alpha_state, float predicted_thrust_lbf, float thrust_command_lbf, RangerThrottleMetrics& metrics)
{
    float dt = Controller::SEC_PER_CONTROL_TICK;
    float thrust_error = thrust_command_lbf - predicted_thrust_lbf;
    float change_alpha_cmd = THRUST_KP * thrust_error;
    change_alpha_cmd *= dt;
    float clamped_change_alpha_cmd = std::clamp(change_alpha_cmd, -MAX_CHANGE_ALPHA_PER_TICK, MAX_CHANGE_ALPHA_PER_TICK);

    // 10. Integrate PID to get alpha
    if (alpha_state == -1.0f) {
        // Initialize alpha to starting guess based on Mprime
        alpha_state = 0.5f;
        // alpha_state = (thrust_command_lbf - thrust_axis_internal[0]) / (thrust_axis_internal[100 - 1] - thrust_axis_internal[0]);
    }
    alpha_state += clamped_change_alpha_cmd;
    alpha_state = std::clamp(alpha_state, MIN_ALPHA, MAX_ALPHA);

    // 11. Plug alpha into Mprime contour
    float thrust_from_alpha_lbf = alpha_state * (thrust_axis_internal[50 - 1] - thrust_axis_internal[0]) + thrust_axis_internal[0];
    float fuel_valve_command_deg = ThrustToFuelAxis::sample(thrust_from_alpha_lbf);
    float lox_valve_command_deg = ThrustToLoxAxis::sample(thrust_from_alpha_lbf);

    // 12. Clamp valve commands to safe ranges
    fuel_valve_command_deg = std::clamp(fuel_valve_command_deg, MIN_VALVE_POS, MAX_VALVE_POS);
    lox_valve_command_deg = std::clamp(lox_valve_command_deg, MIN_VALVE_POS, MAX_VALVE_POS);

    metrics.change_alpha_cmd = change_alpha_cmd;
    metrics.clamped_change_alpha_cmd = clamped_change_alpha_cmd;
    metrics.alpha = alpha_state;
    metrics.thrust_from_alpha_lbf = thrust_from_alpha_lbf;

    return {
        ThrottleValveCommand{.enable = true, .target_deg = fuel_valve_command_deg}, ThrottleValveCommand{.enable = true, .target_deg = lox_valve_command_deg}};
}

/// Generate a comomand for the fuel and lox valve positions in degrees.
std::expected<std::tuple<ThrottleValveCommand, ThrottleValveCommand, RangerThrottleMetrics>, Error>
RangerThrottle::tick(AnalogSensorReadings& analog_sensors, float thrust_command_lbf)
{
    MutexGuard ranger_throttle_guard{&ranger_throttle_lock};
    RangerThrottleMetrics metrics = RangerThrottleMetrics_init_default;

    auto predicted_thrust = thrust_predictor(analog_sensors, metrics);
    if (!predicted_thrust) {
        LOG_INF("Thrust predictor failed");
        return std::unexpected(predicted_thrust.error());
    }

    auto [fuel_command, lox_command] = active_control(alpha, *predicted_thrust, thrust_command_lbf, metrics);

    return {{fuel_command, lox_command, metrics}};
}

#if CONFIG_TEST
std::tuple<ThrottleValveCommand, ThrottleValveCommand>
RangerThrottle::active_control_test(float& alpha_state, float predicted_thrust_lbf, float thrust_command_lbf, RangerThrottleMetrics& metrics)
{
    return ::active_control(alpha_state, predicted_thrust_lbf, thrust_command_lbf, metrics);
}
#endif
