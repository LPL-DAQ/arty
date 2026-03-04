#include "StateThrustSeq.h"
#include "LookupTable.h"

#include <algorithm>
#include <cmath>
#include <zephyr/kernel.h>
#include <zephyr/logging/log.h>

LOG_MODULE_DECLARE(Controller, LOG_LEVEL_INF);

namespace {

// Physics constants
static constexpr float EFFICIENCY = 0.93f;
static constexpr float LBF_CONVERSION = 0.224809f;
static constexpr float K_SLOPE = -1.132744863732548e-04f;
static constexpr float K_OFFSET = 0.123605503801193f;
static constexpr float ALPHA = 307.6704337316606f;
static constexpr float LOX_AREA_SI = 1.39154e-5f;
static constexpr float PSI_TO_PA = 6894.76f;
static constexpr float FUEL_CV_INJ = 0.5f;
static constexpr float FUEL_SG = 0.806f;

// Track duration of low chamber pressure for abort logic.
static uint32_t low_ptc_start_time_ms = 0;

float calculate_fuel_mass_flow(float p_inj_fuel, float p_ch)
{
    float dP = std::max(0.0f, p_inj_fuel - p_ch);
    return 0.06309f * FUEL_CV_INJ * std::sqrt(dP * FUEL_SG);
}

float calculate_lox_mass_flow(float p_inj_lox, float p_ch)
{
    float dP_psi = std::max(0.0f, p_inj_lox - p_ch);
    float p_inj_safe = std::max(0.0f, p_inj_lox);
    float K_var = (K_SLOPE * p_inj_safe) + K_OFFSET;
    float rho_syn = 1141.0f + (ALPHA * p_inj_safe);
    return K_var * LOX_AREA_SI * std::sqrt(2.0f * rho_syn * dP_psi * PSI_TO_PA);
}

} // namespace

void StateThrustSeq::init()
{
    LOG_INF("Entering Closed Loop Throttle Mode");
    low_ptc_start_time_ms = 0;
}

std::pair<ControllerOutput, ThrustSequenceData> StateThrustSeq::tick(const AnalogSensors& sensors, float target_thrust_lbf, float target_of)
{
    ControllerOutput out{};
    ThrustSequenceData data{};

    uint32_t now_ms = k_uptime_get();

    // 1. Safety: abort if PTC401 <= 10 psi for > 500 ms.
    if (sensors.ptc401 <= 10.0f) {
        if (low_ptc_start_time_ms == 0) {
            low_ptc_start_time_ms = now_ms;
        } else if (now_ms - low_ptc_start_time_ms > 500U) {
            LOG_ERR("PTC401 below safe threshold for >500 ms in THRUST_SEQ, aborting.");
            out.set_fuel = true;
            out.fuel_pos = Controller::DEFAULT_FUEL_POS;
            out.set_lox = true;
            out.lox_pos = Controller::DEFAULT_LOX_POS;
            out.next_state = SystemState_STATE_ABORT;
            return {out, data};
        }
    } else {
        low_ptc_start_time_ms = 0;
    }

    // 2. Read pressures
    float p_ch = sensors.ptc401;
    float p_inj_fuel = sensors.pt203;
    float p_inj_lox = sensors.pt103;

    // 3. Calculate mass flows
    float mdot_f = calculate_fuel_mass_flow(p_inj_fuel, p_ch);
    float mdot_lox = calculate_lox_mass_flow(p_inj_lox, p_ch);

    // 4. Clamp fuel mass flow to avoid division by zero
    float mdot_f_safe = std::max(mdot_f, 0.001f);

    // 5. Calculate O/F
    float of_actual = mdot_lox / mdot_f_safe;

    // 6. Clamp O/F for lookup
    float of_safe = std::clamp(of_actual, 0.5f, 3.0f);

    // 7. Predict Isp using chamber pressure and O/F
    float predicted_isp = interp2D(isp_pc_axis, isp_pc_len,
                                   isp_of_axis, isp_of_len,
                                   isp_data,
                                   p_ch, of_safe);

    // 8. Predict thrust (convert to lbf-equivalent)
    float predicted_thrust = (mdot_f + mdot_lox) * predicted_isp * EFFICIENCY * LBF_CONVERSION;

    // Clamp requested O/F into safe range as well
    float target_of_safe = std::clamp(target_of, 0.5f, 3.0f);

    // 9. Interpolate requested valve positions from MPrime tables
    float fuel_valve_pos = interp2D(thrust_axis, thrust_axis_len,
                                    of_axis, of_axis_len,
                                    fuel_valve_grid,
                                    target_thrust_lbf, target_of_safe);

    float lox_valve_pos = interp2D(thrust_axis, thrust_axis_len,
                                   of_axis, of_axis_len,
                                   lox_valve_grid,
                                   target_thrust_lbf, target_of_safe);

    // Populate telemetry data
    data.predicted_thrust = predicted_thrust;
    data.predicted_of = of_actual;
    data.mdot_fuel = mdot_f;
    data.mdot_lox = mdot_lox;
    data.target_thrust = target_thrust_lbf;

    // 10. Populate ControllerOutput
    out.set_fuel = true;
    out.fuel_pos = fuel_valve_pos;
    out.set_lox = true;
    out.lox_pos = lox_valve_pos;
    out.next_state = SystemState_STATE_THRUST_SEQ;

    return {out, data};
}
