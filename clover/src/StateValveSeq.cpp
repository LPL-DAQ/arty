#include "StateValveSeq.h"
#include <zephyr/logging/log.h>

LOG_MODULE_REGISTER(StateValveSeq, LOG_LEVEL_INF);

static Trace fuel_trace;
static Trace lox_trace;
static bool has_fuel;
static bool has_lox;
static float fuel_total_time;
static float lox_total_time;

void StateValveSeq::init(bool has_fuel_trace, bool has_lox_trace, float fuel_total_time, float lox_total_time)
{
    // Timer is reset in handle_start_sequence before entering

    has_fuel = has_fuel_trace;
    has_lox = has_lox_trace;
    fuel_total_time = fuel_total_time;
    lox_total_time = lox_total_time;
}

std::pair<ControllerOutput, ValveSequenceData> StateValveSeq::tick(uint32_t current_time, uint32_t start_time)
{
    ControllerOutput out;
    ValveSequenceData data{};
    out.next_state = SystemState_STATE_VALVE_SEQ;  // Assume we stay in this state by default

    float dt = current_time - start_time;

    if (has_fuel) {
        auto f_target = fuel_trace.sample(dt);
        if (!f_target) {
            out.next_state = SystemState_STATE_IDLE;
            return {out, data};
        }
        out.set_fuel = true;
        out.fuel_pos = *f_target;
    }

    if (has_lox) {
        auto l_target = lox_trace.sample(dt);
        if (!l_target) {
            out.next_state = SystemState_STATE_IDLE;
            return {out, data};
        }
        out.set_lox = true;
        out.lox_pos = *l_target;
    }

    if (((has_fuel && has_lox) && (dt >= lox_total_time && dt >= fuel_total_time)) || ((has_fuel && !has_lox) && dt >= fuel_total_time) ||
        ((!has_fuel && has_lox) && dt >= lox_total_time)) {
        out.next_state = SystemState_STATE_IDLE;
        return {out, data};
    }

    return std::make_pair(out, data);
}

Trace& StateValveSeq::get_fuel_trace()
{
    return fuel_trace;
}

Trace& StateValveSeq::get_lox_trace()
{
    return lox_trace;
}
