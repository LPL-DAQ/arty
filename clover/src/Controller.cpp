#include "Controller.h"
#include "sequencer.h"
#include "ThrottleValve.h" 
#include <zephyr/kernel.h>

struct control_iter_data {
    float time;
    float fuel_target;
    float lox_target;
};

// FIX: Define with 'C' linkage to match the extern "C" in sequencer.h
extern "C" { 
    K_MSGQ_DEFINE(control_data_msgq, sizeof(control_iter_data), 250, 4); 
}

static void step_control_loop(k_work* work) {
    if (step_count > count_to) {
        FuelValve::stop(); 
        LoxValve::stop();
        return;
    }

    float f = fuel_breakpoints[step_count];
    float l = lox_breakpoints[step_count];
    
    FuelValve::tick(f);
    LoxValve::tick(l);

    control_iter_data d = { 
        .time = static_cast<float>(k_uptime_get()) / 1000.0f, 
        .fuel_target = f, 
        .lox_target = l 
    };
    k_msgq_put(&control_data_msgq, &d, K_NO_WAIT);
}

K_WORK_DEFINE(control_loop, step_control_loop);

static void control_loop_schedule(k_timer* t) { 
    k_work_submit(&control_loop); 
    step_count++; 
}

K_TIMER_DEFINE(control_loop_timer, control_loop_schedule, nullptr);

std::expected<void, Error> Controller::handle_load_motor_sequence(const LoadMotorSequenceRequest& req) {
    sequencer_load_from_proto(req);
    return {};
}

std::expected<void, Error> Controller::handle_start_sequence(const StartSequenceRequest& req) {
    step_count = 0; 
    start_clock = k_cycle_get_64();
    k_timer_start(&control_loop_timer, K_MSEC(1), K_MSEC(1));
    return {};
}

std::expected<void, Error> Controller::handle_halt_sequence(const HaltSequenceRequest& req) {
    k_timer_stop(&control_loop_timer);
    FuelValve::stop(); 
    LoxValve::stop();
    return {};
}