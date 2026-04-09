#ifndef APP_STATE_CALIBRATE_VALVE_H
#define APP_STATE_CALIBRATE_VALVE_H

#include "ThrottleController.h"
#include "ThrottleValve.h"

#include <cmath>

namespace StateCalibrateValve {


    void init( float fuel_pos, float fuel_pos_enc,float lox_pos, float lox_pos_enc);
    void seek_hardstop(ThrottleControllerOutput& out, float fuel_pos, float fuel_pos_enc,float lox_pos, float lox_pos_enc);
    void end_movement(ThrottleControllerOutput& out, uint32_t timestamp);
    void power_off(ThrottleControllerOutput& out, uint32_t timestamp);
    void repower(ThrottleControllerOutput& out, uint32_t timestamp);
    void complete(ThrottleControllerOutput& out, uint32_t timestamp);
    void measure(ThrottleControllerOutput& out, float fuel_pos,float fuel_pos_enc,float lox_pos, float lox_pos_enc);
    void error(ThrottleControllerOutput& out, uint32_t timestamp);
    int get_phase_id();
    std::pair<ThrottleControllerOutput, ValveCalibrationData> tick(uint32_t timestamp,float fuel_pos, float lox_pos,float fuel_pos_enc, float lox_pos_enc);
}

#endif // APP_STATE_CALIBRATE_VALVE_H
