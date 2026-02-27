#ifndef APP_CALIBRATION_STATE_H
#define APP_CALIBRATION_STATE_H

#include "Controller.h"
#include "ThrottleValve.h"

#include <cmath>

namespace CalibrationState {




    void init();

    void seek_hardstop(ControllerOutput& out, float fuel_pos,float fuel_pos_enc,float lox_pos, float lox_pos_enc);
    void back_off(ControllerOutput& out ,float fuel_pos_enc,float lox_pos_enc);
    void end_movement(ControllerOutput& out, uint32_t timestamp);
    void power_off(ControllerOutput& out, uint32_t timestamp);
    void repower(ControllerOutput& out, uint32_t timestamp);
    void complete(ControllerOutput& out);
    void error(ControllerOutput& out, uint32_t timestamp);

    ControllerOutput tick(uint32_t timestamp,float fuel_pos,float fuel_pos_enc,float lox_pos, float lox_pos_enc);
}

#endif // APP_CALIBRATION_STATE_H
