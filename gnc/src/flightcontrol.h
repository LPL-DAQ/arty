//
// Created by lpl on 11/17/25.
//

#ifndef CLOVER_FLIGHTCONTROL_H
#define CLOVER_FLIGHTCONTROL_H

#include <array>

void centerTVC();

int flight_init();
std::array<float, 2> rotationalPID(float dt);
std::array<float, 2> lateralPID(float dt);
float verticalPID(float dt);
float headingPID(float dt);
void updateState();

int flight_control_loop_step(double dt);
void flight_stop();


#endif //CLOVER_FLIGHTCONTROL_H