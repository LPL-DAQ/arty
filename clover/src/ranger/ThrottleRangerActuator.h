#ifndef APP_THROTTLE_RANGER_H
#define APP_THROTTLE_RANGER_H

#include <cmath>
#include "../Error.h"
#include "../ThrottleController.h"

namespace ThrottleRangerActuator {



std::expected<void, Error> tick(ThrottleRangerStateOutput& output, DataPacket& data);
std::expected<void, Error> reset_valve_position(Valve valve, float new_pos_deg);


// fuel, lox
float fuel_get_pos_internal();
float fuel_get_pos_encoder();
float lox_get_pos_internal();
float lox_get_pos_encoder();
void fuel_reset_pos(float new_pos);
void lox_reset_pos(float new_pos);
bool fuel_get_power_on();
bool lox_get_power_on();

}
#endif // APP_THROTTLE_RANGER_H
