#include "AnalogSensors.h"

std::expected<void, Error> AnalogSensors::init()
{
    return {};
}

std::expected<void, Error> AnalogSensors::handle_configure_analog_sensors(
    const ConfigureAnalogSensorsRequest& req)
{
    (void)req;
    return {};
}

void AnalogSensors::start_sense()
{
}

std::optional<std::pair<AnalogSensorReadings, float>> AnalogSensors::read()
{
    return std::nullopt;
}
