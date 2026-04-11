#include <expected>
#include "Error.h"

template<const device* lidar_dt_init>
class Lidar {
private:
    k_mutex lidar_lock;

public:
std::expected<void, Error> init();

std::expected<void, Error> handle_configure_analog_sensors(const ConfigureAnalogSensorsRequest& req);

void start_sense();
std::optional<std::pair<AnalogSensorReadings, float>> read();
}
