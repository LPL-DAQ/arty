#include "AnalogSensors.h"

#include <zephyr/kernel.h>
#include <zephyr/device.h>
#include <zephyr/drivers/adc.h>
#include <zephyr/devicetree.h>
#include <zephyr/sys/util.h>

#define USER_NODE DT_PATH(zephyr_user)

static const struct adc_dt_spec adc_specs[] = {
    ADC_DT_SPEC_GET_BY_NAME(USER_NODE, bank1_ain0),
    ADC_DT_SPEC_GET_BY_NAME(USER_NODE, bank1_ain1),
    ADC_DT_SPEC_GET_BY_NAME(USER_NODE, bank2_ain0),
    ADC_DT_SPEC_GET_BY_NAME(USER_NODE, bank2_ain1),
};

static int read_one_channel(const struct adc_dt_spec *spec, int16_t *sample)
{
    struct adc_sequence seq = {};
    adc_sequence_init_dt(spec, &seq);

    seq.buffer = sample;
    seq.buffer_size = sizeof(*sample);
    seq.calibrate = false;
    seq.oversampling = 0;

    return adc_read_dt(spec, &seq);
}

std::expected<void, Error> AnalogSensors::init()
{
    for (size_t i = 0; i < ARRAY_SIZE(adc_specs); ++i) {
        if (!adc_is_ready_dt(&adc_specs[i])) {
            return std::unexpected(Error::from_code(-ENODEV));
        }

        int ret = adc_channel_setup_dt(&adc_specs[i]);
        if (ret < 0) {
            return std::unexpected(Error::from_code(ret));
        }
    }

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
    int16_t raw[ARRAY_SIZE(adc_specs)] = {0};

    for (size_t i = 0; i < ARRAY_SIZE(adc_specs); ++i) {
        int ret = read_one_channel(&adc_specs[i], &raw[i]);
        if (ret < 0) {
            return std::nullopt;
        }
    }

    AnalogSensorReadings readings = AnalogSensorReadings_init_zero;

    readings.has_pt001 = true;
    readings.pt001 = static_cast<float>(raw[0]);

    readings.has_pt002 = true;
    readings.pt002 = static_cast<float>(raw[1]);

    readings.has_pt003 = true;
    readings.pt003 = static_cast<float>(raw[2]);

    readings.has_pt004 = true;
    readings.pt004 = static_cast<float>(raw[3]);

    float timestamp_s = static_cast<float>(k_uptime_get()) / 1000.0f;
    return std::make_pair(readings, timestamp_s);
}
