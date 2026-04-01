#include "AnalogSensors.h"
#include "MutexGuard.h"
#include "clover.pb.h"
#include "config.h"
#include <optional>
#include <tuple>
#include <zephyr/drivers/adc.h>
#include <zephyr/kernel.h>
#include <zephyr/logging/log.h>

static const device* adc_device = DEVICE_DT_GET(DT_ALIAS(analog_sensors_adc));

static constexpr int OVERSAMPLING = DT_PROP(DT_PATH(zephyr_user), analog_sensors_adc_oversampling);
static constexpr int RESOLUTION = DT_PROP(DT_PATH(zephyr_user), analog_sensors_adc_resolution);

// Subset of ADC channels we make available to be assigned to PTs/TCs.
static constexpr int NUM_ANALOG_CHANNELS = DT_PROP_LEN(DT_PATH(zephyr_user), io_channels);
#define ADC_CHANNEL_DT_SPEC_AND_COMMA(node_id, prop, idx) ADC_DT_SPEC_GET_BY_IDX(node_id, idx),
static constexpr struct adc_dt_spec adc_channels[NUM_ANALOG_CHANNELS] = {
    DT_FOREACH_PROP_ELEM(DT_PATH(zephyr_user), io_channels, ADC_CHANNEL_DT_SPEC_AND_COMMA)};

// Spec and buffers used for ADC reads.

// Generates an ADC channel bitmask from the devicetree config.
static consteval uint32_t channels_bitmask()
{
    uint32_t mask = 0;
    for (const auto& channel : adc_channels) {
        mask |= (1 << channel.channel_id);
    }
    return mask;
}

// Buffer into which raw readings are written. Only accessed by the analog_sensors thread.
static std::array<uint16_t, NUM_ANALOG_CHANNELS> raw_readings;

// Sequence read options
static adc_sequence_options adc_read_options = {
    .interval_us = 0,
    .extra_samplings = 0,  // No multi-sampling
};

// Sequence spec used to read from ADC. Only accessed by the analog_sensors thread.
static adc_sequence adc_read_seq = {
    .options = &adc_read_options,
    .channels = channels_bitmask(),
    .buffer = raw_readings.data(),
    .buffer_size = sizeof raw_readings,
    .resolution = RESOLUTION,
    .oversampling = OVERSAMPLING};

// Signals readiness
K_SEM_DEFINE(ready_sem, 0, 1);

// Taken every sensor read to coordinate read times with controller ticks.
K_SEM_DEFINE(allow_sense_sem, 0, 1);

// Sensor outputs
static bool has_reading = false;
static PTs pts = PTs_init_default;
static TCs tcs = TCs_init_default;
static float sense_time_ns = 0.0f;
K_MUTEX_DEFINE(reading_mutex);

LOG_MODULE_REGISTER(AnalogSensors, CONFIG_LOG_DEFAULT_LEVEL);

/// Continuously sense, coordinating with control ticks so the reading is ready at the start of each one.
static void sense()
{
    // Await initialization
    k_sem_take(&ready_sem, K_FOREVER);
    LOG_INF("Sense loop initiated");

    while (true) {
        // Single read is allowed at the end of a controller tick.
        k_sem_take(&allow_sense_sem, K_FOREVER);

        int err = adc_read(adc_device, &adc_read_seq);

        // TODO -- configure stuff

        {
            MutexGuard reading_guard{&reading_mutex};
            has_reading = true;
            pts.has_pt102 = true;
            pts.pt102 = 67.0f;
            // TODO
            // and so on
        }
    }
}

K_THREAD_DEFINE(analog_sensors, 2048, sense, nullptr, nullptr, nullptr, ANALOG_SENSORS_THREAD_PRIORITY, 0, 0);

/// Check ADC readiness and initialize all ADC channels we're using.
std::expected<void, Error> AnalogSensors::init()
{
    LOG_INF("Checking ADC readiness");
    if (!device_is_ready(adc_device)) {
        return std::unexpected(Error::from_device_not_ready(adc_device).context("AnalogSensors ADC is not ready"));
    }

    // Configure ADC channels.
    for (const auto& channel : adc_channels) {
        LOG_INF("Initializing channel %d", channel.channel_id);

        if (int err = adc_channel_setup_dt(&channel)) {
            return std::unexpected(Error::from_code(err).context("failed to set up ADC channel %d", channel.channel_id));
        }
    }

    LOG_INF("Initiating sense loop");
    k_sem_give(&ready_sem);

    return {};
}

/// Called from at the end of a control tick to allow AnalogSensors to initiate a sensor reading.
void AnalogSensors::start_sense()
{
    // Semaphore count is only non-zero if sense is taking longer than a controller tick and is
    // still in-progress. In this case, we guard against giving multiple semaphores and just allow
    // it to immediately re-sense when it finishes its current job.
    if (k_sem_count_get(&allow_sense_sem) == 0) {
        k_sem_give(&allow_sense_sem);
    }
}

/// Returns the last read PTs, TCs, and sense time, if it is ready.
std::optional<std::tuple<PTs, TCs, float>> AnalogSensors::read()
{
    MutexGuard reading_guard{&reading_mutex};

    if (!has_reading) {
        return std::nullopt;
    }
    return {{pts, tcs, sense_time_ns}};
    return std::make_optional(std::make_tuple(pts, tcs, sense_time_ns));
}
