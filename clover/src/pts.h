#ifndef ARTY_PTS_H
#define ARTY_PTS_H

#include "Error.h"

#include <expected>
#include <zephyr/devicetree.h>
#include <stdint.h>
/*
 * The following macro magic is used to generate a struct to hold the result of one PT reading. If the device tree has:
 *     ...
 *     names = "pt201", "pt202", "pt203", "pt204";
 *     io-channels = <&adc1 7>, <&adc1 8>, <&adc1 12>, <&adc1 11>;
 *     ...
 *
 * We will generate the following:
 * struct pt_readings {
 *     float pt201;
 *     float pt202;
 *     ...
 * }
 */

#define USER_NODE DT_PATH(zephyr_user)

#define ARTY_PTS_DT_TO_READINGS_FIELD(node_id, prop, idx) float DT_STRING_TOKEN_BY_IDX(node_id, prop, idx);
struct pt_readings {
    DT_FOREACH_PROP_ELEM(USER_NODE, pt_names, ARTY_PTS_DT_TO_READINGS_FIELD)
};

struct pt_config {
    float scale;
    float bias;
    float range;
};

struct pts_metrics {
    uint32_t last_latency_us;
};

constexpr int NUM_PTS = DT_PROP_LEN(USER_NODE, io_channels);
extern pt_config pt_configs[NUM_PTS];

// --- Existing API ---
std::expected<void, Error> pts_init();
pt_readings pts_sample();
pt_readings pts_get_last_reading();
float pts_get_adc_read_time_ns();
std::expected<void, Error> pts_set_bias(int index, float bias);
std::expected<void, Error> pts_set_range(int index, float range);

// --- Async API ---
void pts_request_async();
bool pts_try_collect(pt_readings* out);

#endif  // ARTY_PTS_H
