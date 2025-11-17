#ifndef CLOVER_PTS_H
#define CLOVER_PTS_H

#include <zephyr/devicetree.h>

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

#define CLOVER_PTS_DT_TO_READINGS_FIELD(node_id, prop, idx) float DT_STRING_TOKEN_BY_IDX(node_id, prop, idx);
struct pt_readings {
    DT_FOREACH_PROP_ELEM(USER_NODE, pt_names, CLOVER_PTS_DT_TO_READINGS_FIELD)
};

struct pt_config {
    float scale; // psig per analog reading unit. For teensy, resolution = 12, so for a 1k PT this would be (1000.0 / 4096.0)
    float bias;
    float range;
};

constexpr int NUM_PTS = DT_PROP_LEN(USER_NODE, io_channels);
extern pt_config pt_configs[NUM_PTS];

int pts_init();

pt_readings pts_sample();

void pts_log_readings(const pt_readings &readings);

int pts_set_bias(int index, float bias);

int pts_set_range(int index, float range);

#endif //CLOVER_PTS_H
