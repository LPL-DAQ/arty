#include "adc_ads7953.h"

#include <errno.h>
#include <zephyr/device.h>
#include <zephyr/drivers/adc.h>
#include <zephyr/kernel.h>
#include <zephyr/sys/printk.h>
#include <zephyr/sys/util.h>

#define USER_NODE DT_PATH(zephyr_user)

#if !DT_NODE_EXISTS(USER_NODE)
#error "DT_PATH(zephyr_user) does not exist"
#endif

static const struct adc_dt_spec adc_specs[] = {
    ADC_DT_SPEC_GET_BY_NAME(USER_NODE, bank1_ain0),
    ADC_DT_SPEC_GET_BY_NAME(USER_NODE, bank1_ain1),
    ADC_DT_SPEC_GET_BY_NAME(USER_NODE, bank2_ain0),
    ADC_DT_SPEC_GET_BY_NAME(USER_NODE, bank2_ain1),
};

static const char *const adc_names[] = {
    "bank1_ain0",
    "bank1_ain1",
    "bank2_ain0",
    "bank2_ain1",
};

static bool adc_ads7953_initialized;

static int setup_all_channels(void)
{
    for (int i = 0; i < ARRAY_SIZE(adc_specs); i++) {
        if (!adc_is_ready_dt(&adc_specs[i])) {
            printk("adc_ads7953: channel %d not ready\n", i);
            return -ENODEV;
        }

        int ret = adc_channel_setup_dt(&adc_specs[i]);
        if (ret < 0) {
            printk("adc_ads7953: adc_channel_setup_dt[%d] failed: %d\n", i, ret);
            return ret;
        }
    }

    return 0;
}

static int read_one_channel(const struct adc_dt_spec *spec, int16_t *sample)
{
    struct adc_sequence seq = {0};

    adc_sequence_init_dt(spec, &seq);
    seq.buffer = sample;
    seq.buffer_size = sizeof(*sample);
    seq.calibrate = false;
    seq.oversampling = 0;

    return adc_read_dt(spec, &seq);
}

const char *adc_ads7953_channel_name(enum adc_ads7953_channel ch)
{
    if (ch < 0 || ch >= ADC_ADS7953_CHANNEL_COUNT) {
        return "invalid";
    }

    return adc_names[ch];
}

int adc_ads7953_init(void)
{
    int ret;

    if (adc_ads7953_initialized) {
        return 0;
    }

    ret = setup_all_channels();
    if (ret < 0) {
        return ret;
    }

    adc_ads7953_initialized = true;
    return 0;
}

int adc_ads7953_read(enum adc_ads7953_channel ch, int16_t *sample)
{
    if (sample == NULL) {
        return -EINVAL;
    }

    if (ch < 0 || ch >= ADC_ADS7953_CHANNEL_COUNT) {
        return -EINVAL;
    }

    if (!adc_ads7953_initialized) {
        int ret = adc_ads7953_init();
        if (ret < 0) {
            return ret;
        }
    }

    return read_one_channel(&adc_specs[ch], sample);
}

int adc_ads7953_read_all(int16_t samples[ADC_ADS7953_CHANNEL_COUNT])
{
    if (samples == NULL) {
        return -EINVAL;
    }

    if (!adc_ads7953_initialized) {
        int ret = adc_ads7953_init();
        if (ret < 0) {
            return ret;
        }
    }

    for (int i = 0; i < ADC_ADS7953_CHANNEL_COUNT; i++) {
        int ret = read_one_channel(&adc_specs[i], &samples[i]);
        if (ret < 0) {
            printk("adc_ads7953: read %s failed: %d\n", adc_names[i], ret);
            return ret;
        }
    }

    return 0;
}
