#include <zephyr/kernel.h>
#include <zephyr/device.h>
#include <zephyr/drivers/adc.h>
#include <zephyr/usb/usb_device.h>
#include <zephyr/sys/printk.h>
#include <zephyr/sys/util.h>

#define USER_NODE DT_PATH(zephyr_user)

static const struct adc_dt_spec adc_specs[] = {
    ADC_DT_SPEC_GET_BY_NAME(USER_NODE, bank1_ain0),
    ADC_DT_SPEC_GET_BY_NAME(USER_NODE, bank1_ain1),
    ADC_DT_SPEC_GET_BY_NAME(USER_NODE, bank2_ain0),
    ADC_DT_SPEC_GET_BY_NAME(USER_NODE, bank2_ain1),
};

static int setup_all_channels(void)
{
    for (int i = 0; i < ARRAY_SIZE(adc_specs); i++) {
        if (!adc_is_ready_dt(&adc_specs[i])) {
            printk("ADC spec %d not ready\n", i);
            return -ENODEV;
        }

        int ret = adc_channel_setup_dt(&adc_specs[i]);
        if (ret < 0) {
            printk("adc_channel_setup_dt[%d] failed: %d\n", i, ret);
            return ret;
        }

        printk("adc spec %d ready, channel_id=%d\n", i, adc_specs[i].channel_id);
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

int main(void)
{
    printk("main start\n");

    int usb_ret = usb_enable(NULL);
    printk("usb_enable ret=%d\n", usb_ret);

    int ret = setup_all_channels();
    if (ret < 0) {
        printk("ADC setup failed: %d\n", ret);
        while (1) {
            k_msleep(1000);
        }
    }

    while (1) {
        for (int i = 0; i < ARRAY_SIZE(adc_specs); i++) {
            int16_t raw = 0;
            ret = read_one_channel(&adc_specs[i], &raw);

            printk("ain%d ch_id=%d ret=%d raw=%d\n",
                   i, adc_specs[i].channel_id, ret, raw);
        }

        printk("----\n");
        k_msleep(1000);
    }
}
