#include <zephyr/kernel.h>
#include <zephyr/device.h>
#include <zephyr/drivers/adc.h>
#include <zephyr/usb/usb_device.h>
#include <zephyr/sys/printk.h>

#define ADC_SPEC_NODE DT_PATH(zephyr_user)

static const struct adc_dt_spec ads_adc =
    ADC_DT_SPEC_GET_BY_NAME(ADC_SPEC_NODE, ain0);

volatile int g_init_ret = 0;
volatile int g_ret = 0;
volatile int g_match = 0;

int16_t g_raw[1] = {0};

static struct adc_sequence g_seq = {
    .buffer = g_raw,
    .buffer_size = sizeof(g_raw),
};

int main(void)
{
    printk("main start\n");

    int usb_ret = usb_enable(NULL);
    printk("usb_enable ret=%d\n", usb_ret);

    printk("before adc_is_ready_dt\n");
    bool ready = adc_is_ready_dt(&ads_adc);
    printk("adc_is_ready_dt=%d\n", ready ? 1 : 0);

    if (!ready) {
        printk("ADC not ready\n");
        while (1) {
            k_msleep(1000);
        }
    }

    printk("before adc_channel_setup_dt\n");
    int ret = adc_channel_setup_dt(&ads_adc);
    printk("adc_channel_setup_dt ret=%d\n", ret);

    if (ret < 0) {
        printk("ADC channel setup failed\n");
        while (1) {
            k_msleep(1000);
        }
    }

    printk("before adc_sequence_init_dt\n");
    adc_sequence_init_dt(&ads_adc, &g_seq);

    while (1) {
        g_raw[0] = 0;

        printk("before adc_read_dt\n");
        g_ret = adc_read_dt(&ads_adc, &g_seq);
        printk("adc_read_dt ret=%d raw=%d\n", g_ret, g_raw[0]);

        g_match = (g_ret == 0) ? 1 : 0;
        k_msleep(1000);
    }
}
