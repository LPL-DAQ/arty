#include <zephyr/usb/usb_device.h>
#include <zephyr/sys/printk.h>
#include <zephyr/kernel.h>
#include <zephyr/device.h>
#include <zephyr/drivers/adc.h>

/* Reference the ADC channel defined in devicetree (zephyr,user) */
#define ADC_SPEC_NODE DT_PATH(zephyr_user)

/* Get ADC specification from devicetree */
static const struct adc_dt_spec ads_adc =
    ADC_DT_SPEC_GET_BY_NAME(ADC_SPEC_NODE, ain0);

/* Debug variables (observe via debugger) */
volatile int g_init_ret = 0;
volatile int g_ret = 0;
volatile int g_match = 0;

/* Buffer to store raw ADC sample */
int16_t g_raw[1] = {0};

/* Keep ADC sequence out of the stack */
static struct adc_sequence g_seq = {
    .buffer = g_raw,
    .buffer_size = sizeof(g_raw),
};

int main(void)
{
    usb_enable(nullptr);

    /* Check if ADC device is ready */
    if (!adc_is_ready_dt(&ads_adc)) {
        g_init_ret = -1;
        while (1) {
            k_msleep(1000);
        }
    }

    /* Configure ADC channel based on devicetree settings */
    int ret = adc_channel_setup_dt(&ads_adc);
    if (ret < 0) {
        g_init_ret = ret;
        while (1) {
            k_msleep(1000);
        }
    }

    /* Initialize sequence from devicetree */
    adc_sequence_init_dt(&ads_adc, &g_seq);

    while (1) {
        /* Clear previous sample */
        g_raw[0] = 0;

        /* Perform ADC read (driver handles SPI internally) */
        g_ret = adc_read_dt(&ads_adc, &g_seq);

        /* Check if read was successful */
        if (g_ret == 0) {
            g_match = 1;  /* Read success */
        } else {
            g_match = 0;  /* Read failed */
        }

        k_msleep(1000);
    }

    return 0;
}


