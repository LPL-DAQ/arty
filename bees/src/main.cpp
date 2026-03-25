#include <zephyr/kernel.h>
#include <zephyr/device.h>
#include <zephyr/drivers/adc.h>

/* Get ADC device from devicetree label */
#define ADC_NODE DT_NODELABEL(ads7953)

int main(void)
{
    const struct device *adc_dev = DEVICE_DT_GET(ADC_NODE);

    /* 1. Check if driver is ready */
    if (!device_is_ready(adc_dev)) {
        return -1; // If it stops here, driver didn't load
    }

    /* 2. Configure ADC channel 0 */
    struct adc_channel_cfg ch_cfg = {
        .gain = ADC_GAIN_1,
        .reference = ADC_REF_INTERNAL,
        .acquisition_time = ADC_ACQ_TIME_DEFAULT,
        .channel_id = 0, /* ADS7953 CH0 */
    };
    adc_channel_setup(adc_dev, &ch_cfg);

    /* 3. Prepare sequence and buffer */
    int16_t sample_val = 0;
    struct adc_sequence sequence = {
        .channels = BIT(0),
        .buffer = &sample_val,
        .buffer_size = sizeof(sample_val),
        .resolution = 12,
    };

    while (1) {
        /* 4. Read ADC value (Internal driver handles SPI/CS) */
        int ret = adc_read(adc_dev, &sequence);
        __ASSERT(ret == 0, "adc_read failed: %d", ret);

        if (ret == 0) {
            /* Success! Check 'sample_val' in Debugger (Range: 0-4095) */
        }

        /* Using Busy Wait as you requested */
        k_busy_wait(1000000);
    }
}
