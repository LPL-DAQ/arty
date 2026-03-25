#include <zephyr/kernel.h>
#include <zephyr/device.h>
#include <zephyr/drivers/adc.h>
#include <zephyr/arch/cpu.h>  // Required for direct register access (SPI1->SR)

/* Get ADC device from devicetree label */
#define ADC_NODE DT_NODELABEL(ads7953)

int main(void)
{
    const struct device *adc_dev = DEVICE_DT_GET(ADC_NODE);

    /* 1. Check if driver is ready */
    if (!device_is_ready(adc_dev)) {
        printk("ADC Device not ready!\n");
        return -1;
    }

    /* 2. Configure ADC channel 0 */
    struct adc_channel_cfg ch_cfg = {
        .gain = ADC_GAIN_1,
        .reference = ADC_REF_INTERNAL,
        .acquisition_time = ADC_ACQ_TIME_DEFAULT,
        .channel_id = 0, /* ADS7953 CH0 */
    };

    int setup_ret = adc_channel_setup(adc_dev, &ch_cfg);
    if (setup_ret != 0) {
        printk("ADC Channel Setup Failed: %d\n", setup_ret);
    }

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

        if (ret == 0) {
            /* Read Success */
            printk("ADC Read Success: %d | SPI1_SR: 0x%08x\n", sample_val, SPI1->SR);
        } else {
            /* Read Failed - Detailed Hardware Debug */
            printk("ADC Read Failed! Error: %d | SPI1_SR: 0x%08x | GPIOA_IDR: 0x%08x\n",
                    ret, SPI1->SR, GPIOA->IDR);

            // Check specific SPI error flags
            if (SPI1->SR & 0x00000400) printk(" -> SPI Overrun Error detected!\n");
            if (SPI1->SR & 0x00000001) printk(" -> SPI Rx Buffer Not Empty\n");
        }

        /* Using Busy Wait as requested (1,000,000 us = 1 second) */
        k_busy_wait(1000000);
    }
}
