#include <zephyr/kernel.h>
#include <zephyr/drivers/gpio.h>

/* ---- Denny ---- */
#include <extmem.h>
#include <stdint.h>
#define ENABLE_QSPI_MMAP_DEMO

__attribute__((section(".ext_rodata"))) __attribute__((used))
const uint8_t test_table[1024] = {1};
//

static const device* rcc_dev = DEVICE_DT_GET(DT_NODELABEL(rcc));
static gpio_dt_spec dir_gpio = GPIO_DT_SPEC_GET(DT_PATH(zephyr_user), led_test_gpios);

int main(void)
{
    /* enable QSPI memory-mapped mode */
    #ifdef ENABLE_QSPI_MMAP_DEMO
        extmem_enable_mmap();
        volatile uint8_t v = test_table[0];
        printk("ext[0]=%d\n", v);
    #endif
    //

    int my_var = rcc_dev->state->init_res;
    if(my_var == 111) {
        k_sleep(K_MSEC(1000));
        return 1;
    }

    int ret = device_is_ready(rcc_dev);
    if(!ret) {
        k_sleep(K_MSEC(1000));
        return 1;
    }

    ret = gpio_is_ready_dt(&dir_gpio);
    if (!ret) {
        k_sleep(K_MSEC(1000));
		return 0;
	}

	ret = gpio_pin_configure_dt(&dir_gpio, GPIO_OUTPUT_ACTIVE);
	if (ret < 0) {
        k_sleep(K_MSEC(1000));
		return 0;
	}

    while(true) {
        gpio_pin_toggle_dt(&dir_gpio);
        k_sleep(K_MSEC(500));
        // gpio_pin_set_dt(&dir_gpio, 1);
    }
}
