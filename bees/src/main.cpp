#include <zephyr/kernel.h>
#include <zephyr/drivers/gpio.h>

static gpio_dt_spec dir_gpio = GPIO_DT_SPEC_GET(DT_NODELABEL(user_led), gpios);

int main(void)
{
    while(true) {
        k_sleep(K_MSEC(500));
        gpio_pin_toggle_dt(&dir_gpio);
        // gpio_pin_set_dt(&dir_gpio, 1);
    }
}
