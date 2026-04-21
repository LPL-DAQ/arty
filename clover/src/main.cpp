#include <zephyr/kernel.h>
#include <zephyr/sys/printk.h>
#include <zephyr/usb/usb_device.h>
#include "AnalogSensors.h"

int main(void)
{
    printk("main start\n");

    int usb_ret = usb_enable(NULL);
    printk("usb_enable ret=%d\n", usb_ret);

    auto ret = AnalogSensors::init();
    if (!ret) {
        printk("AnalogSensors init failed\n");
        while (1) {
            k_msleep(1000);
        }
    }

    AnalogSensors::start_sense();

    while (1) {
        auto sample = AnalogSensors::read();
        if (sample) {
            const auto& [readings, ts] = *sample;

            printk("analog sample at %.3f s\n", static_cast<double>(ts));
            printk("pt001=%.3f pt002=%.3f pt003=%.3f pt004=%.3f\n",
                   static_cast<double>(readings.pt001),
                   static_cast<double>(readings.pt002),
                   static_cast<double>(readings.pt003),
                   static_cast<double>(readings.pt004));
        } else {
            printk("AnalogSensors read failed\n");
        }

        k_msleep(100);
    }
}
