#define __APPLE__ 
#include "moteus.h"

#include <zephyr/kernel.h>
#include <limits>
#include <zephyr/usb/usb_device.h>
#include <zephyr/drivers/uart.h>
#include <zephyr/sys/sys_io.h>
#include <zephyr/sys/printk.h>

int main(void) {
    // 1. Turn on the USB port
    usb_enable(NULL);

    // 2. Give TyCommander exactly 4 seconds to catch up
    for (int i = 4; i > 0; i--) {
        printk("\n>>> TEENSY IS ALIVE! Booting in %d seconds... <<<\n", i);
        k_msleep(1000); 
    }

    // ==========================================
    // BREADCRUMB DIAGNOSTICS
    // ==========================================
    printk("\n\n========================================\n");
    printk("PHASE 1: Hardware Pinmux Override...\n");
    k_msleep(50); 

    printk("PHASE 1.1: Writing TX MUX Register (0x401F8094)...\n");
    k_msleep(50);
    // Correct Address for GPIO_EMC_32 (Teensy Pin 31)
    sys_write32(9, 0x401F8094); 

    printk("PHASE 1.2: Writing TX PAD Register (0x401F8284)...\n");
    k_msleep(50);
    sys_write32(0x10B0, 0x401F8284); 

    printk("PHASE 1.3: Writing RX MUX Register (0x401F8090)...\n");
    k_msleep(50);
    // Correct Address for GPIO_EMC_31 (Teensy Pin 30)
    sys_write32(9, 0x401F8090); 

    printk("PHASE 1.4: Writing RX PAD Register (0x401F8280)...\n");
    k_msleep(50);
    sys_write32(0x10B0, 0x401F8280); 

    printk("PHASE 1.5: Writing RX Daisy Chain Register (0x401F8388)...\n");
    k_msleep(50);
    sys_write32(1, 0x401F8388); 

    printk("PHASE 1 COMPLETE: Hardware FlexCAN3 routed.\n");
    k_msleep(50);

    printk("PHASE 2: Naked Socket Test...\n");
    k_msleep(50);

    printk("-> Opening AF_CAN socket...\n");
    k_msleep(50);
    int fd = socket(AF_CAN, SOCK_RAW, CAN_RAW);
    printk("-> Socket FD returned: %d\n", fd);
    k_msleep(50);

    printk("-> Fetching L2 CANBUS_RAW interface...\n");
    k_msleep(50);
    struct net_if *iface = net_if_get_first_by_type(&NET_L2_GET_NAME(CANBUS_RAW));
    printk("-> Interface Pointer returned: %p\n", (void*)iface);
    k_msleep(50);

    if (iface != NULL) {
        printk("-> Binding socket to interface...\n");
        k_msleep(50);
        struct sockaddr_can addr = {};
        addr.can_family = AF_CAN;
        addr.can_ifindex = net_if_get_by_iface(iface);
        int ret = bind(fd, (struct sockaddr *)&addr, sizeof(addr));
        printk("-> Bind returned: %d\n", ret);
        k_msleep(50);
    } else {
        printk("-> SKIPPING BIND (Interface is NULL!)\n");
        k_msleep(50);
    }

    printk("PHASE 2 COMPLETE: Socket test finished.\n");
    k_msleep(50);

    printk("PHASE 3: Moteus C++ Library Init...\n");
    k_msleep(50);

    try {
        printk("-> Allocating SocketCAN memory...\n");
        k_msleep(50);
        mjbots::moteus::Socketcan::Options transport_opts;
        transport_opts.ignore_errors = true; 
        auto transport = std::make_shared<mjbots::moteus::Socketcan>(transport_opts);
        
        printk("-> Configuring Moteus Controller...\n");
        k_msleep(50);
        mjbots::moteus::Controller::Options controller_opts;
        controller_opts.transport = transport;
        controller_opts.id = 1; 
        controller_opts.default_query = false; 
        mjbots::moteus::Controller controller(controller_opts);

        printk("PHASE 3 COMPLETE: Entering Spin Loop!\n");
        k_msleep(50);
        
        while (1) {
            mjbots::moteus::PositionMode::Command cmd;
            cmd.position = std::numeric_limits<double>::quiet_NaN(); 
            cmd.velocity = 0.1; 
            controller.SetPosition(cmd, nullptr, nullptr);
            k_msleep(10);
        } 
        
    } catch (const std::exception& e) {
        printk("\n!!! C++ FATAL ERROR !!!\n%s\n", e.what());
        k_msleep(100);
    }

    return 0;
}