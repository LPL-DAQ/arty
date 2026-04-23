#include "RangerTvc.h"
#include <zephyr/kernel.h>
#include <zephyr/device.h>
#include <zephyr/drivers/can.h>
#include <math.h> // For isfinite()
#include "Moteus.h" 

#define TVC_STACK_SIZE 4096 // Smaller stack is safer when using static objects
#define TVC_PRIORITY 5

CAN_MSGQ_DEFINE(tvc_rx_msgq, 10);

// Move the object out of the thread stack to prevent overflows
static Moteus moteus1; 

void tvc_thread(void) {
    k_sleep(K_SECONDS(5));
    printk("\n--- ENABLING HARDENED TVC LOOP ---\n");

    const struct device *can_dev = DEVICE_DT_GET(DT_CHOSEN(zephyr_canbus));
    
    // Hardware Handshake
    can_stop(can_dev);
    k_sleep(K_MSEC(100));
    can_set_mode(can_dev, CAN_MODE_FD);
    k_sleep(K_MSEC(100));
    can_start(can_dev);

    const struct can_filter filter = { .id = 0, .mask = 0, .flags = CAN_FILTER_IDE };
    can_add_rx_filter_msgq(can_dev, &tvc_rx_msgq, &filter);

    moteus1.Initialize(); 
    printk("System Online. Starting 10Hz warmup...\n");

    uint32_t loop_count = 0;

    while (1) {
        int64_t start_time = k_uptime_get();

        Moteus::PositionMode::Command spin_cmd;
        spin_cmd.position = 0.0f;       
        spin_cmd.velocity = 0.5f; 
        spin_cmd.accel_limit = 1.0f;    
        spin_cmd.maximum_torque = 0.5f; 
        spin_cmd.kp_scale = 0.0f; 
        spin_cmd.kd_scale = 1.0f; 

        auto payload = moteus1.MakePosition(spin_cmd);

        struct can_frame z_frame = {0};
        z_frame.id = payload.arbitration_id; 
        z_frame.flags = CAN_FRAME_IDE | CAN_FRAME_FDF | CAN_FRAME_BRS; 
        z_frame.dlc = can_bytes_to_dlc(payload.size);
        memcpy(z_frame.data, payload.data, payload.size);

        can_send(can_dev, &z_frame, K_MSEC(5), NULL, NULL);

        // --- FIXED TELEMETRY BLOCK ---
        struct can_frame rx_frame; // Declared ONCE
        while (k_msgq_get(&tvc_rx_msgq, &rx_frame, K_NO_WAIT) == 0) {
            // SNIFFER: Print raw ID of anything hitting the Teensy
            if (loop_count % 10 == 0) {
                printk("  RAW RX: ID 0x%08X | DLC %d\n", rx_frame.id, rx_frame.dlc);
            }
            moteus1.ParseTelemetry(rx_frame.data, can_dlc_to_bytes(rx_frame.dlc));
        }

        // --- PRINT TELEMETRY ---
        if (loop_count % 10 == 0) {
            float pos = moteus1.last_result().values.position;
            float vel = moteus1.last_result().values.velocity;

            if (isfinite(pos) && isfinite(vel)) {
                printk("M1 Pos: %d | Velo: %d (mdeg/sec)\n", 
                       (int)(pos * 1000.0f), (int)(vel * 1000.0f));
            } else {
                printk("M1: Waiting for valid telemetry...\n");
            }
        }
        loop_count++;

        int delay = (loop_count < 50) ? 100 : 20;
        int64_t elapsed = k_uptime_get() - start_time;
        if (elapsed < delay) {
            k_sleep(K_MSEC(delay - elapsed));
        }
    }
}

K_THREAD_DEFINE(tvc_tid, TVC_STACK_SIZE, tvc_thread, NULL, NULL, NULL, TVC_PRIORITY, 0, 0);