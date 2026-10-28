#include "watchdog.h"

#include <zephyr/kernel.h>
#include <zephyr/device.h>
#include <zephyr/drivers/watchdog.h>
#include <zephyr/logging/log.h>

#include <zephyr/arch/cpu.h>

LOG_MODULE_REGISTER(watchdog_mgr, LOG_LEVEL_INF);

#define WDT_NODE DT_ALIAS(watchdog0)

namespace Watchdog {

static const struct device* s_wdt_dev = DEVICE_DT_GET(WDT_NODE);
static int s_channel_id = -1;

bool init(uint32_t timeout_ms)
{
    if (!device_is_ready(s_wdt_dev)) {
        LOG_ERR("Watchdog device %s is not ready", s_wdt_dev->name);
        return false;
    }

    struct wdt_timeout_cfg config = {
        .window = {
            .min = 0,
            .max = timeout_ms,
        },
        .callback = nullptr,
        .flags = WDT_FLAG_RESET_SOC,
    };

    s_channel_id = wdt_install_timeout(s_wdt_dev, &config);
    if (s_channel_id < 0) {
        LOG_ERR("Watchdog install timeout failed (%d)", s_channel_id);
        return false;
    }

    // int err = wdt_setup(s_wdt_dev, WDT_OPT_PAUSE_HALTED_BY_DBG);
    int err = wdt_setup(s_wdt_dev, 0);

    if (err < 0) {
        LOG_ERR("Watchdog setup failed (%d)", err);
        return false;
    }

    // 3. HARDWARE OVERRIDE: Calculate WT value and force WDE bit high
    // Formula for NXP WDOG1: Timeout (sec) = (WT + 1) * 0.5
    uint16_t wt_val = (timeout_ms / 500) - 1;
    if (wt_val > 0xFF) {
        wt_val = 0xFF;
    }

    // Build WCR word: WT in bits[15:8], WDA(bit 5), SRS(bit 4), WDE(bit 2)
    uint16_t wcr_val = (wt_val << 8) | (1 << 5) | (1 << 4) | (1 << 2);
    
    // Write directly to hardware to override register locking
    WDOG1->WCR = wcr_val;

    LOG_INF("Watchdog hardware initialized with %u ms timeout", timeout_ms);
    LOG_INF("WDOG1 WCR (Control): 0x%04X", (uint16_t)WDOG1->WCR);

    return true;
}

void feed()
{
    // if (s_channel_id >= 0) {
    //     int err = wdt_feed(s_wdt_dev, s_channel_id);
    //     if (err != 0) {
    //         LOG_ERR("wdt_feed failed with error: %d", err);
    //     }
    //     else {
    //         LOG_INF("wdt_feed success");
    //     }
    // } else {
    //     LOG_ERR("Cannot feed: s_channel_id is invalid (%d)", s_channel_id);
    // }
    WDOG1->WSR = 0x5555;
    WDOG1->WSR = 0xAAAA;
}

} // namespace Watchdog