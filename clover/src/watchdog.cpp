#include "watchdog.h"

#include <zephyr/kernel.h>
#include <zephyr/device.h>
#include <zephyr/drivers/watchdog.h>
#include <zephyr/logging/log.h>
#include <zephyr/drivers/hwinfo.h>


LOG_MODULE_REGISTER(watchdog_mgr, LOG_LEVEL_INF);

#define WDT_NODE DT_ALIAS(watchdog0)

namespace Watchdog {

static const struct device* wdt = DEVICE_DT_GET(WDT_NODE);
static int channel = -1; 


// ** CRITICAL WATCHDOG BEHAVIOR FUNCTIONS (NO LOGGING DUE TO UNCERTAINTY OF COMMS)
// * on_watchdog_timeout() - Called right before the watchdog restarts the teensy.
// * on_watchdog_reboot() - Called when teensy starts up after a watchdog reboot.

static void on_watchdog_timeout(const ::device *dev, int channel_id) {
    // Function to be called before the Teensy restarts.
    // Currently does nothing, should be kept minimal.

    // Avoids compiler warnings for unused arguments - can be removed if we decide to actually make this function do something.
    ARG_UNUSED(dev);
    ARG_UNUSED(channel_id);

    return;
}

void on_watchdog_reboot() {
    // Function designed to be called when the Teensy starts up and Watchdog was the cause of the reboot.
    // Currently does nothing. This function is designed to be called BEFORE logging is established, and therefore cannot leverage comms.
    return;
}

// ** WATCHDOG CORE FUNCTIONS
// * init() - Configures the watchdog. 
// * feed() - Resets watchdog timer (must be called continuously so as to not trip the reset)
// * evaluate_reboot_cause() - Called on Startup to check if watchdog was the cause of the last hardware reset.

bool init(uint32_t timeout_ms) {
    LOG_INF("Watchdog device: %s, base: 0x%lx",
        wdt->name,
        (unsigned long)DT_REG_ADDR(WDT_NODE));

    if (!device_is_ready(wdt)) {
        LOG_ERR("Watchdog device %s is not ready", wdt->name);
        return false;
    }

    struct wdt_timeout_cfg config = {};
    config.window.min = 0;
    config.window.max = timeout_ms; // SETS MAXIMUM TIMEOUT
    config.callback = on_watchdog_timeout; 
    config.flags = WDT_FLAG_RESET_SOC;

    channel = wdt_install_timeout(wdt, &config);
    if (channel < 0) {
        LOG_ERR("wdt_install_timeout failed (%d)", channel);
        return false;
    }

    int err = wdt_setup(wdt, 0);
    if (err < 0) {
        LOG_ERR("wdt_setup failed (%d)", err);
        return false;
    }

    return true;
}

void feed() {
    // Feed the Watchdog
    int err = wdt_feed(wdt, channel);

    // Ensure the feed went through successfully
    if (err != 0) {
        LOG_ERR("wdt_feed failed (%d)", err);
    }
}

RebootCheckResult evaluate_reboot_cause() {
    // Checks how the teensy started up (can maybe be moved to another file in the future...)
    // NOTE THAT LOGGING HAS BEEN REMOVED. This function is designed to be called as soon as the teensy boots up before logs are established..

    // Returns a RebootCheckResult struct
    RebootCheckResult result{};

    // Try to get the cause
    result.read_error = hwinfo_get_reset_cause(&result.cause);
    if (result.read_error != 0) {
        // Cause is unknown. Don't clear flags we haven't captured.
        return result;
    }

    // If successful, we determined the cause successfully.
    result.cause_available = true;
    result.watchdog_reset =
        (result.cause & RESET_WATCHDOG) != 0;

    // Preserve the detected cause even if clearing fails.
    result.clear_error = hwinfo_clear_reset_cause();

    if (result.watchdog_reset) {
        on_watchdog_reboot();
    }

    return result;
}

} // namespace Watchdog