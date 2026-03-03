#include <zephyr/kernel.h>
#include <zephyr/net/sntp.h>
#include <zephyr/logging/log.h>
#include <time.h>

LOG_MODULE_REGISTER(hotfire_sntp_static, LOG_LEVEL_INF);

// This is the static IP of the laptop running Chrony
#define CHRONY_SERVER_IP "169.254.88.88"
#define SNTP_TIMEOUT_MS 5000

bool sync_time() {
    struct sntp_time ts;
    struct timespec tspec;

    LOG_INF("Sending SNTP request to local Chrony server: %s", CHRONY_SERVER_IP);
    int res = sntp_simple(CHRONY_SERVER_IP, SNTP_TIMEOUT_MS, &ts);

    if (res == 0) {
        LOG_INF("SNTP time received. Epoch seconds: %llu", ts.seconds);

        tspec.tv_sec = ts.seconds;
        tspec.tv_nsec = (uint32_t)(((uint64_t)ts.fraction * 1000000000ULL) >> 32);

        sys_clock_settime(CLOCK_REALTIME, &tspec);
        LOG_INF("System clock updated.");
        return true;
    } else {
        LOG_ERR("Failed to get SNTP time. Error code: %d", res);
        return false;
    }
}

int sntp_init() {
    LOG_INF("Booting");

    // Give the network stack a brief moment to initialize the physical Ethernet link
    k_sleep(K_SECONDS(2));

    // Keep trying until it successfully gets the time from the Laptop
    while (!sync_time()) {
        LOG_WRN("Retrying SNTP sync in 0.2 seconds...");
        k_sleep(K_MSEC(200));
    }

    struct timespec tspec;

    /* Main loop */
    sys_clock_gettime(CLOCK_REALTIME, &tspec);

    LOG_INF("Timestamp: %lld.%09ld", (long long)tspec.tv_sec, tspec.tv_nsec);

    return 0;
}

timespec get_system_time() {
    struct timespec tspec;
    sys_clock_gettime(CLOCK_REALTIME, &tspec);
    return tspec;
}
