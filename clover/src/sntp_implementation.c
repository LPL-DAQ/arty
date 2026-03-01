#include <zephyr/kernel.h>
#include <zephyr/net/net_if.h>
#include <zephyr/net/net_core.h>
#include <zephyr/net/net_context.h>
#include <zephyr/net/net_mgmt.h>
#include <zephyr/net/sntp.h>
#include <zephyr/logging/log.h>
#include <zephyr/posix/time.h>

LOG_MODULE_REGISTER(hotfire_sntp_cpp, LOG_LEVEL_INF);

class NetworkTimeSync {
private:
    /* Static flag so the static callback can access it */
    static bool network_is_up;

    struct net_mgmt_event_callback mgmt_cb;
    const char* sntp_server;
    uint32_t timeout_ms;

    /* * The callback MUST be static. Zephyr's C-based API cannot call
     * a standard C++ member function because it doesn't know about 'this'.
     */
    static void net_event_handler(struct net_mgmt_event_callback *cb,
                                  uint32_t mgmt_event, struct net_if *iface) {
        if (mgmt_event == NET_EVENT_IPV4_DHCP_BOUND) {
            LOG_INF("Network connected and DHCP bound!");
            network_is_up = true;
        }
    }

public:
    // Constructor allows you to inject different servers/timeouts if needed
    NetworkTimeSync(const char* server = "pool.ntp.org", uint32_t timeout = 5000)
        : sntp_server(server), timeout_ms(timeout) {}

    void init() {
        LOG_INF("Initializing SNTP sync");

        // Register for DHCP events
        net_mgmt_init_event_callback(&mgmt_cb, net_event_handler, NET_EVENT_IPV4_DHCP_BOUND);
        net_mgmt_add_event_callback(&mgmt_cb);
    }

    void waitForNetwork() {
        LOG_INF("Waiting for DHCP");
        while (!network_is_up) {
            k_sleep(K_MSEC(500));
        }
    }

    bool syncTime() {
        struct sntp_time ts;
        struct timespec tspec;

        LOG_INF("Sending SNTP request to %s...", sntp_server);
        int res = sntp_simple(sntp_server, timeout_ms, &ts);

        if (res == 0) {
            LOG_INF("SNTP time received. Epoch seconds: %llu", ts.seconds);

            // Convert SNTP fraction to POSIX nanoseconds
            tspec.tv_sec = ts.seconds;
            tspec.tv_nsec = (uint32_t)(((uint64_t)ts.fraction * 1000000000ULL) >> 32);

            // Set the Zephyr POSIX clock
            clock_settime(CLOCK_REALTIME, &tspec);
            LOG_INF("System clock updated");
            return true;
        } else {
            LOG_ERR("Failed to get SNTP time. Error code: %d", res);
            return false;
        }
    }
};

// Allocate memory for the static class member
bool NetworkTimeSync::network_is_up = false;


int main(void) {
    // Instantiate our new C++ object
    NetworkTimeSync timeSync;

    timeSync.init();
    timeSync.waitForNetwork();

    // Robustness: Added a retry loop in case the network drops packets
    // right before the hotfire sequence begins.
    while (!timeSync.syncTime()) {
        LOG_WRN("Retrying SNTP sync in 2 seconds...");
        k_sleep(K_SECONDS(2));
    }

    struct timespec tspec;

    /* Main Engine DAQ loop */
    while (1) {
        clock_gettime(CLOCK_REALTIME, &tspec);
        LOG_INF("Current Engine DAQ timestamp: %lld.%09ld",
                (long long)tspec.tv_sec, tspec.tv_nsec);
        k_sleep(K_SECONDS(1));
    }

    return 0;
}
