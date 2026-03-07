#ifndef ARTY_SERVER_H
#define ARTY_SERVER_H

#include <string>

struct daq_client_status {
    bool connected;
    int64_t last_pinged_ms;
};

void serve_connections();
daq_client_status get_daq_client_status();

#endif  // ARTY_SERVER_H
