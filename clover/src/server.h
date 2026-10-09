// Declares server startup and DAQ client connection status.
#ifndef ARTY_SERVER_H
#define ARTY_SERVER_H

#include <string>

struct DaqClientStatus {
    bool connected;
    int64_t last_pinged_ms;
};

/// Starts command and data server threads.
/// Parameters: None.
/// Returns: nothing.
void serve_connections();

/// Gets the current DAQ client connection and last-ping status.
/// Parameters: None.
/// Returns: connected status and elapsed milliseconds since the last ping.
DaqClientStatus get_daq_client_status();

#endif  // ARTY_SERVER_H
