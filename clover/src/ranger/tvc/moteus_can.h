#pragma once

// Zephyr CAN-FD transport for the TVC moteus controllers. Thin layer over the native Zephyr CAN API (FlexCAN3, chosen
// as zephyr,canbus): one hardware RX filter + message queue per controller, so replies are demultiplexed by the driver
// in ISR context and never allocate. Pattern adapted from prabhu/moteusTest's ZephyrCanTransport, minus the heap and
// with non-blocking transmit.
//
// Only the TVC thread calls into this module, so it needs no locking.

#include "Error.h"

#include <cstddef>
#include <cstdint>
#include <expected>
#include <zephyr/kernel.h>

namespace moteus_can {

constexpr int MAX_DEVICES = 2;

struct Reply {
    uint8_t data[64];
    uint8_t len;
};

/// Put the controller in CAN-FD mode, start it, and install one reply filter per device. Call once, from the TVC
/// thread, before anything else.
std::expected<void, Error> init(const uint8_t (&moteus_ids)[MAX_DEVICES], uint16_t prefix, uint8_t source_id, bool brs);

/// Drop any replies still queued for a device (late replies from a previous cycle).
void flush(int device);

/// Queue a frame to device `device` without blocking. Returns false if no TX mailbox was free or the controller is
/// stopped / bus-off; the caller treats that as a missed reply.
bool send(int device, const uint8_t* data, size_t len, bool reply_required);

/// Wait until `deadline` for the next reply from `device`.
bool receive(int device, Reply& out, k_timepoint_t deadline);

/// Diagnostics.
uint32_t tx_errors();
uint32_t tx_rejected();

}  // namespace moteus_can
