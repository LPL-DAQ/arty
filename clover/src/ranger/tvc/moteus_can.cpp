#include "moteus_can.h"
#include "moteus.h"

#include <cstring>
#include <zephyr/device.h>
#include <zephyr/drivers/can.h>
#include <zephyr/logging/log.h>
#include <zephyr/sys/atomic.h>

LOG_MODULE_REGISTER(moteus_can, CONFIG_LOG_DEFAULT_LEVEL);

#if !DT_HAS_CHOSEN(zephyr_canbus)
#error "CONFIG_RANGER_TVC needs a zephyr,canbus chosen node (the moteus CAN-FD bus)"
#endif
#ifndef CONFIG_CAN_FD_MODE
#error "CONFIG_RANGER_TVC needs CONFIG_CAN_FD_MODE=y (moteus speaks CAN-FD)"
#endif

namespace {

const device* const can_dev = DEVICE_DT_GET(DT_CHOSEN(zephyr_canbus));

// Depth 4: a reply per cycle plus slack for a late one; flushed before every send.
CAN_MSGQ_DEFINE(rx_queue_0, 4);
CAN_MSGQ_DEFINE(rx_queue_1, 4);
k_msgq* const rx_queues[moteus_can::MAX_DEVICES] = {&rx_queue_0, &rx_queue_1};

uint8_t ids[moteus_can::MAX_DEVICES];
uint16_t can_prefix;
uint8_t our_source_id;
bool use_brs;

atomic_t tx_error_count = ATOMIC_INIT(0);
atomic_t tx_rejected_count = ATOMIC_INIT(0);

/// TX completion, ISR context. A missing ACK never completes (the controller keeps retrying), so errors here are rare;
/// the real liveness signal is the reply.
void tx_done(const device*, int error, void*)
{
    if (error != 0) {
        atomic_inc(&tx_error_count);
    }
}

}  // namespace

std::expected<void, Error> moteus_can::init(const uint8_t (&moteus_ids)[MAX_DEVICES], uint16_t prefix, uint8_t source_id, bool brs)
{
    if (!device_is_ready(can_dev)) {
        return std::unexpected(Error::from_device_not_ready(can_dev));
    }

    std::memcpy(ids, moteus_ids, sizeof(ids));
    can_prefix = prefix;
    our_source_id = source_id;
    use_brs = brs;

    // Mode can only change while stopped. -EALREADY just means it was not running.
    if (int err = can_stop(can_dev); err != 0 && err != -EALREADY) {
        return std::unexpected(Error::from_code(err).context("failed to stop CAN controller"));
    }
    if (int err = can_set_mode(can_dev, CAN_MODE_FD); err != 0) {
        return std::unexpected(Error::from_code(err).context("failed to set CAN-FD mode"));
    }

    // Replies come from (source = moteus ID) to (destination = us), with the reply-request bit clear. Exact match on
    // all 29 bits so each queue only ever sees its own controller.
    for (int i = 0; i < MAX_DEVICES; i++) {
        can_filter filter = {
            .id = moteus::arbitration_id(can_prefix, ids[i], our_source_id, false),
            .mask = CAN_EXT_ID_MASK,
            .flags = CAN_FILTER_IDE,
        };
        int filter_id = can_add_rx_filter_msgq(can_dev, rx_queues[i], &filter);
        if (filter_id < 0) {
            return std::unexpected(Error::from_code(filter_id).context("failed to add CAN filter for moteus %d", ids[i]));
        }
    }

    if (int err = can_start(can_dev); err != 0) {
        return std::unexpected(Error::from_code(err).context("failed to start CAN controller"));
    }
    return {};
}

void moteus_can::flush(int device)
{
    k_msgq_purge(rx_queues[device]);
}

bool moteus_can::send(int device, const uint8_t* data, size_t len, bool reply_required)
{
    can_frame frame = {};
    frame.id = moteus::arbitration_id(can_prefix, our_source_id, ids[device], reply_required);
    frame.flags = CAN_FRAME_IDE | CAN_FRAME_FDF | (use_brs ? CAN_FRAME_BRS : 0);
    frame.dlc = can_bytes_to_dlc(static_cast<uint8_t>(len));
    std::memcpy(frame.data, data, len);

    // Never block on the bus: with a callback can_send returns as soon as the frame is queued in a mailbox.
    int err = can_send(can_dev, &frame, K_NO_WAIT, tx_done, nullptr);
    if (err != 0) {
        atomic_inc(&tx_rejected_count);
        return false;
    }
    return true;
}

bool moteus_can::receive(int device, Reply& out, k_timepoint_t deadline)
{
    can_frame frame;
    if (k_msgq_get(rx_queues[device], &frame, sys_timepoint_timeout(deadline)) != 0) {
        return false;
    }
    out.len = can_dlc_to_bytes(frame.dlc);
    std::memcpy(out.data, frame.data, out.len);
    return true;
}

uint32_t moteus_can::tx_errors()
{
    return static_cast<uint32_t>(atomic_get(&tx_error_count));
}

uint32_t moteus_can::tx_rejected()
{
    return static_cast<uint32_t>(atomic_get(&tx_rejected_count));
}
