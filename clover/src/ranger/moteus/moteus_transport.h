// Zephyr-specific transport for the mjbots moteus C++ library.
//
// Replaces the upstream moteus_transport.h (which requires Linux-only headers
// such as <linux/can.h>, <net/if.h>, <thread>, <condition_variable>) with a
// transport that uses Zephyr's CAN driver API directly.
//
// The public interface (CompletionCallback, BlockingCallback, Transport) is
// identical to the upstream file so that moteus.h compiles unchanged.

#pragma once

#include <functional>
#include <memory>
#include <string>
#include <vector>

#include <zephyr/drivers/can.h>
#include <zephyr/kernel.h>

#include "moteus_protocol.h"
#include "moteus_tokenizer.h"

// ---- Zephyr compatibility shims ----
#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

// moteus.h calls ::usleep() in diagnostic retry loops — map to Zephyr.
#ifndef usleep
#define usleep(us) k_usleep(us)
#endif

namespace mjbots {
namespace moteus {

using CompletionCallback = std::function<void(int /* errno */)>;

// Simplified BlockingCallback for synchronous transports.
// ZephyrCanTransport calls callback() before returning from Cycle(), so
// Wait() always finds the result already set — no condition variable needed.
class BlockingCallback {
 public:
  CompletionCallback callback() {
    return [this](int v) {
      result_ = v;
    };
  }

  int Wait() { return result_; }

 private:
  int result_ = 0;
};

class Transport {
 public:
  virtual ~Transport() {}

  virtual void Cycle(const CanFdFrame* frames,
                     size_t size,
                     std::vector<CanFdFrame>* replies,
                     CompletionCallback completed_callback) = 0;

  virtual void BlockingCycle(const CanFdFrame* frames,
                             size_t size,
                             std::vector<CanFdFrame>* replies) {
    BlockingCallback cbk;
    this->Cycle(frames, size, replies, cbk.callback());
    cbk.Wait();
  }

  virtual void Post(std::function<void()> callback) = 0;
};

// ---------------------------------------------------------------------------
// Zephyr CAN-FD transport
// ---------------------------------------------------------------------------

class ZephyrCanTransport : public Transport {
 public:
  struct Options {
    // CAN device obtained via DEVICE_DT_GET(DT_CHOSEN(zephyr_canbus)).
    const struct device *can_dev = nullptr;
    // Timeout waiting for the TX mailbox to become free.
    int send_timeout_ms = 10;
    // Timeout waiting for a reply frame after sending.
    int recv_timeout_ms = 100;
    // Depth of the persistent RX queue (absorbs bursts between Cycle() calls).
    int rx_queue_depth = 16;
    // Enable CAN-FD Bit Rate Switching (faster data phase). Disable if the
    // motor firmware or transceiver is not configured for the higher data rate.
    bool brs_enabled = true;
  };

  explicit ZephyrCanTransport(const Options& opts) : opts_(opts) {
    // Install a single catch-all extended-ID filter that stays active for the
    // lifetime of the transport — exactly like ACAN_T4's continuous reception.
    // No per-cycle filter add/remove means no FlexCAN Freeze Mode overhead
    // between TX and the motor's reply.
    k_msgq_init(&rx_queue_,
                reinterpret_cast<char*>(rx_queue_buf_),
                sizeof(struct can_frame),
                kRxQueueDepth);

    struct can_filter f = {};
    f.flags = CAN_FILTER_IDE;
    f.id    = 0;
    f.mask  = 0;
    rx_filter_id_ = can_add_rx_filter_msgq(opts_.can_dev, &rx_queue_, &f);
    if (rx_filter_id_ < 0) {
      printk("[transport] ERROR: can_add_rx_filter_msgq failed (%d)\n",
             rx_filter_id_);
    } else {
      printk("[transport] persistent catch-all filter installed (id=%d)\n",
             rx_filter_id_);
    }
  }

  ~ZephyrCanTransport() {
    if (rx_filter_id_ >= 0) {
      can_remove_rx_filter(opts_.can_dev, rx_filter_id_);
    }
  }

  void Cycle(const CanFdFrame* frames,
             size_t size,
             std::vector<CanFdFrame>* replies,
             CompletionCallback callback) override {
    if (replies) replies->clear();

    // Count how many reply frames we expect.
    int n_expected = 0;
    for (size_t i = 0; i < size; i++) {
      if (frames[i].reply_required) n_expected++;
    }

    // Drain any stale frames that arrived before this Cycle() call.
    {
      struct can_frame stale = {};
      while (k_msgq_get(&rx_queue_, &stale, K_NO_WAIT) == 0) {}
    }

    // Send all outgoing frames.
    for (size_t i = 0; i < size; i++) {
      send_can_frame(frames[i]);
    }

    if (!replies || n_expected == 0) {
      callback(0);
      return;
    }

    // Collect reply frames — the filter is already active so we just wait.
    for (int i = 0; i < n_expected; i++) {
      struct can_frame rx = {};
      if (k_msgq_get(&rx_queue_, &rx, K_MSEC(opts_.recv_timeout_ms)) != 0) {
        printk("[transport] reply timeout after %d ms\n", opts_.recv_timeout_ms);
        break;
      }

      const uint32_t arb_id = rx.id & CAN_EXT_ID_MASK;
      printk("[transport] rx id=0x%08X prefix=%u src=%u dst=%u len=%u\n",
             (unsigned)arb_id,
             (unsigned)((arb_id >> 16) & 0x1FFF),
             (unsigned)((arb_id >> 8) & 0x7F),
             (unsigned)(arb_id & 0xFF),
             can_dlc_to_bytes(rx.dlc));

      CanFdFrame reply = {};
      reply.arbitration_id = arb_id;
      reply.source         = (arb_id >> 8) & 0x7F;
      reply.destination    =  arb_id       & 0xFF;
      reply.can_prefix     = (arb_id >> 16) & 0x1FFF;
      reply.size           = can_dlc_to_bytes(rx.dlc);
      memcpy(reply.data, rx.data, reply.size);
      replies->push_back(reply);
    }

    callback(0);
  }

  void Post(std::function<void()> fn) override { fn(); }

 private:
  static constexpr int kRxQueueDepth = 16;

  void send_can_frame(const CanFdFrame& frame) {
    struct can_frame zf = {};
    zf.flags = CAN_FRAME_IDE;

    if (frame.fdcan_frame != CanFdFrame::kForceOff) {
      zf.flags |= CAN_FRAME_FDF;
      if (opts_.brs_enabled && frame.brs != CanFdFrame::kForceOff) {
        zf.flags |= CAN_FRAME_BRS;
      }
    }

    zf.id  = frame.arbitration_id;
    zf.dlc = can_bytes_to_dlc(frame.size);
    memcpy(zf.data, frame.data, frame.size);

    int ret = can_send(opts_.can_dev, &zf, K_MSEC(opts_.send_timeout_ms),
                       nullptr, nullptr);
    if (ret != 0) {
      printk("[transport] can_send failed: %d (id=0x%08X)\n",
             ret, (unsigned)zf.id);
    }
  }

  Options opts_;
  int rx_filter_id_ = -1;

  // Queue buffer — allocated here so it has static lifetime (required by
  // k_msgq_init when not using K_MSGQ_DEFINE).
  alignas(4) struct can_frame rx_queue_buf_[kRxQueueDepth];
  struct k_msgq rx_queue_;
};

// ---------------------------------------------------------------------------
// Stubs required by moteus.h — not used when transport is set explicitly
// ---------------------------------------------------------------------------

// moteus.h calls this in the Controller constructor when opts.transport is
// null. On Zephyr, always provide opts.transport explicitly — this should
// never be reached.
inline std::shared_ptr<Transport> MakeSingletonTransport(
    const std::vector<std::string>&) {
  abort();
  return {};
}

// Fdcanusb stub — moteus.h calls Fdcanusb::GetNow() for diagnostic timeouts.
// On Zephyr we use k_uptime_ticks() converted to nanoseconds.
class Fdcanusb : public Transport {
 public:
  struct Options {};
  Fdcanusb(const Options& = {}) {}
  void Cycle(const CanFdFrame*, size_t, std::vector<CanFdFrame>*,
             CompletionCallback cb) override { cb(0); }
  void Post(std::function<void()> fn) override { fn(); }

  // Returns current time in nanoseconds — used by moteus.h diagnostic code.
  static int64_t GetNow() {
    return (int64_t)k_uptime_get() * 1000000LL;
  }
};

// Stubs for TransportFactory and TransportRegistry so that
// moteus.h's DefaultArgProcess() and cmdline_arguments() compile without
// pulling in Linux-specific transport implementations.

class TransportFactory {
 public:
  struct Argument {
    std::string name;
    int nargs = 1;
    std::string help;
    Argument(const std::string& n, int na, const std::string& h)
        : name(n), nargs(na), help(h) {}
  };
  using TransportArgPair =
      std::pair<std::shared_ptr<Transport>, std::vector<std::string>>;

  virtual ~TransportFactory() {}
  virtual std::string name() = 0;
  virtual std::vector<Argument> cmdline_arguments() = 0;
  virtual bool is_args_set(const std::vector<std::string>&) = 0;
  virtual TransportArgPair make(const std::vector<std::string>&) = 0;
};

class TransportRegistry {
 public:
  template <typename T>
  void Register() {}

  static TransportRegistry& singleton() {
    static TransportRegistry r;
    return r;
  }

  std::vector<TransportFactory::Argument> cmdline_arguments() { return {}; }

  TransportFactory::TransportArgPair make(const std::vector<std::string>& args) {
    return {MakeSingletonTransport({}), args};
  }
};

}  // namespace moteus
}  // namespace mjbots
