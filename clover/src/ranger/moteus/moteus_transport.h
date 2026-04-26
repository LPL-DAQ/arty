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
    int recv_timeout_ms = 10;
    // Maximum number of simultaneous reply filters (one per motor in a batch).
    int max_replies = 4;
  };

  explicit ZephyrCanTransport(const Options& opts) : opts_(opts) {}

  void Cycle(const CanFdFrame* frames,
             size_t size,
             std::vector<CanFdFrame>* replies,
             CompletionCallback callback) override {
    if (replies) replies->clear();

    // Allocate the message queue on the stack.
    // can_frame is ~72 bytes; 4 frames = ~288 bytes — acceptable.
    alignas(4) struct can_frame q_buf[4];
    struct k_msgq q;
    const int q_depth = (opts_.max_replies <= 4) ? opts_.max_replies : 4;
    k_msgq_init(&q, reinterpret_cast<char*>(q_buf),
                sizeof(struct can_frame), q_depth);

    // Install RX filters before sending so we don't miss the reply.
    int filter_ids[4];
    int n_filters = 0;

    for (size_t i = 0; i < size && n_filters < q_depth; i++) {
      if (!frames[i].reply_required) continue;

      // Reply arbitration ID: (motor_id << 8) | host_source
      // frames[i].destination = motor_id, frames[i].source = host (0)
      struct can_filter f = {};
      f.flags = CAN_FILTER_IDE;
      f.id    = ((uint32_t)frames[i].destination << 8) | frames[i].source;
      f.mask  = CAN_EXT_ID_MASK;

      int fid = can_add_rx_filter_msgq(opts_.can_dev, &q, &f);
      if (fid >= 0) {
        filter_ids[n_filters++] = fid;
      }
    }

    // Send all frames.
    for (size_t i = 0; i < size; i++) {
      send_can_frame(frames[i]);
    }

    // Collect reply frames.
    for (int i = 0; i < n_filters && replies; i++) {
      struct can_frame rx = {};
      if (k_msgq_get(&q, &rx, K_MSEC(opts_.recv_timeout_ms)) != 0) {
        break;  // timeout — stop waiting
      }

      CanFdFrame reply = {};
      reply.arbitration_id = rx.id & CAN_EXT_ID_MASK;
      reply.source         = (reply.arbitration_id >> 8) & 0x7F;
      reply.destination    =  reply.arbitration_id       & 0x7F;
      reply.can_prefix     =  reply.arbitration_id >> 16;
      reply.size           = can_dlc_to_bytes(rx.dlc);
      memcpy(reply.data, rx.data, reply.size);
      replies->push_back(reply);
    }

    for (int i = 0; i < n_filters; i++) {
      can_remove_rx_filter(opts_.can_dev, filter_ids[i]);
    }

    callback(0);
  }

  void Post(std::function<void()> fn) override { fn(); }

 private:
  void send_can_frame(const CanFdFrame& frame) {
    struct can_frame zf = {};
    zf.flags = CAN_FRAME_IDE;

    // Use CAN-FD unless the frame explicitly forces classic CAN.
    if (frame.fdcan_frame != CanFdFrame::kForceOff) {
      zf.flags |= CAN_FRAME_FDF;
      if (frame.brs != CanFdFrame::kForceOff) {
        zf.flags |= CAN_FRAME_BRS;
      }
    }

    zf.id  = frame.arbitration_id;
    zf.dlc = can_bytes_to_dlc(frame.size);
    memcpy(zf.data, frame.data, frame.size);

    can_send(opts_.can_dev, &zf, K_MSEC(opts_.send_timeout_ms),
             nullptr, nullptr);
  }

  Options opts_;
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
