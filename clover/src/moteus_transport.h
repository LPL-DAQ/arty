// Copyright 2023 mjbots Robotic Systems, LLC.  info@mjbots.com
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#pragma once

#include <fcntl.h>
#include <limits.h>
#include <poll.h>
#include <stdarg.h>
#include <sys/socket.h>
#include <sys/types.h>
#include <sys/stat.h>
#include <unistd.h>

#include <algorithm>
#include <atomic>
#include <functional>
#include <fstream>
#include <memory>
#include <set>
#include <stdexcept>
#include <string>
#include <vector>

// Zephyr specific networking and kernel headers
#include <zephyr/kernel.h>
#include <zephyr/drivers/can.h>
#include <zephyr/net/socket.h>
#include <zephyr/net/socketcan.h>
#include <zephyr/net/net_if.h>

#include "moteus_protocol.h"
#include "moteus_tokenizer.h"

namespace mjbots {
namespace moteus {

using CompletionCallback = std::function<void(int /* errno */)>;

/// RTOS Patch: Simplified synchronous callback wrapper
class BlockingCallback {
 public:
  CompletionCallback callback() {
    return [&](int v) {
      result_ = v;
    };
  }

  int Wait() {
    return result_;
  }

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

namespace details {
class FileDescriptor {
 public:
  FileDescriptor() {}
  FileDescriptor(int fd) { fd_ = fd; }
  ~FileDescriptor() {
    if (fd_ >= 0) { ::close(fd_); }
  }

  FileDescriptor& operator=(int fd) {
    if (fd_ >= 0) { ::close(fd_); }
    fd_ = fd;
    return *this;
  }

  bool operator==(const FileDescriptor& rhs) const {
    return fd_ == rhs.fd_;
  }

  operator int() const {
    return fd_;
  }

  int release() {
    const auto result = fd_;
    fd_ = -1;
    return result;
  }

 private:
  int fd_ = -1;
};

/// RTOS Patch: Bypasses threading overhead. Executes callbacks inline.
class ThreadedEventLoop {
 public:
  ThreadedEventLoop() {}
  ~ThreadedEventLoop() {}

  void Post(std::function<void()> callback) {
    callback();
  }
};

class TimeoutTransport : public Transport {
 public:
  struct Options {
    bool disable_brs = false;
    uint32_t min_ok_wait_ns = 2000000;
    uint32_t min_rcv_wait_ns = 50000000;
    uint32_t rx_extra_wait_ns = 50000000;
    uint32_t final_wait_ns = 50000;
    int max_pipeline = -1;

    Options() {}
  };

  TimeoutTransport(const Options& options) : t_options_(options) {}

  virtual void Cycle(const CanFdFrame* frames,
                     size_t size,
                     std::vector<CanFdFrame>* replies,
                     CompletionCallback completed_callback) override {
    auto copy = std::atomic_load(&UNPROTECTED_event_loop_);
    FailIf(!copy, "unexpected null event loop");
    copy->Post(
        std::bind(&TimeoutTransport::CHILD_Cycle,
                  this, frames, size, replies, completed_callback));
  }

  virtual void Post(std::function<void()> callback) override {
    auto copy = std::atomic_load(&UNPROTECTED_event_loop_);
    if (copy) {
      copy->Post(callback);
    }
  }

  static int64_t GetNow() {
    struct timespec ts = {};
    ::clock_gettime(CLOCK_MONOTONIC, &ts); // RTOS Patch: Removed _RAW
    return static_cast<int64_t>(ts.tv_sec) * 1000000000ll +
      static_cast<int64_t>(ts.tv_nsec);
  }

  static void Fail(const std::string& message) {
    throw std::runtime_error(message);
  }

  static void FailIf(bool terminate, const std::string& message) {
    if (terminate) { Fail(message); }
  }

  static void FailIfErrno(bool terminate) {
    if (terminate) { Fail(::strerror(errno)); }
  }

 protected:
  virtual int CHILD_GetReadFd() const = 0;
  virtual void CHILD_SendCanFdFrame(const CanFdFrame&) = 0;

  struct ConsumeCount {
    int rcv = 0;
    int ok = 0;
  };

  virtual ConsumeCount CHILD_ConsumeData(
      std::vector<CanFdFrame>* replies,
      int expected_ok_count,
      std::vector<int>* expected_reply_count) = 0;
  virtual void CHILD_FlushTransmit() = 0;

  void CHILD_Cycle(const CanFdFrame* frames,
                   size_t size,
                   std::vector<CanFdFrame>* replies,
                   CompletionCallback completed_callback) {
    if (replies) { replies->clear(); }
    CHILD_CheckReplies(replies, kFlush, 0, nullptr);

    const auto advance = t_options_.max_pipeline < 0 ?
        size : t_options_.max_pipeline;

    for (size_t start = 0; start < size; start += advance) {
      int expected_ok_count = 0;
      for (auto& v : expected_reply_count_) { v = 0; }

      for (size_t i = start; i < (start + advance) && i < size; i++) {
        expected_ok_count++;
        CHILD_SendCanFdFrame(frames[i]);
        if (frames[i].reply_required) {
          if ((frames[i].destination + 1) >
              static_cast<int>(expected_reply_count_.size())) {
            expected_reply_count_.resize(frames[i].destination + 1);
          }
          expected_reply_count_[frames[i].destination]++;
        }
      }

      CHILD_FlushTransmit();

      CHILD_CheckReplies(replies,
                         kWait,
                         expected_ok_count,
                         &expected_reply_count_);
    }

    Post(std::bind(completed_callback, 0));
  }

  enum ReadDelay {
    kWait,
    kFlush,
  };

  void CHILD_CheckReplies(std::vector<CanFdFrame>* replies,
                          ReadDelay read_delay,
                          int expected_ok_count,
                          std::vector<int>* expected_reply_count) {
    const auto start = GetNow();

    const auto any_reply_checker = [&]() {
      if (!expected_reply_count) { return false; }
      for (auto v : *expected_reply_count) {
        if (v) { return true; }
      }
      return false;
    };
    auto end_time =
        start +
        (read_delay == kWait ?
         std::max(expected_ok_count != 0 ? t_options_.min_ok_wait_ns : 0,
                  any_reply_checker() ? t_options_.min_rcv_wait_ns : 0) :
         t_options_.final_wait_ns);

    struct pollfd fds[1] = {};
    fds[0].fd = CHILD_GetReadFd();
    fds[0].events = POLLIN;

    int ok_count = 0;

    while (true) {
      const auto now = GetNow();
      fds[0].revents = 0;

      const auto to_sleep_ns = std::max<int64_t>(0, end_time - now);

      const int poll_ret = ::poll(&fds[0], 1, static_cast<int>(to_sleep_ns / 1000000));

      if (poll_ret < 0) {
        if (errno == EINTR) { continue; }
        FailIfErrno(true);
      }
      if (poll_ret == 0) { return; }

      const auto consume_count = CHILD_ConsumeData(
          replies, expected_ok_count, expected_reply_count);

      ok_count += consume_count.ok;

      if (read_delay != kFlush &&
          !any_reply_checker() && ok_count >= expected_ok_count) {
        return;
      }

      if (consume_count.rcv || consume_count.ok) {
        const auto finish_time = GetNow();
        end_time = finish_time + t_options_.rx_extra_wait_ns;
      }
    }
  }

  std::shared_ptr<details::ThreadedEventLoop> UNPROTECTED_event_loop_ =
      std::make_shared<details::ThreadedEventLoop>();

 private:
  const Options t_options_;
  std::vector<int> expected_reply_count_;
};
}

class Socketcan : public details::TimeoutTransport {
 public:
  struct Options : details::TimeoutTransport::Options {
    std::string ifname = "can0";
    bool ignore_errors = false;
    Options() {}
  };

  Socketcan(const Options& options)
      : details::TimeoutTransport(options),
        options_(options) {
    socket_ = Open(options_.ifname);
  }

  virtual ~Socketcan() {
    std::atomic_store(&UNPROTECTED_event_loop_, {});
  }

 private:
  static void SetNonblock(int fd) {
    int flags = ::fcntl(fd, F_GETFL, 0);
    FailIf(flags < 0, "error getting flags");
    flags |= O_NONBLOCK;
    FailIf(::fcntl(fd, F_SETFL, flags), "error setting flags");
  }

  static int Open(const std::string& ifname) {
    int fd = socket(AF_CAN, SOCK_RAW, CAN_RAW);
    FailIf(fd < 0, "error opening CAN socket");

    SetNonblock(fd);

    struct net_if *iface = net_if_get_first_by_type(&NET_L2_GET_NAME(CANBUS_RAW));
    FailIf(!iface, "could not find CAN interface");

    struct sockaddr_can addr = {};
    addr.can_family = AF_CAN;
    addr.can_ifindex = net_if_get_by_iface(iface);

    FailIf(bind(fd, (struct sockaddr *)&addr, sizeof(addr)) < 0,
           "could not bind to CAN if");

    return fd;
  }

  virtual int CHILD_GetReadFd() const override {
    return socket_;
  }

  virtual void CHILD_SendCanFdFrame(const CanFdFrame& frame) override {
    struct can_frame send_frame = {};
    
    send_frame.id = frame.arbitration_id;
    if (send_frame.id >= 0x7ff) {
      #ifndef CAN_EFF_FLAG
      #define CAN_EFF_FLAG 0x80000000U
      #endif
      send_frame.id |= CAN_EFF_FLAG;
    }
    
    send_frame.dlc = std::min<uint8_t>(frame.size, 8);
    std::memcpy(send_frame.data, frame.data, send_frame.dlc);

    const auto write_result = ::write(socket_, &send_frame, sizeof(send_frame));
    if (!options_.ignore_errors) {
      FailIf(write_result < 0, "error writing CAN");
    }
  }

  virtual ConsumeCount CHILD_ConsumeData(
      std::vector<CanFdFrame>* replies,
      int /* expected_ok_count */,
      std::vector<int>* expected_reply_count) override {
      
    struct can_frame recv_frame = {};
    FailIf(::read(socket_, &recv_frame, sizeof(recv_frame)) < 0,
           "error reading CAN frame");

    CanFdFrame this_frame;
    this_frame.arbitration_id = recv_frame.id & 0x1fffffff;
    this_frame.destination = this_frame.arbitration_id & 0x7f;
    this_frame.source = (this_frame.arbitration_id >> 8) & 0x7f;
    this_frame.can_prefix = (this_frame.arbitration_id >> 16);

    this_frame.brs = CanFdFrame::kForceOff;
    this_frame.fdcan_frame = CanFdFrame::kForceOff;
    
    // RTOS Patch: Swapped can_dlc -> dlc
    this_frame.size = recv_frame.dlc;
    std::memcpy(this_frame.data, recv_frame.data, recv_frame.dlc);

    if (expected_reply_count) {
      if (this_frame.source <
          static_cast<int>(expected_reply_count->size())) {
        (*expected_reply_count)[this_frame.source] = std::max(
            (*expected_reply_count)[this_frame.source] - 1, 0);
      }
    }

    if (replies) {
      replies->emplace_back(std::move(this_frame));
    }

    ConsumeCount result;
    result.ok = 1;
    result.rcv = 1;
    return result;
  }

  virtual void CHILD_FlushTransmit() override {}

  const Options options_;
  details::FileDescriptor socket_;
};

class TransportFactory {
 public:
  virtual ~TransportFactory() {}

  virtual int priority() = 0;
  virtual std::string name() = 0;
  using TransportArgPair = std::pair<std::shared_ptr<Transport>,
                                     std::vector<std::string>>;
  virtual TransportArgPair make(const std::vector<std::string>&) = 0;

  struct Argument {
    std::string name;
    int nargs = 1;
    std::string help;

    bool operator<(const Argument& rhs) const {
      if (name < rhs.name) { return true; }
      if (name > rhs.name) { return false; }
      return help < rhs.help;
    }

    Argument(const std::string& name_in,
             int nargs_in,
             const std::string& help_in)
        : name(name_in),
          nargs(nargs_in),
          help(help_in) {}
  };

  virtual std::vector<Argument> cmdline_arguments() = 0;
  virtual bool is_args_set(const std::vector<std::string>&) = 0;
};

class SocketcanFactory : public TransportFactory {
 public:
  virtual ~SocketcanFactory() {}

  virtual int priority() override { return 11; }
  virtual std::string name() override { return "socketcan"; }

  virtual TransportArgPair make(const std::vector<std::string>& args_in) override {
    auto args = args_in;

    Socketcan::Options options;

    {
      auto it = std::find(args.begin(), args.end(), "--can-disable-brs");
      if (it != args.end()) {
        options.disable_brs = true;
        args.erase(it);
      }
    }

    {
      auto it = std::find(args.begin(), args.end(), "--socketcan-iface");
      if (it != args.end()) {
        if ((it + 1) != args.end()) {
          options.ifname = *(it + 1);
          args.erase(it, it + 2);
        } else {
          throw std::runtime_error("--socketcan-iface requires an interface name");
        }
      }
    }
    {
      auto it = std::find(args.begin(), args.end(), "--socketcan-ignore-errors");
      if (it != args.end()) {
        options.ignore_errors = true;
        args.erase(it);
      }
    }

    auto result = std::make_shared<Socketcan>(options);
    return TransportArgPair(result, args);
  }

  virtual std::vector<Argument> cmdline_arguments() override {
    return {
      { "--socketcan-iface", 1, "socketcan iface name" },
      { "--socketcan-ignore-errors", 0, "ignore errors sending socketcan frames" },
      { "--can-disable-brs", 0, "do not set BRS" },
    };
  }

  virtual bool is_args_set(const std::vector<std::string>& args) override {
    for (const auto& arg : args) {
      if (arg == "--socketcan-iface") { return true; }
      if (arg == "--socketcan-ignore-errors") { return true; }
    }
    return false;
  }
};

class TransportRegistry {
 public:
  template <typename T>
  void Register() {
    items_.push_back(std::make_shared<T>());
  }

  static TransportRegistry& singleton() {
    static TransportRegistry reg;
    return reg;
  }

  std::vector<TransportFactory::Argument> cmdline_arguments() const {
    std::vector<TransportFactory::Argument> result;
    std::set<TransportFactory::Argument> uniqifier;

    result.push_back(TransportFactory::Argument(
                         "--force-transport", 1,
                         "force the given transport type to be used"));
    uniqifier.insert(result.back());

    for (const auto& item : items_) {
      const auto item_args = item->cmdline_arguments();
      for (const auto& arg : item_args) {
        if (uniqifier.count(arg) == 0) {
          result.push_back(arg);
          uniqifier.insert(arg);
        }
      }
    }

    return result;
  }

  TransportFactory::TransportArgPair make(const std::vector<std::string>& args_in) const {
    auto args = args_in;
    auto to_try = items_;

    std::sort(to_try.begin(), to_try.end(),
              [](const std::shared_ptr<TransportFactory>& lhs,
                 const std::shared_ptr<TransportFactory>& rhs) {
                return lhs->priority() < rhs->priority();
              });

    const auto it = std::find(args.begin(), args.end(), "--force-transport");
    if (it != args.end()) {
      if ((it + 1) != args.end()) {
        to_try = {};
        const auto name_to_find = *(it + 1);
        for (auto item : items_) {
          if (item->name() == name_to_find) { to_try.push_back(item); }
        }
        args.erase(it, it + 2);
      } else {
        throw std::runtime_error("--force-transport requires an argument");
      }
    } else {
      std::vector<std::shared_ptr<TransportFactory>> options_set;
      for (auto item : items_) {
        if (item->is_args_set(args)) {
          options_set.push_back(item);
        }
      }

      if (!options_set.empty()) { to_try = options_set; }
    }

    std::string errors;
    for (auto factory : to_try) {
      try {
        auto maybe_result = factory->make(args);
        if (maybe_result.first) {
          return maybe_result;
        }
      } catch (std::runtime_error& re) {
        if (!errors.empty()) { errors += ", "; }
        errors += factory->name() + ": " + re.what();
      }
    }
    throw std::runtime_error("Unable to find a default transport: " + errors);
  }

 private:
  TransportRegistry() {
    Register<SocketcanFactory>();
  }

  std::vector<std::shared_ptr<TransportFactory>> items_;
};

}
}