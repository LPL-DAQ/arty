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

#include <cmath>
#include <limits>
#include <cstring>
#include <cstdlib>

#include "moteus_protocol.h"

namespace mm = mjbots::moteus;

/// This is the primary interface to a moteus controller. One
/// instance of this class should be created per controller that is
/// commanded or monitored.
///
/// Note: In Zephyr, this class acts purely as a payload generator.
/// Hardware transmission is handled externally via SocketCAN.
class Moteus {
 public:
  using CanFdFrame = mm::CanFdFrame;

  using Query = mm::Query;
  using PositionMode = mm::PositionMode;
  using VFOCMode = mm::VFOCMode;
  using CurrentMode = mm::CurrentMode;
  using StayWithinMode = mm::StayWithinMode;

  static constexpr mm::Resolution kInt8 = mm::Resolution::kInt8;
  static constexpr mm::Resolution kInt16 = mm::Resolution::kInt16;
  static constexpr mm::Resolution kInt32 = mm::Resolution::kInt32;
  static constexpr mm::Resolution kFloat = mm::Resolution::kFloat;

  struct Options {
    // The ID of the servo to communicate with.
    int8_t id = 1;

    // The source ID to use for the commanding node.
    int8_t source = 0;

    mm::Query::Format query_format;

    // Use the given prefix for all CAN IDs.
    uint16_t can_prefix = 0x0000;

    // Disable BRS on outgoing frames.
    bool disable_brs = true;

    // Request the configured set of registers as a query with every command.
    bool default_query = true;

    Options() {}
  };

  Options options_;

  Moteus() {}

  // ------------------------------------------------------------------
  // CRITICAL: Call this before the control loop to allocate memory 
  // for the telemetry query suffix.
  // ------------------------------------------------------------------
  void Initialize() {
    mm::CanData can_data;
    mm::WriteCanData query_write(&can_data);
    mm::Query::Make(&query_write, options_.query_format);

    query_size_ = can_data.size;
    query_data_ = reinterpret_cast<char*>(realloc(query_data_, query_size_));
    ::memcpy(query_data_, &can_data.data[0], query_size_);
  }

  struct Result {
    unsigned long timestamp = 0;
    CanFdFrame frame;
    mm::Query::Result values;
  };

  // The most recent result from any command.
  const Result& last_result() const { return last_result_; }

  /////////////////////////////////////////
  // Query
  CanFdFrame MakeQuery(const mm::Query::Format* format_override = nullptr) {
    return MakeFrame(mm::EmptyMode(), {}, {},
                     format_override == nullptr ?
                     &options_.query_format : format_override);
  }

  /////////////////////////////////////////
  // StopMode
  CanFdFrame MakeStop(const mm::Query::Format* query_override = nullptr) {
    return MakeFrame(mm::StopMode(), {}, {}, query_override);
  }

  /////////////////////////////////////////
  // BrakeMode
  CanFdFrame MakeBrake(const mm::Query::Format* query_override = nullptr) {
    return MakeFrame(mm::BrakeMode(), {}, {}, query_override);
  }

  /////////////////////////////////////////
  // PositionMode
  CanFdFrame MakePosition(const mm::PositionMode::Command& cmd,
                          const mm::PositionMode::Format* command_override = nullptr,
                          const mm::Query::Format* query_override = nullptr) {
    return MakeFrame(mm::PositionMode(),
                     cmd,
                     (command_override == nullptr ?
                      mm::PositionMode::Format() : *command_override),
                     query_override);
  }

  /////////////////////////////////////////
  // VFOCMode
  CanFdFrame MakeVFOC(const mm::VFOCMode::Command& cmd,
                      const mm::VFOCMode::Format* command_override = nullptr,
                      const mm::Query::Format* query_override = nullptr) {
    return MakeFrame(mm::VFOCMode(),
                     cmd,
                     (command_override == nullptr ?
                      mm::VFOCMode::Format() : *command_override),
                     query_override);
  }

  /////////////////////////////////////////
  // CurrentMode
  CanFdFrame MakeCurrent(const mm::CurrentMode::Command& cmd,
                         const mm::CurrentMode::Format* command_override = nullptr,
                         const mm::Query::Format* query_override = nullptr) {
    return MakeFrame(mm::CurrentMode(),
                     cmd,
                     (command_override == nullptr ?
                      mm::CurrentMode::Format() : *command_override),
                     query_override);
  }

  /////////////////////////////////////////
  // StayWithinMode
  CanFdFrame MakeStayWithin(const mm::StayWithinMode::Command& cmd,
                            const mm::StayWithinMode::Format* command_override = nullptr,
                            const mm::Query::Format* query_override = nullptr) {
    return MakeFrame(mm::StayWithinMode(),
                     cmd,
                     (command_override == nullptr ?
                      mm::StayWithinMode::Format() : *command_override),
                     query_override);
  }

  /////////////////////////////////////////
  // OutputNearest
  CanFdFrame MakeOutputNearest(const mm::OutputNearest::Command& cmd,
                               const mm::OutputNearest::Format* command_override = nullptr,
                               const mm::Query::Format* query_override = nullptr) {
    return MakeFrame(mm::OutputNearest(),
                     cmd,
                     (command_override == nullptr ?
                      mm::OutputNearest::Format() : *command_override),
                     query_override);
  }

  /////////////////////////////////////////
  // OutputExact
  CanFdFrame MakeOutputExact(const mm::OutputExact::Command& cmd,
                               const mm::OutputExact::Format* command_override = nullptr,
                               const mm::Query::Format* query_override = nullptr) {
    return MakeFrame(mm::OutputExact(),
                     cmd,
                     (command_override == nullptr ?
                      mm::OutputExact::Format() : *command_override),
                     query_override);
  }

  /////////////////////////////////////////
  // RequireReindex
  CanFdFrame MakeRequireReindex(const mm::RequireReindex::Command& cmd,
                                const mm::RequireReindex::Format* command_override = nullptr,
                               const mm::Query::Format* query_override = nullptr) {
    return MakeFrame(mm::RequireReindex(),
                     cmd,
                     (command_override == nullptr ?
                      mm::RequireReindex::Format() : *command_override),
                     query_override);
  }

  /////////////////////////////////////////
  // RecapturePositionVelocity
  CanFdFrame MakeRecapturePositionVelocity(const mm::RecapturePositionVelocity::Command& cmd,
                                const mm::RecapturePositionVelocity::Format* command_override = nullptr,
                               const mm::Query::Format* query_override = nullptr) {
    return MakeFrame(mm::RecapturePositionVelocity(),
                     cmd,
                     (command_override == nullptr ?
                      mm::RecapturePositionVelocity::Format() : *command_override),
                     query_override);
  }

  /////////////////////////////////////////
  // RTOS Telemetry Parser
  // Pass the raw data retrieved from recv() on the SocketCAN 
  // interface directly into this function to update telemetry.
  /////////////////////////////////////////
  void ParseTelemetry(const uint8_t* rx_data, size_t length) {
      last_result_.values = mm::Query::Parse(rx_data, length);
  }

 private:
  enum ReplyMode {
    kNoReply,
    kReplyRequired,
  };

  CanFdFrame DefaultFrame(ReplyMode reply_mode = kReplyRequired) {
    CanFdFrame result;
    result.destination = options_.id;
    result.reply_required = (reply_mode == kReplyRequired);

    result.arbitration_id =
        (result.destination) |
        (result.source << 8) |
        (result.reply_required ? 0x8000 : 0x0000) |
        (static_cast<uint32_t>(options_.can_prefix) << 16);
    result.bus = 0;

    return result;
  }

  template <typename CommandType>
  CanFdFrame MakeFrame(const CommandType&,
                       const typename CommandType::Command& cmd,
                       const typename CommandType::Format& fmt,
                       const mm::Query::Format* query_override = nullptr) {
    auto result = DefaultFrame(
        query_override != nullptr ? kReplyRequired :
        options_.default_query ? kReplyRequired : kNoReply);

    mm::WriteCanData write_frame(result.data, &result.size);
    CommandType::Make(&write_frame, cmd, fmt);

    if (query_override) {
      mm::Query::Make(&write_frame, *query_override);
    } else if (options_.default_query && query_data_ != nullptr) {
      // Memory safety check added for query_data_
      ::memcpy(&result.data[result.size],
               query_data_,
               query_size_);
      result.size += query_size_;
    }

    return result;
  }

  static int8_t RoundUpDlc(int8_t size) {
    if (size <= 0) { return 0; }
    if (size <= 1) { return 1; }
    if (size <= 2) { return 2; }
    if (size <= 3) { return 3; }
    if (size <= 4) { return 4; }
    if (size <= 5) { return 5; }
    if (size <= 6) { return 6; }
    if (size <= 7) { return 7; }
    if (size <= 8) { return 8; }
    if (size <= 12) { return 12; }
    if (size <= 16) { return 16; }
    if (size <= 20) { return 20; }
    if (size <= 24) { return 24; }
    if (size <= 32) { return 32; }
    if (size <= 48) { return 48; }
    if (size <= 64) { return 64; }
    return 0;
  }

  static void PadCan(CanFdFrame* msg) {
    const auto new_size = RoundUpDlc(msg->size);
    for (int8_t i = msg->size; i < new_size; i++) {
      msg->data[i] = 0x50;
    }
    msg->size = new_size;
  }

  Result last_result_;
  char* query_data_ = nullptr;
  size_t query_size_ = 0;
};