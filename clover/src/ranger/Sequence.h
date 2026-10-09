#pragma once

#include "Error.h"
#include <array>
#include <cstddef>
#include <cstdint>
#include <expected>
#include <span>
#include <string_view>

namespace Sequence {
constexpr size_t MAX_EVENTS = 64;
constexpr size_t MAX_FILE_SIZE = 8 * 1024;
constexpr size_t MAX_FILE_NAME_SIZE = 48;

struct Event {
    uint32_t time_ms = 0;
    std::array<char, 8> valve{};
    bool open = false;
};

struct Tick {
    std::span<const Event> due_events;
    bool complete = false;
};

class Definition {
public:
    static std::expected<Definition, Error> parse_log(std::string_view text, uint32_t run_index);
    static std::expected<Definition, Error> load_file(std::string_view file_name, uint32_t run_index = 0);

    const Event& event(size_t index) const;
    std::span<const Event> events() const;
    size_t size() const;
    uint32_t duration_ms() const;

private:
    std::expected<void, Error> append(uint32_t time_ms, std::string_view valve, std::string_view state);
    std::expected<void, Error> set_duration(uint32_t duration_ms);

    std::array<Event, MAX_EVENTS> events_{};
    size_t event_count_ = 0;
    uint32_t duration_ms_ = 0;
};

class Scheduler {
public:
    std::expected<void, Error> load(Definition definition);
    std::expected<void, Error> start();
    void reset();
    std::expected<Tick, Error> advance(uint32_t elapsed_ms);

    bool is_loaded() const;
    bool is_running() const;
    bool is_complete() const;
    uint32_t duration_ms() const;

private:
    Definition definition_{};
    size_t next_event_ = 0;
    uint32_t last_elapsed_ms_ = 0;
    bool loaded_ = false;
    bool started_ = false;
    bool complete_ = false;
};
}  // namespace Sequence
