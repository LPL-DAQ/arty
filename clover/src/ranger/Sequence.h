// Defines and schedules timed autonomous valve sequence events.
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
constexpr size_t MAX_UPLOAD_CHUNK_SIZE = 512;

struct Event {
    uint32_t time_ms = 0;
    std::array<char, 8> valve{};
    bool open = false;
};

struct Tick {
    // Contains zero or one due event; the event remains pending until acknowledged.
    std::span<const Event> due_events;
    bool complete = false;
};

class Definition {
public:
    /// Parses one ignition run from a sequence log.
    /// Parameters: text is the full log; run_index selects its zero-based run.
    /// Returns: the parsed run or an error if it is missing or invalid.
    static std::expected<Definition, Error> parse_log(std::string_view text, uint32_t run_index);

    /// Loads a sequence from uploaded memory or the configured SD-card directory.
    /// Parameters: file_name identifies the sequence; run_index selects its run.
    /// Returns: the parsed run or an error if it cannot be loaded.
    static std::expected<Definition, Error> load_file(std::string_view file_name, uint32_t run_index = 0);

    /// Accepts one ordered chunk of a sequence upload and validates the complete selected run.
    /// Parameters: file_name, offset, total_size, data, final_chunk, and run_index describe the upload.
    /// Returns: success when the chunk is accepted or the completed run validates; otherwise an error.
    static std::expected<void, Error> upload_chunk(
        std::string_view file_name,
        uint32_t offset,
        uint32_t total_size,
        std::string_view data,
        bool final_chunk,
        uint32_t run_index);

    /// Returns one event by index.
    /// Parameters: index is the event's zero-based position.
    /// Returns: a reference to the indexed event.
    const Event& event(size_t index) const;

    /// Returns all parsed events in sequence order.
    /// Parameters: None.
    /// Returns: a non-owning span of the events.
    std::span<const Event> events() const;

    /// Returns the number of parsed events.
    /// Parameters: None.
    /// Returns: the event count.
    size_t size() const;

    /// Returns the sequence duration in milliseconds.
    /// Parameters: None.
    /// Returns: the duration.
    uint32_t duration_ms() const;

private:
    /// Adds a validated event to the definition.
    /// Parameters: time_ms is relative event time; valve and state identify its target.
    /// Returns: success or an error if the event is invalid or capacity is exhausted.
    std::expected<void, Error> append(uint32_t time_ms, std::string_view valve, std::string_view state);

    /// Sets the duration after checking it does not precede an event.
    /// Parameters: duration_ms is the requested duration.
    /// Returns: success or an error when the duration is too short.
    std::expected<void, Error> set_duration(uint32_t duration_ms);

    std::array<Event, MAX_EVENTS> events_{};
    size_t event_count_ = 0;
    uint32_t duration_ms_ = 0;
};

class Scheduler {
public:
    /// Stores a parsed definition and prepares the scheduler.
    /// Parameters: definition is the validated sequence to schedule.
    /// Returns: success or an error for an empty definition.
    std::expected<void, Error> load(Definition definition);

    /// Starts or restarts a loaded sequence.
    /// Parameters: None.
    /// Returns: success or an error when no definition is loaded.
    std::expected<void, Error> start();

    /// Clears scheduler progress without discarding the loaded definition.
    /// Parameters: None.
    /// Returns: nothing.
    void reset();

    /// Reports at most the next due event; it remains pending until acknowledged.
    /// Parameters: elapsed_ms is monotonic time since sequence start.
    /// Returns: the pending event (if due) and completion status, or a timing/state error.
    std::expected<Tick, Error> advance(uint32_t elapsed_ms);

    /// Acknowledges that the controller successfully issued the pending event.
    /// Parameters: None.
    /// Returns: success or an error when no due event is awaiting acknowledgement.
    std::expected<void, Error> acknowledge_event();

    /// Reports whether a definition is loaded.
    /// Parameters: None.
    /// Returns: true when a definition is loaded.
    bool is_loaded() const;

    /// Reports whether the sequence has started and is not complete.
    /// Parameters: None.
    /// Returns: true while the scheduler is running.
    bool is_running() const;

    /// Reports whether all events were acknowledged and the duration elapsed.
    /// Parameters: None.
    /// Returns: true when the sequence has completed.
    bool is_complete() const;

    /// Returns the loaded sequence duration in milliseconds.
    /// Parameters: None.
    /// Returns: the duration, or zero when no definition is loaded.
    uint32_t duration_ms() const;

private:
    Definition definition_{};
    size_t next_event_ = 0;
    uint32_t last_elapsed_ms_ = 0;
    // Prevents elapsed-time advancement from consuming an event before actuation succeeds.
    bool event_pending_ = false;
    bool loaded_ = false;
    bool started_ = false;
    bool complete_ = false;
};
}  // namespace Sequence
