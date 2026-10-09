// Parses ignition logs, stores uploaded definitions, and schedules valve events.
#include "Sequence.h"
#include "MutexGuard.h"

#include <algorithm>
#include <cctype>
#include <cstdio>
#include <cerrno>
#include <limits>

namespace {
constexpr size_t MAX_RUNS = 64;
constexpr std::string_view SEQUENCE_DIRECTORY = "/SD:/sequences/";

K_MUTEX_DEFINE(sequence_file_lock);
std::array<char, Sequence::MAX_FILE_SIZE + 1> sequence_file_buffer{};
std::array<char, Sequence::MAX_FILE_NAME_SIZE + 1> uploaded_sequence_file_name{};
std::array<char, Sequence::MAX_FILE_NAME_SIZE + 1> upload_staging_file_name{};
size_t upload_received_size = 0;
uint32_t upload_expected_size = 0;
uint32_t uploaded_sequence_run_index = 0;
uint32_t upload_run_index = 0;
Sequence::Definition uploaded_sequence_definition{};
bool uploaded_sequence_ready = false;
bool upload_in_progress = false;

/// Checks a sequence filename against the controller's accepted basename rules.
/// Parameters: file_name is the name supplied by the client.
/// Returns: true when the name is safe and uses the supported log extension.
bool valid_file_name(std::string_view file_name)
{
    return !file_name.empty() && file_name.size() <= Sequence::MAX_FILE_NAME_SIZE &&
           file_name.find("..") == std::string_view::npos &&
           std::all_of(file_name.begin(), file_name.end(), [](char ch) {
               return (ch >= 'a' && ch <= 'z') || (ch >= 'A' && ch <= 'Z') || (ch >= '0' && ch <= '9') ||
                      ch == '_' || ch == '-' || ch == '.';
           }) &&
           file_name.ends_with(".log");
}

/// Removes leading and trailing whitespace from a view.
/// Parameters: value is the text to trim.
/// Returns: a view of the trimmed text.
std::string_view trim(std::string_view value)
{
    while (!value.empty() && std::isspace(static_cast<unsigned char>(value.front()))) {
        value.remove_prefix(1);
    }
    while (!value.empty() && std::isspace(static_cast<unsigned char>(value.back()))) {
        value.remove_suffix(1);
    }
    return value;
}

/// Parses an unsigned decimal integer.
/// Parameters: text is the numeric token; value receives the result on success.
/// Returns: true when text is a valid in-range integer.
bool parse_uint_value(std::string_view text, uint32_t& value)
{
    text = trim(text);
    if (text.empty()) {
        return false;
    }

    uint32_t parsed = 0;
    for (const char digit : text) {
        if (digit < '0' || digit > '9' || parsed > (std::numeric_limits<uint32_t>::max() - (digit - '0')) / 10) {
            return false;
        }
        parsed = parsed * 10 + static_cast<uint32_t>(digit - '0');
    }
    value = parsed;
    return true;
}

/// Checks whether a token has a supported valve-name shape.
/// Parameters: name is the valve token from the ignition log.
/// Returns: true when the token matches a supported valve-name pattern.
bool valid_valve_name(std::string_view name)
{
    if (name.size() == 7 && name.starts_with("PBV-") &&
        std::all_of(name.begin() + 4, name.end(), [](char ch) { return ch >= '0' && ch <= '9'; })) {
        return true;
    }
    if ((name.size() == 6 && (name.starts_with("PBV") || name.starts_with("SVR")) &&
         std::all_of(name.end() - 3, name.end(), [](char ch) { return ch >= '0' && ch <= '9'; })) ||
        (name.size() == 5 && name.starts_with("SV") &&
         std::all_of(name.end() - 3, name.end(), [](char ch) { return ch >= '0' && ch <= '9'; }))) {
        return true;
    }
    return false;
}

/// Converts a decimal seconds token to milliseconds.
/// Parameters: text is the seconds token; value receives milliseconds on success.
/// Returns: true when text is a valid nonnegative timestamp.
bool parse_seconds_ms(std::string_view text, uint32_t& value)
{
    text = trim(text);
    if (text.empty()) {
        return false;
    }

    const size_t decimal = text.find('.');
    const std::string_view whole_text = text.substr(0, decimal);
    uint32_t seconds = 0;
    if (!parse_uint_value(whole_text, seconds) || seconds > std::numeric_limits<uint32_t>::max() / 1000) {
        return false;
    }

    uint32_t milliseconds = seconds * 1000;
    if (decimal != std::string_view::npos) {
        const auto fraction = text.substr(decimal + 1);
        if (fraction.empty()) {
            return false;
        }
        uint32_t fraction_ms = 0;
        size_t digits = 0;
        for (const char digit : fraction) {
            if (digit < '0' || digit > '9') {
                return false;
            }
            if (digits < 3) {
                fraction_ms = fraction_ms * 10 + static_cast<uint32_t>(digit - '0');
            }
            ++digits;
        }
        while (digits < 3) {
            fraction_ms *= 10;
            ++digits;
        }
        if (milliseconds > std::numeric_limits<uint32_t>::max() - fraction_ms) {
            return false;
        }
        milliseconds += fraction_ms;
    }
    value = milliseconds;
    return true;
}

/// Splits a line into whitespace-delimited tokens.
/// Parameters: line is the source text; fields and count receive the tokens and count.
/// Returns: true when the line fits in the token array.
bool split_fields(std::string_view line, std::array<std::string_view, 8>& fields, size_t& count)
{
    count = 0;
    while (!(line = trim(line)).empty()) {
        if (count == fields.size()) {
            return false;
        }
        const size_t separator = line.find_first_of(" \t");
        fields[count++] = line.substr(0, separator);
        if (separator == std::string_view::npos) {
            break;
        }
        line.remove_prefix(separator + 1);
    }
    return true;
}

/// Parses an ignition valve event line.
/// Parameters: line is the source; valve, target, and timestamp_ms receive parsed fields.
/// Returns: true when all required fields are valid.
bool parse_log_event(std::string_view line, std::string_view& valve, std::string_view& target, uint32_t& timestamp_ms)
{
    std::array<std::string_view, 8> fields{};
    size_t count = 0;
    if (!split_fields(line, fields, count) || count != 6 || fields[0] != "[IGNITION]" || fields[3] != "->") {
        return false;
    }
    valve = fields[1];
    target = fields[4];
    return valid_valve_name(valve) && (fields[2] == "OPEN" || fields[2] == "CLOSE" || fields[2] == "CLOSED") &&
           (target == "OPEN" || target == "CLOSE") && parse_seconds_ms(fields[5], timestamp_ms);
}

/// Parses an ignition-start marker.
/// Parameters: line is the source; timestamp_ms receives the marker time.
/// Returns: true when the line is a valid start marker.
bool parse_ignition_marker(std::string_view line, uint32_t& timestamp_ms)
{
    std::array<std::string_view, 8> fields{};
    size_t count = 0;
    return split_fields(line, fields, count) && count == 3 && fields[0] == "[USER]" && fields[1] == "IGNITION" &&
           parse_seconds_ms(fields[2], timestamp_ms);
}

/// Parses an ignition-termination marker.
/// Parameters: line is the source; timestamp_ms receives the marker time.
/// Returns: true when the line is a valid termination marker.
bool parse_terminated_marker(std::string_view line, uint32_t& timestamp_ms)
{
    std::array<std::string_view, 8> fields{};
    size_t count = 0;
    return split_fields(line, fields, count) && count == 3 && fields[0] == "[USER]" && fields[1] == "IGNITION_TERMINATED" &&
           parse_seconds_ms(fields[2], timestamp_ms);
}

}  // namespace

/// Adds a validated valve event to the definition.
/// Parameters: time_ms is relative event time; valve and state identify the target.
/// Returns: success or an error if the event is invalid or capacity is exhausted.
std::expected<void, Error> Sequence::Definition::append(uint32_t time_ms, std::string_view valve, std::string_view state)
{
    if (event_count_ == events_.size()) {
        return std::unexpected(Error::from_cause("sequence exceeds maximum event count of %u", static_cast<unsigned>(events_.size())));
    }
    if (!valid_valve_name(valve)) {
        return std::unexpected(Error::from_cause("unsupported valve name '%.*s'", static_cast<int>(valve.size()), valve.data()));
    }
    if (state != "OPEN" && state != "CLOSE") {
        return std::unexpected(Error::from_cause("unsupported valve state '%.*s'", static_cast<int>(state.size()), state.data()));
    }
    if (event_count_ > 0 && time_ms < events_[event_count_ - 1].time_ms) {
        return std::unexpected(Error::from_cause("sequence event times must be nondecreasing"));
    }

    Event& event = events_[event_count_++];
    event.time_ms = time_ms;
    if (valve.size() == 7) {
        std::copy_n(valve.begin(), 3, event.valve.begin());
        std::copy(valve.begin() + 4, valve.end(), event.valve.begin() + 3);
    }
    else {
        std::copy(valve.begin(), valve.end(), event.valve.begin());
    }
    event.open = state == "OPEN";
    duration_ms_ = std::max(duration_ms_, time_ms);
    return {};
}

/// Sets the duration after checking it does not precede the last event.
/// Parameters: duration_ms is the requested sequence duration.
/// Returns: success or an error when the requested duration is too short.
std::expected<void, Error> Sequence::Definition::set_duration(uint32_t duration_ms)
{
    if (duration_ms < duration_ms_) {
        return std::unexpected(Error::from_cause("sequence duration is shorter than its last event"));
    }
    duration_ms_ = duration_ms;
    return {};
}

/// Parses the selected ignition run from a complete log.
/// Parameters: text is the complete log; run_index selects a zero-based run.
/// Returns: the selected definition or an error when the run is invalid or absent.
std::expected<Sequence::Definition, Error> Sequence::Definition::parse_log(std::string_view text, uint32_t run_index)
{
    if (text.empty() || text.size() > MAX_FILE_SIZE || run_index >= MAX_RUNS) {
        return std::unexpected(Error::from_cause("sequence log is empty, too large, or run index is out of range"));
    }

    Definition selected;
    Definition current;
    uint32_t run_count = 0;
    uint32_t ignition_time_ms = 0;
    bool in_run = false;
    bool selected_run_complete = false;

    while (!text.empty()) {
        const size_t newline = text.find('\n');
        const std::string_view line = trim(text.substr(0, newline));
        text = newline == std::string_view::npos ? std::string_view{} : text.substr(newline + 1);
        if (line.empty()) {
            continue;
        }

        uint32_t marker_time_ms = 0;
        if (parse_ignition_marker(line, marker_time_ms)) {
            if (in_run || run_count >= MAX_RUNS) {
                return std::unexpected(Error::from_cause("invalid or excessive ignition runs in sequence log"));
            }
            in_run = true;
            ignition_time_ms = marker_time_ms;
            current = Definition{};
            continue;
        }
        uint32_t termination_time_ms = 0;
        if (parse_terminated_marker(line, termination_time_ms)) {
            if (!in_run) {
                return std::unexpected(Error::from_cause("ignition termination marker has no matching start"));
            }
            if (termination_time_ms < ignition_time_ms) {
                return std::unexpected(Error::from_cause("ignition termination precedes its start"));
            }
            if (run_count == run_index) {
                if (auto result = current.set_duration(termination_time_ms - ignition_time_ms); !result) {
                    return std::unexpected(result.error());
                }
                selected = current;
                selected_run_complete = true;
            }
            ++run_count;
            in_run = false;
            continue;
        }
        if (!in_run || !line.starts_with("[IGNITION]")) {
            continue;
        }

        std::string_view valve;
        std::string_view target;
        uint32_t event_time_ms = 0;
        if (!parse_log_event(line, valve, target, event_time_ms) || event_time_ms < ignition_time_ms) {
            return std::unexpected(Error::from_cause("invalid ignition event in sequence log"));
        }
        if (run_count == run_index) {
            if (auto result = current.append(event_time_ms - ignition_time_ms, valve, target); !result) {
                return std::unexpected(result.error());
            }
        }
    }

    if (in_run) {
        return std::unexpected(Error::from_cause("selected ignition run is not terminated"));
    }
    if (!selected_run_complete) {
        return std::unexpected(Error::from_cause("requested ignition run does not exist or is incomplete"));
    }
    if (selected.size() == 0) {
        return std::unexpected(Error::from_cause("selected ignition run contains no valve events"));
    }
    return selected;
}

/// Loads a previously uploaded definition or reads the named file from SD.
/// Parameters: file_name identifies the sequence; run_index selects its ignition run.
/// Returns: the selected definition or a loading/parsing error.
std::expected<Sequence::Definition, Error> Sequence::Definition::load_file(std::string_view file_name, uint32_t run_index)
{
    if (!valid_file_name(file_name)) {
        return std::unexpected(Error::from_cause("sequence file name is invalid"));
    }

    MutexGuard guard{&sequence_file_lock};
    if (upload_in_progress &&
        (!uploaded_sequence_ready || std::string_view{uploaded_sequence_file_name.data()} != file_name)) {
        return std::unexpected(Error::from_cause("a sequence upload is in progress"));
    }
    if (uploaded_sequence_ready && std::string_view{uploaded_sequence_file_name.data()} == file_name) {
        if (run_index != uploaded_sequence_run_index) {
            return std::unexpected(Error::from_cause("uploaded sequence was prepared for run index %u", uploaded_sequence_run_index));
        }
        return uploaded_sequence_definition;
    }

    std::array<char, 80> path{};
    const size_t required_path_size = SEQUENCE_DIRECTORY.size() + file_name.size() + 1;
    if (required_path_size > path.size()) {
        return std::unexpected(Error::from_cause("sequence file path is too long"));
    }
    std::copy(SEQUENCE_DIRECTORY.begin(), SEQUENCE_DIRECTORY.end(), path.begin());
    std::copy(file_name.begin(), file_name.end(), path.begin() + SEQUENCE_DIRECTORY.size());

    FILE* file = std::fopen(path.data(), "rb");
    if (file == nullptr) {
        return std::unexpected(Error::from_code(errno).context("failed to open sequence file"));
    }

    const size_t bytes_read = std::fread(sequence_file_buffer.data(), 1, sequence_file_buffer.size(), file);
    const bool read_failed = std::ferror(file) != 0;
    const int close_result = std::fclose(file);
    if (read_failed) {
        return std::unexpected(Error::from_code(errno).context("failed to read sequence file"));
    }
    if (bytes_read > MAX_FILE_SIZE) {
        return std::unexpected(Error::from_cause("sequence file exceeds %u bytes", static_cast<unsigned>(MAX_FILE_SIZE)));
    }
    if (close_result != 0) {
        return std::unexpected(Error::from_code(errno).context("failed to close sequence file"));
    }
    sequence_file_buffer[bytes_read] = '\0';
    const std::string_view content{sequence_file_buffer.data(), bytes_read};
    return parse_log(content, run_index);
}

/// Adds one ordered upload chunk and validates the selected run when complete.
/// Parameters: file_name, offset, total_size, data, final_chunk, and run_index describe the upload.
/// Returns: success when accepted or an error for invalid, incomplete-order, or malformed input.
std::expected<void, Error> Sequence::Definition::upload_chunk(
    std::string_view file_name,
    uint32_t offset,
    uint32_t total_size,
    std::string_view data,
    bool final_chunk,
    uint32_t run_index)
{
    if (!valid_file_name(file_name)) {
        return std::unexpected(Error::from_cause("sequence file name is invalid"));
    }
    if (total_size == 0 || total_size > MAX_FILE_SIZE) {
        return std::unexpected(Error::from_cause("sequence file size must be between 1 and %u bytes", static_cast<unsigned>(MAX_FILE_SIZE)));
    }
    if (data.empty() || data.size() > MAX_UPLOAD_CHUNK_SIZE) {
        return std::unexpected(Error::from_cause("sequence upload chunk size must be between 1 and %u bytes", static_cast<unsigned>(MAX_UPLOAD_CHUNK_SIZE)));
    }
    if (data.find('\0') != std::string_view::npos) {
        return std::unexpected(Error::from_cause("sequence upload chunk contains a null byte"));
    }
    if (offset > total_size) {
        return std::unexpected(Error::from_cause("sequence upload offset exceeds the declared file size"));
    }

    MutexGuard guard{&sequence_file_lock};
    if (offset == 0) {
        upload_staging_file_name.fill('\0');
        std::copy(file_name.begin(), file_name.end(), upload_staging_file_name.begin());
        upload_received_size = 0;
        upload_expected_size = total_size;
        upload_run_index = run_index;
        upload_in_progress = true;
    }
    else if (!upload_in_progress) {
        return std::unexpected(Error::from_cause("sequence upload must start at offset 0"));
    }

    if (!upload_in_progress || std::string_view{upload_staging_file_name.data()} != file_name ||
        upload_expected_size != total_size || upload_run_index != run_index || offset != upload_received_size) {
        return std::unexpected(Error::from_cause("sequence upload chunk is out of order or does not match the active upload"));
    }
    if (data.size() > total_size - offset) {
        return std::unexpected(Error::from_cause("sequence upload chunk exceeds the declared file size"));
    }
    const size_t next_size = upload_received_size + data.size();
    if (final_chunk != (next_size == total_size)) {
        return std::unexpected(Error::from_cause("sequence upload final-chunk flag does not match the declared file size"));
    }

    std::copy(data.begin(), data.end(), sequence_file_buffer.begin() + upload_received_size);
    upload_received_size = next_size;
    if (final_chunk) {
        sequence_file_buffer[upload_received_size] = '\0';
        auto definition = parse_log(std::string_view{sequence_file_buffer.data(), upload_received_size}, upload_run_index);
        if (!definition) {
            upload_in_progress = false;
            return std::unexpected(definition.error().context("failed to validate uploaded sequence"));
        }
        std::copy(upload_staging_file_name.begin(), upload_staging_file_name.end(), uploaded_sequence_file_name.begin());
        uploaded_sequence_definition = std::move(*definition);
        uploaded_sequence_run_index = upload_run_index;
        uploaded_sequence_ready = true;
        upload_in_progress = false;
    }
    return {};
}

/// Returns one event by zero-based index.
/// Parameters: index is the event position.
/// Returns: a reference to the indexed event.
const Sequence::Event& Sequence::Definition::event(size_t index) const
{
    return events_[index];
}

/// Returns the number of events in the definition.
/// Parameters: None.
/// Returns: the event count.
size_t Sequence::Definition::size() const
{
    return event_count_;
}

/// Returns the total sequence duration.
/// Parameters: None.
/// Returns: duration in milliseconds.
uint32_t Sequence::Definition::duration_ms() const
{
    return duration_ms_;
}

/// Returns a non-owning view of events in time order.
/// Parameters: None.
/// Returns: a span over the parsed events.
std::span<const Sequence::Event> Sequence::Definition::events() const
{
    return {events_.data(), event_count_};
}

/// Loads a parsed sequence and resets scheduler progress.
/// Parameters: definition is the validated sequence to schedule.
/// Returns: success or an error when the sequence has no events.
std::expected<void, Error> Sequence::Scheduler::load(Definition definition)
{
    if (definition.size() == 0) {
        return std::unexpected(Error::from_cause("cannot schedule an empty sequence"));
    }
    definition_ = std::move(definition);
    next_event_ = 0;
    last_elapsed_ms_ = 0;
    event_pending_ = false;
    loaded_ = true;
    started_ = false;
    complete_ = false;
    return {};
}

/// Starts or restarts the loaded sequence.
/// Parameters: None.
/// Returns: success or an error when no sequence has been loaded.
std::expected<void, Error> Sequence::Scheduler::start()
{
    if (!loaded_) {
        return std::unexpected(Error::from_cause("no sequence is loaded"));
    }
    next_event_ = 0;
    last_elapsed_ms_ = 0;
    event_pending_ = false;
    started_ = true;
    complete_ = false;
    return {};
}

/// Clears scheduler progress while retaining its loaded definition.
/// Parameters: None.
/// Returns: nothing.
void Sequence::Scheduler::reset()
{
    next_event_ = 0;
    last_elapsed_ms_ = 0;
    event_pending_ = false;
    started_ = false;
    complete_ = false;
}

/// Reports the next due event without consuming it.
/// Parameters: elapsed_ms is monotonic time since the sequence started.
/// Returns: at most one due event and the completion status, or a scheduler error.
std::expected<Sequence::Tick, Error> Sequence::Scheduler::advance(uint32_t elapsed_ms)
{
    if (!started_) {
        return std::unexpected(Error::from_cause("sequence scheduler has not been started"));
    }
    if (elapsed_ms < last_elapsed_ms_) {
        return std::unexpected(Error::from_cause("sequence elapsed time must be monotonic"));
    }

    if (!event_pending_ && next_event_ < definition_.size() && definition_.event(next_event_).time_ms <= elapsed_ms) {
        event_pending_ = true;
    }
    last_elapsed_ms_ = elapsed_ms;
    complete_ = !event_pending_ && next_event_ == definition_.size() && elapsed_ms >= definition_.duration_ms();
    return Tick{
        .due_events = event_pending_ ? definition_.events().subspan(next_event_, 1) : std::span<const Event>{},
        .complete = complete_,
    };
}

/// Marks the pending due event consumed after the controller successfully issues it.
/// Parameters: None.
/// Returns: success or an error when no event is pending.
std::expected<void, Error> Sequence::Scheduler::acknowledge_event()
{
    if (!started_ || !event_pending_) {
        return std::unexpected(Error::from_cause("no due sequence event is awaiting acknowledgement"));
    }
    ++next_event_;
    event_pending_ = false;
    complete_ = next_event_ == definition_.size() && last_elapsed_ms_ >= definition_.duration_ms();
    return {};
}

/// Reports whether a definition is loaded.
/// Parameters: None.
/// Returns: true when a definition is loaded.
bool Sequence::Scheduler::is_loaded() const
{
    return loaded_;
}

/// Reports whether the scheduler is active and incomplete.
/// Parameters: None.
/// Returns: true while the sequence is running.
bool Sequence::Scheduler::is_running() const
{
    return started_ && !complete_;
}

/// Reports whether all events were acknowledged and the duration elapsed.
/// Parameters: None.
/// Returns: true when the sequence is complete.
bool Sequence::Scheduler::is_complete() const
{
    return complete_;
}

/// Returns the loaded sequence duration.
/// Parameters: None.
/// Returns: duration in milliseconds, or zero before loading.
uint32_t Sequence::Scheduler::duration_ms() const
{
    return definition_.duration_ms();
}
