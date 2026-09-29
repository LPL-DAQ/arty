#include "Sequence.h"
#include "MutexGuard.h"

#include <algorithm>
#include <cctype>
#include <cstdio>
#include <cerrno>
#include <limits>

namespace {
constexpr size_t MAX_RUNS = 64;
constexpr uint32_t INVALID_TIME = std::numeric_limits<uint32_t>::max();
constexpr std::string_view SEQUENCE_DIRECTORY = "/SD:/sequences/";

K_MUTEX_DEFINE(sequence_file_lock);
std::array<char, Sequence::MAX_FILE_SIZE + 1> sequence_file_buffer{};

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

bool parse_log_event(std::string_view line, std::string_view& valve, std::string_view& target, uint32_t& timestamp_ms)
{
    std::array<std::string_view, 8> fields{};
    size_t count = 0;
    if (!split_fields(line, fields, count) || count != 6 || fields[0] != "[IGNITION]" || fields[3] != "->") {
        return false;
    }
    valve = fields[1];
    target = fields[4];
    return valid_valve_name(valve) && (fields[2] == "OPEN" || fields[2] == "CLOSE") &&
           (target == "OPEN" || target == "CLOSE") && parse_seconds_ms(fields[5], timestamp_ms);
}

bool parse_ignition_marker(std::string_view line, uint32_t& timestamp_ms)
{
    std::array<std::string_view, 8> fields{};
    size_t count = 0;
    return split_fields(line, fields, count) && count == 3 && fields[0] == "[USER]" && fields[1] == "IGNITION" &&
           parse_seconds_ms(fields[2], timestamp_ms);
}

bool parse_terminated_marker(std::string_view line, uint32_t& timestamp_ms)
{
    std::array<std::string_view, 8> fields{};
    size_t count = 0;
    return split_fields(line, fields, count) && count == 3 && fields[0] == "[USER]" && fields[1] == "IGNITION_TERMINATED" &&
           parse_seconds_ms(fields[2], timestamp_ms);
}

class JsonCursor {
public:
    explicit JsonCursor(std::string_view text) : text_(text) {}

    void skip_whitespace()
    {
        while (position_ < text_.size() && std::isspace(static_cast<unsigned char>(text_[position_]))) {
            ++position_;
        }
    }

    bool consume(char expected)
    {
        skip_whitespace();
        if (position_ >= text_.size() || text_[position_] != expected) {
            return false;
        }
        ++position_;
        return true;
    }

    bool parse_string(std::string_view& value)
    {
        skip_whitespace();
        if (position_ >= text_.size() || text_[position_++] != '"') {
            return false;
        }
        const size_t start = position_;
        while (position_ < text_.size() && text_[position_] != '"') {
            const unsigned char ch = static_cast<unsigned char>(text_[position_]);
            if (ch < 0x20 || ch == '\\') {
                return false;
            }
            ++position_;
        }
        if (position_ == text_.size()) {
            return false;
        }
        value = text_.substr(start, position_ - start);
        ++position_;
        return true;
    }

    bool parse_uint(uint32_t& value)
    {
        skip_whitespace();
        const size_t start = position_;
        while (position_ < text_.size() && text_[position_] >= '0' && text_[position_] <= '9') {
            ++position_;
        }
        return parse_uint_value(text_.substr(start, position_ - start), value);
    }

    bool at_end()
    {
        skip_whitespace();
        return position_ == text_.size();
    }

private:
    std::string_view text_;
    size_t position_ = 0;
};
}  // namespace

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

std::expected<void, Error> Sequence::Definition::set_duration(uint32_t duration_ms)
{
    if (duration_ms < duration_ms_) {
        return std::unexpected(Error::from_cause("sequence duration is shorter than its last event"));
    }
    duration_ms_ = duration_ms;
    return {};
}

std::expected<Sequence::Definition, Error> Sequence::Definition::parse_json(std::string_view text)
{
    if (text.empty() || text.size() > MAX_FILE_SIZE) {
        return std::unexpected(Error::from_cause("sequence JSON is empty or exceeds %u bytes", static_cast<unsigned>(MAX_FILE_SIZE)));
    }

    JsonCursor cursor{text};
    if (!cursor.consume('{')) {
        return std::unexpected(Error::from_cause("invalid JSON sequence document"));
    }

    Definition definition;
    bool has_events = false;
    bool has_duration = false;
    uint32_t declared_duration = 0;
    bool first_field = true;
    while (!cursor.consume('}')) {
        if (!first_field && !cursor.consume(',')) {
            return std::unexpected(Error::from_cause("invalid JSON sequence object separator"));
        }
        first_field = false;

        std::string_view key;
        if (!cursor.parse_string(key) || !cursor.consume(':')) {
            return std::unexpected(Error::from_cause("invalid JSON sequence field"));
        }
        if (key == "name") {
            std::string_view ignored_name;
            if (!cursor.parse_string(ignored_name)) {
                return std::unexpected(Error::from_cause("JSON sequence name must be a string"));
            }
        }
        else if (key == "duration_ms") {
            if (has_duration || !cursor.parse_uint(declared_duration)) {
                return std::unexpected(Error::from_cause("invalid or duplicate JSON duration_ms"));
            }
            has_duration = true;
        }
        else if (key == "events") {
            if (has_events || !cursor.consume('[')) {
                return std::unexpected(Error::from_cause("invalid or duplicate JSON events array"));
            }
            has_events = true;
            bool first_event = true;
            while (!cursor.consume(']')) {
                if (!first_event && !cursor.consume(',')) {
                    return std::unexpected(Error::from_cause("invalid JSON event array separator"));
                }
                first_event = false;
                if (!cursor.consume('{')) {
                    return std::unexpected(Error::from_cause("each JSON event must be an object"));
                }

                uint32_t event_time = 0;
                bool has_time = false;
                bool has_valve = false;
                bool has_state = false;
                std::string_view valve;
                std::string_view state;
                bool first_event_field = true;
                while (!cursor.consume('}')) {
                    if (!first_event_field && !cursor.consume(',')) {
                        return std::unexpected(Error::from_cause("invalid JSON event field separator"));
                    }
                    first_event_field = false;
                    std::string_view event_key;
                    if (!cursor.parse_string(event_key) || !cursor.consume(':')) {
                        return std::unexpected(Error::from_cause("invalid JSON event field"));
                    }
                    if (event_key == "time_ms") {
                        if (has_time || !cursor.parse_uint(event_time)) {
                            return std::unexpected(Error::from_cause("invalid or duplicate JSON time_ms"));
                        }
                        has_time = true;
                    }
                    else if (event_key == "valve" || event_key == "action") {
                        if (has_valve || !cursor.parse_string(valve)) {
                            return std::unexpected(Error::from_cause("invalid or duplicate JSON valve"));
                        }
                        has_valve = true;
                    }
                    else if (event_key == "state" || event_key == "value") {
                        if (has_state || !cursor.parse_string(state)) {
                            return std::unexpected(Error::from_cause("invalid or duplicate JSON state"));
                        }
                        has_state = true;
                    }
                    else {
                        return std::unexpected(Error::from_cause("unsupported JSON event field"));
                    }
                }
                if (!has_time || !has_valve || !has_state) {
                    return std::unexpected(Error::from_cause("JSON event is missing time_ms, valve, or state"));
                }
                if (auto result = definition.append(event_time, valve, state); !result) {
                    return std::unexpected(result.error());
                }
            }
        }
        else {
            return std::unexpected(Error::from_cause("unsupported JSON sequence field"));
        }
    }

    if (!cursor.at_end() || !has_events || definition.size() == 0) {
        return std::unexpected(Error::from_cause("invalid or empty JSON sequence"));
    }
    if (has_duration) {
        if (auto result = definition.set_duration(declared_duration); !result) {
            return std::unexpected(result.error());
        }
    }
    return definition;
}

std::expected<Sequence::Definition, Error> Sequence::Definition::parse_yaml(std::string_view text)
{
    if (text.empty() || text.size() > MAX_FILE_SIZE) {
        return std::unexpected(Error::from_cause("sequence YAML is empty or exceeds %u bytes", static_cast<unsigned>(MAX_FILE_SIZE)));
    }

    Definition definition;
    bool in_events = false;
    bool has_events = false;
    bool has_duration = false;
    bool has_time = false;
    bool has_valve = false;
    bool has_state = false;
    uint32_t time_ms = 0;
    uint32_t declared_duration = 0;
    std::string_view valve;
    std::string_view state;

    auto finish_event = [&]() -> std::expected<void, Error> {
        if (!has_time && !has_valve && !has_state) {
            return {};
        }
        if (!has_time || !has_valve || !has_state) {
            return std::unexpected(Error::from_cause("YAML sequence event is missing a required field"));
        }
        auto result = definition.append(time_ms, valve, state);
        has_time = false;
        has_valve = false;
        has_state = false;
        return result;
    };

    while (!text.empty()) {
        const size_t newline = text.find('\n');
        std::string_view line = trim(text.substr(0, newline));
        text = newline == std::string_view::npos ? std::string_view{} : text.substr(newline + 1);
        if (const size_t comment = line.find('#'); comment != std::string_view::npos) {
            line = trim(line.substr(0, comment));
        }
        if (line.empty()) {
            continue;
        }
        if (!in_events && line.starts_with("name:")) {
            continue;
        }
        if (!in_events && line.starts_with("duration_ms:")) {
            if (has_duration || !parse_uint_value(trim(line.substr(std::string_view{"duration_ms:"}.size())), declared_duration)) {
                return std::unexpected(Error::from_cause("invalid or duplicate YAML duration_ms"));
            }
            has_duration = true;
            continue;
        }
        if (line == "events:") {
            if (has_events) {
                return std::unexpected(Error::from_cause("duplicate YAML events collection"));
            }
            has_events = true;
            in_events = true;
            continue;
        }
        if (!in_events) {
            return std::unexpected(Error::from_cause("unsupported YAML sequence field"));
        }
        if (line.starts_with("- ")) {
            if (auto result = finish_event(); !result) {
                return std::unexpected(result.error());
            }
            line.remove_prefix(2);
        }

        const size_t separator = line.find(':');
        if (separator == std::string_view::npos) {
            return std::unexpected(Error::from_cause("invalid YAML sequence field"));
        }
        const std::string_view key = trim(line.substr(0, separator));
        std::string_view value = trim(line.substr(separator + 1));
        if (value.size() >= 2 && ((value.front() == '"' && value.back() == '"') || (value.front() == '\'' && value.back() == '\''))) {
            value = value.substr(1, value.size() - 2);
        }
        if (key == "time_ms") {
            if (has_time || !parse_uint_value(value, time_ms)) {
                return std::unexpected(Error::from_cause("invalid or duplicate YAML time_ms value"));
            }
            has_time = true;
        }
        else if (key == "valve" || key == "action") {
            if (has_valve) {
                return std::unexpected(Error::from_cause("duplicate YAML valve field"));
            }
            valve = value;
            has_valve = true;
        }
        else if (key == "state" || key == "value") {
            if (has_state) {
                return std::unexpected(Error::from_cause("duplicate YAML state field"));
            }
            state = value;
            has_state = true;
        }
        else {
            return std::unexpected(Error::from_cause("unsupported YAML event field '%.*s'", static_cast<int>(key.size()), key.data()));
        }
    }

    if (auto result = finish_event(); !result) {
        return std::unexpected(result.error());
    }
    if (!has_events || !in_events || definition.size() == 0) {
        return std::unexpected(Error::from_cause("YAML sequence has no events"));
    }
    if (has_duration) {
        if (auto result = definition.set_duration(declared_duration); !result) {
            return std::unexpected(result.error());
        }
    }
    return definition;
}

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

std::expected<Sequence::Definition, Error> Sequence::Definition::load_file(std::string_view file_name, uint32_t run_index)
{
    if (file_name.empty() || file_name.size() > MAX_FILE_NAME_SIZE || file_name.find("..") != std::string_view::npos ||
        !std::all_of(file_name.begin(), file_name.end(), [](char ch) {
            return (ch >= 'a' && ch <= 'z') || (ch >= 'A' && ch <= 'Z') || (ch >= '0' && ch <= '9') || ch == '_' || ch == '-' || ch == '.';
        })) {
        return std::unexpected(Error::from_cause("sequence file name is invalid"));
    }

    const size_t extension_pos = file_name.find_last_of('.');
    if (extension_pos == std::string_view::npos) {
        return std::unexpected(Error::from_cause("sequence file must end in .json, .yaml, .yml, or .log"));
    }
    const std::string_view extension = file_name.substr(extension_pos);

    MutexGuard guard{&sequence_file_lock};
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

    if (extension == ".json") {
        return parse_json(content);
    }
    if (extension == ".yaml" || extension == ".yml") {
        return parse_yaml(content);
    }
    if (extension == ".log") {
        return parse_log(content, run_index);
    }
    return std::unexpected(Error::from_cause("unsupported sequence file extension"));
}

const Sequence::Event& Sequence::Definition::event(size_t index) const
{
    return events_[index];
}

size_t Sequence::Definition::size() const
{
    return event_count_;
}

uint32_t Sequence::Definition::duration_ms() const
{
    return duration_ms_;
}

std::span<const Sequence::Event> Sequence::Definition::events() const
{
    return {events_.data(), event_count_};
}

std::expected<void, Error> Sequence::Scheduler::load(Definition definition)
{
    if (definition.size() == 0) {
        return std::unexpected(Error::from_cause("cannot schedule an empty sequence"));
    }
    definition_ = std::move(definition);
    next_event_ = 0;
    last_elapsed_ms_ = 0;
    loaded_ = true;
    started_ = false;
    complete_ = false;
    return {};
}

std::expected<void, Error> Sequence::Scheduler::start()
{
    if (!loaded_) {
        return std::unexpected(Error::from_cause("no sequence is loaded"));
    }
    next_event_ = 0;
    last_elapsed_ms_ = 0;
    started_ = true;
    complete_ = false;
    return {};
}

void Sequence::Scheduler::reset()
{
    next_event_ = 0;
    last_elapsed_ms_ = 0;
    started_ = false;
    complete_ = false;
}

std::expected<Sequence::Tick, Error> Sequence::Scheduler::advance(uint32_t elapsed_ms)
{
    if (!started_) {
        return std::unexpected(Error::from_cause("sequence scheduler has not been started"));
    }
    if (elapsed_ms < last_elapsed_ms_) {
        return std::unexpected(Error::from_cause("sequence elapsed time must be monotonic"));
    }

    const size_t first_due = next_event_;
    while (next_event_ < definition_.size() && definition_.event(next_event_).time_ms <= elapsed_ms) {
        ++next_event_;
    }
    last_elapsed_ms_ = elapsed_ms;
    complete_ = next_event_ == definition_.size() && elapsed_ms >= definition_.duration_ms();
    return Tick{
        .due_events = definition_.events().subspan(first_due, next_event_ - first_due),
        .complete = complete_,
    };
}

bool Sequence::Scheduler::is_loaded() const
{
    return loaded_;
}

bool Sequence::Scheduler::is_running() const
{
    return started_ && !complete_;
}

bool Sequence::Scheduler::is_complete() const
{
    return complete_;
}

uint32_t Sequence::Scheduler::duration_ms() const
{
    return definition_.duration_ms();
}
