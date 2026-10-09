// Tests ignition-log parsing, volatile uploads, and scheduler event delivery.
#include "../../../../clover/src/ranger/Sequence.h"

#include <string_view>
#include <utility>
#include <zephyr/ztest.h>

/// Verifies run selection and normalization of event timestamps.
/// Parameters: fixture provides the test harness context.
/// Returns: nothing; assertions report test failures.
ZTEST(Sequence_tests, test_parse_log_selects_run_and_normalizes_event_times)
{
    constexpr std::string_view log =
        "[USER] IGNITION 10.000\r\n"
        "[IGNITION] PBV001 CLOSED -> OPEN 10.250\r\n"
        "[IGNITION] SV001 OPEN -> CLOSE 10.500\r\n"
        "[USER] IGNITION_TERMINATED 11.000\r\n"
        "[USER] IGNITION 20.000\r\n"
        "[IGNITION] PBV-002 CLOSED -> OPEN 20.125\r\n"
        "[USER] IGNITION_TERMINATED 20.750\r\n";

    auto definition = Sequence::Definition::parse_log(log, 1);
    zassert_true(definition.has_value(), "second complete ignition run should parse");
    zassert_equal(definition->size(), 1);
    zassert_equal(definition->duration_ms(), 750);
    zassert_equal(definition->event(0).time_ms, 125);
    zassert_true(std::string_view{definition->event(0).valve.data()} == "PBV002");
    zassert_true(definition->event(0).open);
}

/// Verifies incomplete ignition runs are rejected.
/// Parameters: fixture provides the test harness context.
/// Returns: nothing; assertions report test failures.
ZTEST(Sequence_tests, test_parse_log_rejects_missing_termination)
{
    constexpr std::string_view log =
        "[USER] IGNITION 1.000\n"
        "[IGNITION] PBV001 CLOSED -> OPEN 1.100\n";

    auto definition = Sequence::Definition::parse_log(log, 0);
    zassert_false(definition.has_value(), "unterminated ignition run should be rejected");
    (void)definition.error().build_message();
}

/// Verifies unsupported sequence file extensions are rejected.
/// Parameters: fixture provides the test harness context.
/// Returns: nothing; assertions report test failures.
ZTEST(Sequence_tests, test_load_file_rejects_non_log_extension)
{
    auto definition = Sequence::Definition::load_file("sequence.yaml");
    zassert_false(definition.has_value(), "non-log sequence files should be rejected");
    (void)definition.error().build_message();
}

/// Verifies a complete upload can be loaded from device memory.
/// Parameters: fixture provides the test harness context.
/// Returns: nothing; assertions report test failures.
ZTEST(Sequence_tests, test_uploaded_file_can_be_loaded_without_sd_card)
{
    constexpr std::string_view log =
        "[USER] IGNITION 1.000\n"
        "[IGNITION] PBV001 CLOSED -> OPEN 1.100\n"
        "[USER] IGNITION_TERMINATED 2.000\n";
    constexpr std::string_view file_name = "unit_test_upload.log";

    auto uploaded = Sequence::Definition::upload_chunk(file_name, 0, static_cast<uint32_t>(log.size()), log, true, 0);
    zassert_true(uploaded.has_value(), "complete sequence upload should succeed");
    auto definition = Sequence::Definition::load_file(file_name);
    zassert_true(definition.has_value(), "uploaded sequence should load without reading the SD card");
    zassert_equal(definition->size(), 1);
    zassert_equal(definition->event(0).time_ms, 100);
}

/// Verifies an ordered multi-chunk upload can be loaded.
/// Parameters: fixture provides the test harness context.
/// Returns: nothing; assertions report test failures.
ZTEST(Sequence_tests, test_uploaded_file_accepts_ordered_chunks)
{
    constexpr std::string_view log =
        "[USER] IGNITION 1.000\n"
        "[IGNITION] PBV001 CLOSED -> OPEN 1.100\n"
        "[USER] IGNITION_TERMINATED 2.000\n";
    constexpr std::string_view file_name = "unit_test_chunks.log";
    constexpr size_t split = 24;

    auto first_chunk = Sequence::Definition::upload_chunk(
        file_name, 0, static_cast<uint32_t>(log.size()), log.substr(0, split), false, 0);
    zassert_true(first_chunk.has_value(), "first sequence upload chunk should be accepted");
    auto last_chunk = Sequence::Definition::upload_chunk(
        file_name, split, static_cast<uint32_t>(log.size()), log.substr(split), true, 0);
    zassert_true(last_chunk.has_value(), "final sequence upload chunk should be accepted");
    auto definition = Sequence::Definition::load_file(file_name);
    zassert_true(definition.has_value(), "ordered chunks should produce a loadable file");
}

/// Verifies an upload is bound to its selected ignition run.
/// Parameters: fixture provides the test harness context.
/// Returns: nothing; assertions report test failures.
ZTEST(Sequence_tests, test_uploaded_file_uses_selected_run_index)
{
    constexpr std::string_view log =
        "[USER] IGNITION 1.000\n"
        "[IGNITION] PBV001 CLOSED -> OPEN 1.100\n"
        "[USER] IGNITION_TERMINATED 2.000\n"
        "[USER] IGNITION 3.000\n"
        "[IGNITION] PBV002 CLOSED -> OPEN 3.250\n"
        "[USER] IGNITION_TERMINATED 4.000\n";
    constexpr std::string_view file_name = "unit_test_run_index.log";

    auto uploaded =
        Sequence::Definition::upload_chunk(file_name, 0, static_cast<uint32_t>(log.size()), log, true, 1);
    zassert_true(uploaded.has_value(), "selected second run should upload successfully");
    auto selected_run = Sequence::Definition::load_file(file_name, 1);
    zassert_true(selected_run.has_value(), "uploaded run index should be loadable");
    zassert_equal(selected_run->event(0).time_ms, 250);
    auto different_run = Sequence::Definition::load_file(file_name, 0);
    zassert_false(different_run.has_value(), "a different run index requires a new upload");
}

/// Verifies upload chunks must arrive in order from offset zero.
/// Parameters: fixture provides the test harness context.
/// Returns: nothing; assertions report test failures.
ZTEST(Sequence_tests, test_upload_rejects_out_of_order_chunks)
{
    constexpr std::string_view file_name = "unit_test_order.log";
    constexpr std::string_view chunk = "data";
    auto uploaded = Sequence::Definition::upload_chunk(file_name, 1, 8, chunk, false, 0);
    zassert_false(uploaded.has_value(), "upload must begin at offset zero");
    (void)uploaded.error().build_message();
}

/// Verifies a malformed replacement does not discard the prior valid upload.
/// Parameters: fixture provides the test harness context.
/// Returns: nothing; assertions report test failures.
ZTEST(Sequence_tests, test_invalid_replacement_upload_keeps_previous_sequence)
{
    constexpr std::string_view valid_log =
        "[USER] IGNITION 1.000\n"
        "[IGNITION] PBV001 CLOSED -> OPEN 1.100\n"
        "[USER] IGNITION_TERMINATED 2.000\n";
    constexpr std::string_view invalid_log = "not a sequence\n";
    constexpr std::string_view file_name = "unit_test_replace.log";

    auto initial_upload =
        Sequence::Definition::upload_chunk(file_name, 0, static_cast<uint32_t>(valid_log.size()), valid_log, true, 0);
    zassert_true(initial_upload.has_value(), "initial valid sequence should be stored");
    auto replacement =
        Sequence::Definition::upload_chunk(file_name, 0, static_cast<uint32_t>(invalid_log.size()), invalid_log, true, 0);
    zassert_false(replacement.has_value(), "invalid replacement should be rejected");
    auto definition = Sequence::Definition::load_file(file_name);
    zassert_true(definition.has_value(), "rejected upload should leave the prior sequence available");
    zassert_equal(definition->size(), 1);
}

/// Verifies pending events persist until acknowledgment and completion follows duration.
/// Parameters: fixture provides the test harness context.
/// Returns: nothing; assertions report test failures.
ZTEST(Sequence_tests, test_scheduler_emits_due_events_and_completes_at_log_termination)
{
    constexpr std::string_view log =
        "[USER] IGNITION 1.000\n"
        "[IGNITION] PBV001 CLOSED -> OPEN 1.100\n"
        "[USER] IGNITION_TERMINATED 2.000\n";

    auto definition = Sequence::Definition::parse_log(log, 0);
    zassert_true(definition.has_value(), "valid ignition log should parse");

    Sequence::Scheduler scheduler;
    auto loaded = scheduler.load(std::move(*definition));
    zassert_true(loaded.has_value(), "parsed definition should load");
    auto started = scheduler.start();
    zassert_true(started.has_value(), "loaded sequence should start");

    auto before_event = scheduler.advance(99);
    zassert_true(before_event.has_value(), "scheduler should advance before the first event");
    zassert_equal(before_event->due_events.size(), 0);
    zassert_false(before_event->complete);

    auto at_event = scheduler.advance(100);
    zassert_true(at_event.has_value(), "scheduler should emit the event at its deadline");
    zassert_equal(at_event->due_events.size(), 1);
    zassert_true(at_event->due_events[0].open);
    zassert_false(at_event->complete);

    auto same_pending_event = scheduler.advance(1000);
    zassert_true(same_pending_event.has_value(), "unacknowledged event should remain pending");
    zassert_equal(same_pending_event->due_events.size(), 1);
    zassert_equal(same_pending_event->due_events[0].time_ms, 100);
    zassert_false(same_pending_event->complete);

    auto acknowledged = scheduler.acknowledge_event();
    zassert_true(acknowledged.has_value(), "successfully issued event should be acknowledged");
    zassert_true(scheduler.is_complete(), "sequence should complete after its final event and duration");

    auto at_termination = scheduler.advance(1000);
    zassert_true(at_termination.has_value(), "scheduler should advance to log termination");
    zassert_true(at_termination->complete);
}

/// Verifies overdue events are emitted one per tick in original order.
/// Parameters: fixture provides the test harness context.
/// Returns: nothing; assertions report test failures.
ZTEST(Sequence_tests, test_scheduler_emits_only_one_overdue_event_per_acknowledgment)
{
    constexpr std::string_view log =
        "[USER] IGNITION 1.000\n"
        "[IGNITION] PBV001 CLOSED -> OPEN 1.100\n"
        "[IGNITION] SV001 OPEN -> CLOSE 1.200\n"
        "[IGNITION] PBV002 CLOSED -> OPEN 1.300\n"
        "[USER] IGNITION_TERMINATED 2.000\n";

    auto definition = Sequence::Definition::parse_log(log, 0);
    zassert_true(definition.has_value(), "valid ignition log should parse");

    Sequence::Scheduler scheduler;
    auto loaded = scheduler.load(std::move(*definition));
    zassert_true(loaded.has_value(), "parsed definition should load");
    auto started = scheduler.start();
    zassert_true(started.has_value(), "loaded sequence should start");

    auto first_event = scheduler.advance(1000);
    zassert_true(first_event.has_value(), "scheduler should advance");
    zassert_equal(first_event->due_events.size(), 1);
    zassert_true(std::string_view{first_event->due_events[0].valve.data()} == "PBV001");

    auto still_first = scheduler.advance(1000);
    zassert_true(still_first.has_value(), "unacknowledged event should remain pending");
    zassert_equal(still_first->due_events.size(), 1);
    zassert_true(std::string_view{still_first->due_events[0].valve.data()} == "PBV001");
    auto first_acknowledged = scheduler.acknowledge_event();
    zassert_true(first_acknowledged.has_value(), "first event should be acknowledged");

    auto second_event = scheduler.advance(1000);
    zassert_true(second_event.has_value(), "next overdue event should be available");
    zassert_equal(second_event->due_events.size(), 1);
    zassert_true(std::string_view{second_event->due_events[0].valve.data()} == "SV001");
    auto second_acknowledged = scheduler.acknowledge_event();
    zassert_true(second_acknowledged.has_value(), "second event should be acknowledged");

    auto third_event = scheduler.advance(1000);
    zassert_true(third_event.has_value(), "last overdue event should be available");
    zassert_equal(third_event->due_events.size(), 1);
    zassert_true(std::string_view{third_event->due_events[0].valve.data()} == "PBV002");
}

/// Registers the sequence test suite.
/// Parameters: None.
/// Returns: nothing.
ZTEST_SUITE(Sequence_tests, NULL, NULL, NULL, NULL, NULL);
