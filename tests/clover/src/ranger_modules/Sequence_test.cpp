#include "../../../../clover/src/ranger/Sequence.h"

#include <string_view>
#include <utility>
#include <zephyr/ztest.h>

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

ZTEST(Sequence_tests, test_parse_log_rejects_missing_termination)
{
    constexpr std::string_view log =
        "[USER] IGNITION 1.000\n"
        "[IGNITION] PBV001 CLOSED -> OPEN 1.100\n";

    auto definition = Sequence::Definition::parse_log(log, 0);
    zassert_false(definition.has_value(), "unterminated ignition run should be rejected");
    (void)definition.error().build_message();
}

ZTEST(Sequence_tests, test_load_file_rejects_non_log_extension)
{
    auto definition = Sequence::Definition::load_file("sequence.yaml");
    zassert_false(definition.has_value(), "non-log sequence files should be rejected");
    (void)definition.error().build_message();
}

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

    auto at_termination = scheduler.advance(1000);
    zassert_true(at_termination.has_value(), "scheduler should advance to log termination");
    zassert_true(at_termination->complete);
}

ZTEST_SUITE(Sequence_tests, NULL, NULL, NULL, NULL, NULL);
