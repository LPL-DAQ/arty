#include "../../../../clover/src/flight/FlightController.h"
#include <zephyr/ztest.h>




ZTEST(FlightController_tests, test_flight_tick_given_default_state)
{
    // Create a 2-second trace with all zeros (no movement)
    FlightLoadSequenceRequest req = FlightLoadSequenceRequest_init_default;

    // Create position traces: 2000 ms duration with linear segment from 0 to 0
    // X trace
    req.x_position_trace.total_time_ms = 2000;
    req.x_position_trace.segments[0].start_ms = 0;
    req.x_position_trace.segments[0].length_ms = 2000;
    req.x_position_trace.segments[0].which_type = Segment_linear_tag;
    req.x_position_trace.segments[0].type.linear.start_val = 0.0f;
    req.x_position_trace.segments[0].type.linear.end_val = 0.0f;

    // Y trace
    req.y_position_trace.total_time_ms = 2000;
    req.y_position_trace.segments[0].start_ms = 0;
    req.y_position_trace.segments[0].length_ms = 2000;
    req.y_position_trace.segments[0].which_type = Segment_linear_tag;
    req.y_position_trace.segments[0].type.linear.start_val = 0.0f;
    req.y_position_trace.segments[0].type.linear.end_val = 0.0f;

    // Z trace
    req.z_position_trace.total_time_ms = 2000;
    req.z_position_trace.segments[0].start_ms = 0;
    req.z_position_trace.segments[0].length_ms = 2000;
    req.z_position_trace.segments[0].which_type = Segment_linear_tag;
    req.z_position_trace.segments[0].type.linear.start_val = 0.0f;
    req.z_position_trace.segments[0].type.linear.end_val = 0.0f;

    // Roll trace
    req.roll_angle_trace.total_time_ms = 2000;
    req.roll_angle_trace.segments[0].start_ms = 0;
    req.roll_angle_trace.segments[0].length_ms = 2000;
    req.roll_angle_trace.segments[0].which_type = Segment_linear_tag;
    req.roll_angle_trace.segments[0].type.linear.start_val = 0.0f;
    req.roll_angle_trace.segments[0].type.linear.end_val = 0.0f;

    // Load traces
    auto load_result = FlightController::load_sequence(req);
    if (!load_result) {
        zassert_true(false, "Failed to load traces");
    }

    // Reset loop count so outer loop runs
    FlightController::set_loop_count_for_testing(0);

    // Set up DataPacket with known values
    DataPacket data = DataPacket_init_default;
    data.estimated_state.R_WB.qw = 1.0f;  // Identity quaternion
    data.estimated_state.R_WB.qx = 0.0f;
    data.estimated_state.R_WB.qy = 0.0f;
    data.estimated_state.R_WB.qz = 0.0f;
    // Position and velocity default to 0

    // Call flight_seq_tick at 500ms into the 2000ms sequence
    auto [out, seq_data] = FlightController::flight_seq_tick(data, 500, 0);

    // With zero desired state and zero estimated state, accelerations should be near zero
    zassert_within(out.pitch_angular_acceleration, 0.0f, 0.01f);
    zassert_within(out.yaw_angular_acceleration, 0.0f, 0.01f);
    zassert_within(out.z_acceleration, 0.0f, 0.01f);
    zassert_equal(out.roll_position, 0.0f);

    // Should remain in flight sequence state
    zassert_equal(out.next_state, FlightState_FLIGHT_STATE_FLIGHT_SEQ);
}

ZTEST(FlightController_tests, test_abort_tick_transitions_to_idle_after_500ms)
{
    auto [out, data] = FlightController::abort_tick(501, 0);
    zassert_equal(out.next_state, FlightState_FLIGHT_STATE_IDLE);
}

ZTEST(FlightController_tests, test_abort_tick_stays_in_abort_before_500ms)
{
    auto [out, data] = FlightController::abort_tick(499, 0);
    zassert_equal(out.next_state, FlightState_FLIGHT_STATE_ABORT);
}

ZTEST_SUITE(FlightController_tests, NULL, NULL, NULL, NULL, NULL);
