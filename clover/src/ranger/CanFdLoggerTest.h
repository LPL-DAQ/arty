#pragma once

namespace CanFdLoggerTest {

// Initialize CAN-FD in FD mode and install a catch-all receive filter.
// Call once before run().
void init();

// Block indefinitely, printing every received CAN-FD frame to the console.
// Decodes moteus telemetry fields (position, velocity, torque, mode, fault)
// when the frame looks like a moteus reply.
void run();

}  // namespace CanFdLoggerTest
