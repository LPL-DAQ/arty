# Valve sequencing architecture

Named sequences and manual GPIO control are part of the Ranger 1 Clover build.
The implementation, mapping, examples, and operational instructions are in
[`clover/src/sequencing`](../clover/src/sequencing/README.md). The Clover TCP
server uses the shared protobuf protocol and dispatches to the same engine as
the USB shell. The sequence engine owns GPIO writes, serializes manual and
sequence control, and reports faults and logical readback.

The JSON inputs are validated and converted into immutable firmware tables at
build time. Controller telemetry uses cached GPIO readback to avoid I2C work in
the high-priority control loop; explicit client status requests read the GPIOs.
The client UI and the board share the channel mapping file.

Host tests are in `tests/clover/sequencing`. The TCP interoperability tests
use a test-only transport harness; they exercise the engine, protocol and client,
not the production Clover server or real Ethernet hardware. See the
[sequencing guide](../clover/src/sequencing/README.md) for commands, mapping,
control behavior, and test instructions.
