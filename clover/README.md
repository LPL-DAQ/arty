# clover

> It's better to be lucky than good.

Embedded software for the Ranger.

## Build

For Hornet:

```shell
west build ~/arty/clover --pristine auto --board hornet_mk_3/mimxrt1062 --build-dir ~/arty/clover/build
```

For Ranger:

```shell
west build ~/arty/clover --pristine auto --board ranger_1/mimxrt1062 --build-dir ~/arty/clover/build
```

For the May '26 hotfire variant:
```shell
west build ~/arty/clover --pristine auto --board tvc_throttle_dev/mimxrt1062 --build-dir ~/arty/clover/build
```

If an existing build directory predates these board settings, run the command
once with `--pristine always` so Zephyr discovers them.

On Ranger 1, this build also includes named GPIO sequences and persistent manual
valve control. It embeds all JSON files in `clover/src/sequencing/sequences/` using
`clover/src/sequencing/valve_map.json`; rebuild and flash after changing either.
The mapping follows the Ranger harness table (channel = GPIO_N − 1); see
`clover/src/sequencing/README.md`. The normal
Clover server accepts `vlvseq` commands on port 19690 alongside its existing
commands. Start `python scripts/client-new.py --no-data --host 169.254.99.99`
and enter `vlvseq`. In the client’s `status` live view, use the arrow keys
or `j`/`k` to scroll, `t` for telemetry, and `v` for valve readback.
Press `q` or Enter to exit.
Alternatively, use the USB shell's `sequence list`, `sequence run NAME`,
`sequence status`, and `sequence abort` commands. The engine starts with all
outputs inactive and requires an explicit run or manual command.

The sequence engine owns the valve GPIOs and also handles Clover's configured
valve actuation. Manual commands are rejected during a sequence. Abort requests
stop the GPIO engine and also trigger Clover's controller abort. GPIO faults
remain latched until reset. The USB `trace` command retains the latest 32 events
in Clover. Automatic log output to the USB shell is disabled on Ranger 1 to
keep synchronous logging from blocking network processing; use `trace` for
retained valve events. The command server opens after the valve engine initializes, even
if another subsystem fails to initialize. Controller-dependent commands are
rejected until the rest of Clover is ready; telemetry begins after startup. See [the valve sequencing guide](src/sequencing/README.md)
for sequence format, wiring and control behavior. Test with valve loads disconnected
first; GPIO readback does not confirm valve position.

## Flash

Ensure the dev board is in bootloader mode, and that tycmd is installed.

```shell
west flash --build-dir ~/arty/clover/build
```
