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

## Flash

Ensure the dev board is in bootloader mode (press PROGRAM), and that tycmd is installed on the host.
Setup, the PROGRAM-button step, and how to confirm a flash worked: see
[flasherd/README.md → Flashing Teensy boards](../flasherd/README.md#flashing-teensy-boards).

```shell
west flash --build-dir ~/arty/clover/build
```
