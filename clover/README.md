# clover

> It's better to be lucky than good.

Embedded software for the GNC board.

## Build

```shell
west build ~/arty/clover --pristine auto --board tvc_throttle_dev/mimxrt1062 --build-dir ~/arty/clover/build
```

## Flash

Ensure the dev board is in bootloader mode, and that tycmd is installed.

```shell
west flash --build-dir ~/arty/clover/build
```
