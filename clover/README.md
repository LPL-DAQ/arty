# clover

> It's better to be lucky than good.

GNC embedded software.

## Build

```shell
west build ~/arty/clover --pristine auto --board tvc_throttle_dev --build-dir ~/arty/clover/build
```

## Flash

Ensure flasherd is running and the Teensy is plugged in to your computer and is in bootloader mode. Then, run:

```shell
west flash --build-dir ~/arty/clover/build
```
