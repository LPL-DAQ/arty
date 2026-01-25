# clover

> It's better to be lucky than good.

GNC embedded software.

## Build

```shell
west build ~/arty/clover --pristine auto --board tvc_throttle_dev/mimxrt1062 --build-dir ~/arty/clover/build
```

## Flash

Ensure flasherd is running and the Teensy is plugged in to your computer and is in bootloader mode. Then, run:

```shell
west flash --build-dir ~/arty/clover/build
```

## Run unit tests

Unit tests are run in the dev container, so we must install a toolchain targeting the dev container ISA.

```shell
west sdk install -t x86_64-zephyr-elf
```
