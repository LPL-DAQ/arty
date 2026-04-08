# clover

> It's better to be lucky than good.

Embedded software for the Ranger.

## Build

For Hornet:

```shell
west build ~/arty/clover --pristine auto --board hornet_mk_3/mimxrt1062 --build-dir ~/arty/clover/build
```

## Flash

Ensure the dev board is in bootloader mode, and that tycmd is installed.

```shell
west flash --build-dir ~/arty/clover/build
```
