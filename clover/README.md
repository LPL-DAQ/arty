# clover

> It's better to be lucky than good.

GNC embedded software.

## Build and Run

Ensure flasherd is running and the Teensy is plugged in to your computer and is in bootloader mode. Then, run:

```shell
west build -p auto clover -b tvc_throttle_dev
west flash
```
