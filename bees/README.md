# bees

**B**ack **E**nd **E**mbedded **S**oftware... for just the feed system board.

## Build

```shell
west build ~/arty/bees --pristine auto --board dev_board/stm32h750xx --build-dir ~/arty/bees/build
```

## Flash

Ensure the dev board is in bootloader mode, and that STM32CubeProgrammer is installed.

```shell
west flash --build-dir ~/arty/bees/build --port usb1
```
