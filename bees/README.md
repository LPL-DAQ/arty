# bees

**B**ack **E**nd **E**mbedded **S**oftware... for just the feed system board.

## Build

Using the feed system board:

```shell
west build ~/arty/bees --pristine auto --board fs_board/stm32h750xx --build-dir ~/arty/bees/build
```

Using the Nucleo H755ZI-Q dev board: [currently broken]

```shell
west build ~/arty/bees --pristine auto --board dev_board --build-dir ~/arty/bees/build
```

## Flash

Ensure the dev board is in bootloader mode, and that STM32CubeProgrammer is installed.

```shell
west flash --build-dir ~/arty/bees/build --port usb1
```
