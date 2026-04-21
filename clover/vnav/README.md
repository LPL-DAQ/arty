# vnav - VN-300 IMU on Teensy 4.1 Microcontroller with Zephyr RTOS

Vectornav VN-300 embedded software for UART communication

## Wiring

| VN-300 | Teensy 4.1  |
|--------|-------------|
| TX     | Pin 15 (RX) |
| RX     | Pin 14 (TX) |
| GND    | GND         |
| 3.3V   | 3.3V        |

## Build

```shell
west build --pristine -b vn300_teensy /workspaces/arty/clover/vnav
```

## Flash

```shell
west flash
```

## Output for Mac (run in Terminal)

```bash
minicom -b 115200 -D /dev/tty.usbmodem[PORT]
```

## Output for Windows (run in Command Line)

```shell
wsl --install
sudo apt update
sudo apt install minicom
minicom -b 115200 -D /dev/ttyS3
```
                              ^ port # might need to be changed

bypass command if issues:

```shell
sudo chmod 666 /dev/ttyS3
```