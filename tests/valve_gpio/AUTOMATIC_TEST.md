# Automatic multimeter test

This firmware initializes every valve channel inactive, waits 15 seconds, then
holds channel 0 active for 60 seconds, then turns it off. It leaves
all outputs inactive afterward. It runs again each time the Teensy boots.
No serial console or network connection is required.

## Prepare

1. Disconnect valve loads and disable their power supply for the first test.
   This procedure checks the logic control output only.
2. Connect the Teensy by USB with a data cable. The two MCP23017 expanders must
   also have their normal logic power and ground; a bare Teensy without them
   cannot run this test. Use the board's documented logic power arrangement.
3. Set the multimeter to DC volts, autorange or a range above the logic supply.
   Put the black lead in COM and the red lead in the V socket, not the current
   socket. Do not use resistance or continuity mode on a powered board.
4. Identify board ground and MCP23017 expander 1 GPB0 from the schematic or
   labeled test points. Channel 0 maps to expander 1 GPIO 8, which is GPB0.
   This is not Teensy header pin 0 or physical IC package pin 8. If the board
   has no labels, consult its schematic before probing; the source cannot
   identify a physical connector or test point.

## Build and flash

In the Docker terminal, from `/workspaces/arty`:

```sh
west build -s tests/valve_gpio -b ranger_1/mimxrt1062 \
  -d tests/valve_gpio/build-auto -- -DEXTRA_CONF_FILE=automatic.conf
west flash -d tests/valve_gpio/build-auto
```

The project uses the same `tycmd_flasherd` runner as Clover. The Teensy must be
accessible to the existing host/container flashing setup. Follow its usual
bootloader-button procedure if prompted. A successful flash starts the test
when the firmware boots. If USB flashing fails, fix device access before
continuing; the test has not necessarily reached the board.

## Measure

1. With board power off, attach the black probe to board ground and position
   the red probe on expander 1 GPB0. Secure probes so they cannot bridge pins.
2. Power on the board normally. A power cycle starts the test again. The
   Teensy program button enters the bootloader; it is not a normal reset.
3. Watch the voltage for at least 75 seconds after initialization:

   | Time after GPIO initialization | Channel 0 |
   | --- | --- |
   | 0–15 s | Inactive |
   | 15–75 s | Active |
   | 75 s onward | Inactive |

   The current Ranger DTS uses `GPIO_ACTIVE_HIGH`, so expect approximately
   0 V while inactive and approximately the expander's logic supply voltage
   while active. Boot/device initialization adds a delay before this timeline.
   If you change the DTS to active-low, the voltage pattern reverses.
4. Confirm the selected output stays HIGH for 60 seconds and returns LOW. If it
   does not, check that flashing succeeded, that you selected the correct
   expander/pin, that its logic supply and ground are present, and that I2C
   wiring matches the board. Firmware does not pulse if any valve GPIO fails
   initialization. The interactive test's `valve list` can help diagnose it.
5. Remove test power before moving the probes or reconnecting loads. Restore
   normal firmware before returning the board to service.

To test another channel, edit `CONFIG_VALVE_TEST_CHANNEL` in `automatic.conf`,
rebuild, and flash again. See README.md for the expander channel mapping.

A changing logic pin verifies the GPIO control path. It does not verify valve
power, driver operation, or mechanical opening. Measuring a powered valve
output requires the schematic's output terminals and reference, a meter rated
for that supply, and the normal valve power setup. Do not connect valve supply
voltage to the Teensy's inputs. A firmware crash or failed I2C write can prevent
deactivation; the pulse duration is not a hardware shutdown guarantee.
