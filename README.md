# MPPTino

This C++ program manages an MPPT board. It is flashed on an ATmega328PB that orchestrates the board.
To do so, it interfaces with a LTC3813 boost controller, a INA180 current sense amplifier and a MCP4716 DAC.
There are a few states that can be used by the MPPT depending on conditions:
- At startup the solar panels have a high impedance, it is important to limit the power consumption as much as possible. The controller enters sleep mode.
- Sleep mode. The ATmega328PB is in power-down mode and wakes-up periodically to check the panel voltage against the minimum start-up voltage.
  If the voltage is OK, it then tries to start the LTC3813 boost controller and waits for positive power at the output.
  If power is produced, the MPPT enters active mode; otherwise sleep is resumed.
- Low power mode. When the power produced is below 5 watts, the tracking algorithm is disabled, the MPPT maintains a constant 28 volts.
- Tracking mode. When the power is sufficient, the tracking algorithm is enabled and tries to maintain maximum power in the range from 28 to 35 volts.

## RS-485 bus

This MPPT can communicate with an external computer using an RS-486 bus. The MPPT acts as a slave. It waits for a command frame to arrive and responds immediately if necessary.
A command frame is structured as follows:
`[0x4f, 0xc7, BOARD_ID, COMMAND, <PARAMETERS>]`. Parameters are sent as little-endian.

The two start bytes identify this protocol, `BOARD_ID` selects the MPPT to communicate with and
`COMMAND` is taken from the following list:
- `MAGIC`: Simply respond with the magic byte 0x42, used for sanity checking.
- `READ_ALL`: Read the latest averaged status of the MPPT. Returns an averaged PowerPoint over the last minute followed by a CRC8 using the SMBUS algorithm.
- `SET_MPP_MANUAL_DV`: Set a manual target power point voltage in decivolts using an `uint16` parameter.
- `SET_MPP_AUTO`: Reset to automatic power point tracking.
- `ENABLE_OUTPUT`: Enables power production.
- `DISABLE_OUTPUT`: Disables power production.

The PowerPoint returned by `READ_ALL` is structured as follows:
```cpp
struct PowerPoint {
  uint16_t vin_cv;  // Input voltage in centivolts
  uint16_t vout_dv;  // Output voltage in decivolts
  uint16_t iout_ca;  // Output current in centiamps
  uint16_t pout_dw;  // Output power in deciwatts
};
```

## Setting up the EEPROM

To identify this MMPT on the RS-485 bus, a unique ID has to be provided. This is done by writing the BOARD_ID byte at EEPROM address 0.

```cpp
#include <EEPROM.h>

#define BOARD_ID 1

void setup() {
  EEPROM.write(0, BOARD_ID);
}
void loop() {}
```

## Setting up the fuses

The correct fuse configuration is as follows:
- efuse is set to `0xf4`. This enables the BOD at 4.3 V.
- hfuse is set to `0xd7`. This preserves the EEPROM contents and disables the bootloader.
- lfuse is set to `0xc2`. This sets the internal RC oscillator as clock source and 8 MHz as frequency. This also enables fast startup.

## LCD

This MPPT can use a ST7565 compatible LCD screen to show its status (input and output voltages, output current and output power).
Add the [ST7565 library](https://github.com/antoine510/ST7565.git) to your arduino libraries, then set the USE_LCD macro in `MMPTino.ino` to 1.
