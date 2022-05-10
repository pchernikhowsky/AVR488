# AVR488

A [Prologix](http://prologix.biz)<sup>TM</sup> compatible<sub><sup>(ish)</sup></sub> USB to [IEEE 488/GPIB/HP-IB](https://en.wikipedia.org/wiki/IEEE-488) controller interface based on the ATmega168/328 microcontroller.
<p align="center">
  <img src="Hardware/hardware.jpg" alt="Completed hardware"/>
</p>

## Description

I designed the AVR488 interface so that I could communicate with my various legacy Hewlett Packard / Agilent test instruments. There are numerous Arduino-based variations of USB/IEEE 488 interfaces out there already, but I prefer programming AVR devices down at the "bare metal" level using C code, so I came up with this version. Since it's necessary to construct a physical interface for the IEEE 488 connector anyway, it's easy enough to add an ATmega and driver ICs into that hardware design. Although it is intended to support a USB interface, a serial EIA-232 connection could also be implemented by deleting the USB interface IC and replacing it with an appropriate TTL to EIA-232 driver.

The embedded code size is approximately 13.5 KB and uses about 360 bytes of RAM, so there's more than enough free memory in an ATmega168 device. I have not included a bootloader (it avoids annoying startup delays and I prefer programming the ATmega directly using an ISP interface). If desired, there are many bootloaders available that could first be flashed to the device and then used to upload the AVR488 firmware.

**NOTE**: only controller mode (`++mode 1`) is implemented at this time, as I had no need for device mode. The latter is a possible future enhancement.

## Getting Started

### Dependencies

AVR488 supports both ATmega168 or ATmega328 devices (both the 10 and 20 MHz variants).

* Requires [MicroChip Studio 7 for AVR devices](https://www.microchip.com/en-us/tools-resources/develop/microchip-studio) for building the firmware.
* An ISP programmer (AVRISP MKII, AVR JTAG ICE, etc.) is needed to program the flash memory and set the fuses.

### Hardware Interfaces
* A USB 2.0 interface is provided by an [MCP2221A](https://www.microchip.com/en-us/product/MCP2221A) USB to TTL IC for connection to a host device (computer).
* The fully level-compliant IEEE 488 interface is implemented using the [SN75160](https://www.ti.com/lit/gpn/sn75als160) and [SN75161](https://www.ti.com/lit/gpn/sn75als161) driver ICs for connection to IEEE 488 devices.
* A direct serial EIA-232 interface could be implemented by using a [MAX232](https://www.ti.com/product/MAX232) (or equivalent) IC.

### Building and Installing the Firmware

1. Create a new project in MicroChip Studio and select the appropriate ATmega device.
2. Add the *.c and *.h files to the project.
3. Build the solution in MicroChip Studio.
4. Burn the code to your ATmega device.
5. Set the correct [fuses](Hardware/fuses.png):
   - Enable 8 MHz internal RC oscillator.
   - Disable divide by 8 clock divider (CKDIV8).
   - Enable brown-out detection (to provide protection against EEPROM corruption).
7. Connect the USB interface to a host computer and the IEEE 488 port to a device/instrument.
8. Upon first power-up, default settings will be stored in EEPROM. Settings can be changed and saved to EEPROM as required.
9. Enjoy!

When the AVR488 is first connected to a computer it will create a new USB serial port. The internal baud rate for the AVR488 serial port is fixed at 38400 bps (this can only be altered by changing a #define in the source code and reflashing). Thus, you MUST set the baud rate in your PC software (terminal program, etc.) to 38400 bps. A simple test to confirm that communications is fully working is to type the command "++help" (without the quotes). This should return a list available commands.

## Hardware Construction and Setup

You will need to construct your own hardware to support various ICs (ATmega168/328, MCP2221A, and SN75160/161) and the appropriate USB and IEEE 488 interface connectors. To make it easier, a full hardware design including [schematic](Hardware/AVR488-schematic.pdf), [bill of materials](Hardware/AVR488-BOM.csv), and PCB layout in KiCad format are provided. The PCB hardware and components have been selected to be hacker/hobbyist friendly; through-hole components are used exclusively. I ordered my PCBs from [JLCPCB](https://jlcpcb.com/), but feel free to use your preferred vendor. The PCB has been designed to fit inside a [Hammond 1593BB](https://www.hammfg.com/electronics/small-case/plastic/1593.pdf) plastic enclosure. A [dimensioned label](Hardware/AVR488-label.pdf) (including LED drilling dimensions) is also provided.

For simplicity, and to save pins, the internal 8 MHz RC oscillator in the ATmega168/328 is used as the clock source, so no external oscillator or crystal is required.

For diagnostic purposes, three LED outputs are provided:
* TE (Talk Enable): flashes when the TE signal to the IEEE 488 drivers goes high for driving data onto the bus.
* SRQ (Service Request): lights when the SRQ line is asserted, indicating that a device requires polling for status.
* STATUS: flashes during startup and then is continuously toggled when the watchdog timer is reset to indicate active processing (the toggling is around 130 kHz, so the LED is slightly dimmer than when fully on).

## Command List

The AVR488 command set is intended to be mostly compatible with the [Prologix GPIB-USB (HPIB-USB Controller)](http://prologix.biz/gpib-usb-controller.html). All commands are preceded by two plus signs (++). Any input not preceded by ++ is passed directly to the connected device(s). Input and commands from the host are buffered in a 254 byte buffer and then transmitted when either a CR (carriage return) or LF (line feed) character is received from the host. Not all Prologix commands have been implemented, and some do not operate exactly the same as the Prologix interface. Also, some extensions have been added for troubleshooting purposes (e.g., `debug` and `xdiag`) or for ease of use (e.g., `echo`).

Values enclosed in `[]` are optional. If a value is provided, the `[]` should not be included.

#### `++addr [n]`
Tell the interface which device to address (must be between 1 and 30). If no address is specified, the current device address will be displayed.

#### `++auto [n]`
Enable (1) or disable (0) automatic read-after-write when a "?" is included in the command sent to the device. If no value is provided, the current auto setting will be displayed.

#### `++clr`
Send a Select Device Clear (SDC) command to the currently addressed device.

#### `++debug [n]`
Enable simple (1), more verbose (2), or disable (0) debug logging messages. Convenient for troubleshooting problematic connections. If no value is provided, the current debug setting will be displayed.

#### `++default`
Emulates functionality of the AR488 command with the same name. The interface configuration is set with the following parameters:
* auto:	0
* eoi:	0 (disabled)
* eos:	0 (CR+LF)
* eot_enable:	0 (disabled)
* eot_char:	0
* address: 0
* read_tmo_ms:	1200
* echo: 0
* debug: 0

#### `++echo [n]`
Enable (1) or disable (0) echoing of characters received from USB port. Most IEEE 488 adapters require the sending application to enable local echo, but this is provided as an alternative. If no value is provided, the current echo setting will be displayed.

#### `++eoi [n]`
Enable (1) or disable (0) asserting the EOI line when the last byte is sent. If no value is provided, the current eoi setting will be displayed.

#### `++eos [n]`
Select the terminator characters to be appended to messages sent to the device: 0=CR+LF, 1=CR, 2=LF, 3=None. If no value is provided, the current eos setting will be displayed.

#### `++eot_char [n]`
ASCII code of character to be appended to messages received from the device (when EOI is used). If no value is provided, the current eot_char setting will be displayed.

#### `++eot_enable [n]`
Controls whether the eot_char is apprended to received messages. If no value is provided, the current eot_enable setting will be displayed.

#### `++help`
Print a list of commands and explanations.

#### `++ifc`
Issue an IEEE 488 interface clear by asserting the IFC line for 150 microseconds.

#### `++llo`
Disable front panel operation of the currently addressed device by sending the Local Lockout (LLO) command.

#### `++loc`
Enable front panel operation of the currently addressed device by sending the Go To Local (GTL) command.

#### `++mode [n]`
Compatibility-only command - we are always a controller. Any value provided is ignored and queries always return mode 1.

#### `++read`
Read from the currently addressed device until the eos terminating characters are received or a timeout occurs.

#### `++read eoi`
Read from the currently addressed device until either EOI is asserted, the eos terminating characters are received, or a timeout occurs.

#### `++read_tmo_ms [n]`
Sets the timeout value, in milliseconds, for the read and spoll commands. The default timeout is 1000 ms. If no value is provided, the current read_tmo_ms setting will be displayed.

#### `++rst`
Performs a power-on reset of the controller interface by forcing a watchdog timer timeout. Takes approximately 2 seconds.

#### `++savecfg 1`
Saves the current settings to EEPROM. Settings saved include: addr, eot_char, eot_enable, eos, eoi, auto, debug, and echo. Settings are **not** saved automatically when they are changed (to prevent unnecessary wear on the EEPROM). The value "1" must be included or the command is ignored.

#### `++spoll [n]`
Read the device status byte by issuing a Serial Poll command. If no address is specified the serial poll request will be sent to the currently addressed device.

#### `++srq`
Query the status of SRQ line. Returns 0 if the line is not asserted and 1 if asserted.

#### `++trg [n]`
Issue a Group Execute Trigger (GET) to a device. If no address is specified the trigger will be sent to the currently addressed device.

#### `++ver`
Query the current AVR488 firmware version.

#### `++xdiag`
Show the current state of the ATmega IO pins (for troubleshooting purposes).


## Version History

* 1.1
    * Firmware changes to improve AR488 and Prologix compatibility.
* 1.0
    * Initial Release

## License

This project is licensed under the GPL-3.0 License. See the [LICENSE](LICENSE) file for details.

## Acknowledgments

Inspiration, code snippets, libraries.
* [AR488](https://github.com/Twilight-Logic/AR488)
* [atmega-gpib](https://github.com/pepaslabs/atmega-gpib)
* [gpibusb-firmware](https://github.com/Galvant/gpibusb-firmware)
* [Peter Fleury's interrupt-driven UART library](http://www.peterfleury.epizy.com/doxygen/avr-gcc-libraries/group__pfleury__uart.html)
