# AVR488

A [Prologix](http://prologix.biz)<sup>TM</sup> compatible<sub><sup>(ish)</sup></sub> USB to [IEEE 488/GPIB/HP-IB](https://en.wikipedia.org/wiki/IEEE-488) controller interface based on the ATmega168/328 microcontroller.

## Description

I designed this ATmega-based interface to allow me with to communicate with my various legacy Hewlett Packard (HP) test instruments. Although it is intended to support a USB interface, a serial EIA232 connection could also be implemented by deleting the USB interface IC and replacing it with an appropriate TTL to EIA232 driver.

The embedded code will run on either the ATmega168 or ATmega328 devices (there's is more than enough free flash in the 16 KB version).

## Getting Started

### Dependencies

* Requires [MicroChip Studio 7 for AVR devices](https://www.microchip.com/en-us/tools-resources/develop/microchip-studio) for building the firmware.
* Supports ATmega168 or ATmega328 devices.
* USB 2.0 interface is provided by a [MCP2221A](https://www.microchip.com/en-us/product/MCP2221A) USB to TTL IC.
* A direct serial EIA232 interface could be implemented by using a [MAX232](https://www.ti.com/product/MAX232) (or equivalent) IC.
* The fully level-compliant IEEE-488 interface is implemented using the [SN75160](https://www.ti.com/lit/gpn/sn75als160) and [SN75161](https://www.ti.com/lit/gpn/sn75als161) driver ICs.

### Building and Installing

1. Create a new project in MicroChip Studio and select the appropriate ATmega device.
2. Add the *.c and *.h files to the project.
3. Build the solution in MicroChip Studio.
4. Burn the code to your ATmega device.
5. Set the correct [fuses](Hardware/fuses.png):
   - Enable 8 MHz internal RC oscillator.
   - Disable divide by 8 oscillator divider.
   - Enable brown-out detector (to provide protection against EEPROM corruption).
7. Connect the USB interface to a host computer (38400 bps) and the IEEE-488 port to a device/instrument.
8. Upon first power-up, default settings will be stored in EEPROM. These can be updated and saved as required.
9. Enjoy!

## Hardware Setup

You will need to construct your own PCB hardware to support various ICs (ATmega168/328, MCP2221A and SN75160/161) and the appropriate USB and IEEE-488 interface connectors. A complete [schematic](Hardware/schematic.pdf) is provided for convenience. An example of [prototype](Hardware/prototype.jpg) hardware is also included.

To save pins, the internal 8 MHz RC oscillator is used as the clock source, so no external oscillator or crystal is required. 

The default baud rate for the ATmega serial port is 38400 bps.

## Command List

The command set is intended to be compatible with the [Prologix GPIB-USB (HPIB-USB Controller)](http://prologix.biz/gpib-usb-controller.html). **All commands are preceded by two plus signs ("++")**. Any input that is not preceded by ++ is passed directly to the connected device(s). Not all commands have been implemented, and some are not exactly the same as the Prologix interface. I've also added some helpful extensions for troubleshooting (debug and xdiag) and ease of use (echo).

**NOTE**: only controller mode is implemented (device mode is a possible future enhancement, but I have no need for it at this time).

```
addr [n]
```
Tell the interface which device to address (must be between 1 and 30). If no address is specified, the current device address will be displayed.

```
auto [n]
```
Enable (1) or disable (0) automatic read-after-write when a "?" is included in the command sent to the device. If no value is provided, the current auto setting will be displayed.

```
clr
```
Send a Select Device Clear (SDC) command to the currently addressed device.

```
debug [n]
```
Enable (1) or disable (0) debug logging messages. Convenient for troubleshooting problematic connections. If no value is provided, the current debug setting will be displayed.

```
echo [n]
```
Enable (1) or disable (0) echoing of characters received from USB port. Most IEEE-488 adapters require the sending application to enable local echo, but this is provided as an alternative. If no value is provided, the current echo setting will be displayed.

```
eoi [n]
```
Enable (1) or disable (0) asserting the EOI line when the last byte is sent. If no value is provided, the current eoi setting will be displayed.

```
eos [n]
```
Select the terminator characters to be appended to messages sent to the device: 0=CR+LF, 1=CR, 2=LF, 3=None. If no value is provided, the current eos setting will be displayed.

```
eot_char [n]
```
ASCII code of character to be appended to messages received from the device (when EOI is used). If no value is provided, the current eot_char setting will be displayed.

```
eot_enable [n]
```
Controls whether the eot_char is apprended to received messages. If no value is provided, the current eot_enable setting will be displayed.

```
help
```
Print a list of commands and explanations.

```
ifc
```
Issue an IEEE-488 interface clear by asserting the IFC line for 150 microseconds.

```
llo
```
Disable front panel operation of the currently addressed device by sending the Local Lockout (LLO) command.

```
loc
```
Enable front panel operation of the currently addressed device by sending the Go To Local (GTL) command.

```
mode [n]
```
Compatibility-only command - we are always a controller. Any value provided is ignored and queries always return mode 1.

```
read
```
Read from the currently addressed device until the eos terminating characters are received or a timeout occurs.

```
read eoi
```
Read from the currently addressed device until either EOI is asserted, the eos terminating characters are received, or a timeout occurs.

```
read_tmo_ms [n]
```
Sets the timeout value, in milliseconds, for the read and spoll commands. The default timeout is 1000 ms. If no value is provided, the current read_tmo_ms setting will be displayed.

```
rst
```
Performs a power-on reset of the controller interface by forcing a watchdog timer timeout. Takes approximately 2 seconds.

```
savecfg 1
```
Saves the current settings to EEPROM. Settings saved include: addr, eot_char, eot_enable, eos, eoi, auto, debug, and echo. Settings are **not** saved automatically when they are changed (to prevent unnecessary wear on the EEPROM). The value "1" must be included or the command is ignored.

```
spoll [n]
```
Read the device status byte by issuing a Serial Poll command. If no address is specified the serial poll request will be sent to the currently addressed device.

```
srq
```
Query the status of SRQ line. Returns 0 if the line is not asserted and 1 if asserted.

```
trg [n]
```
Issue a Group Execute Trigger (GET) to a device. If no address is specified the trigger will be sent to the currently addressed device.

```
ver
```
Query the current AVR488 firmware version.

```
xdiag
```
Show the current state of the ATmega IO pins (for troubleshooting purposes).


## Version History

* 1.0
    * Initial Release

## License

This project is licensed under the GPL-3.0 License - see the [LICENSE](LICENSE) file for details

## Acknowledgments

Inspiration, code snippets, libraries.
* [AR488](https://github.com/Twilight-Logic/AR488)
* [atmega-gpib](https://github.com/pepaslabs/atmega-gpib)
* [gpibusb-firmware](https://github.com/Galvant/gpibusb-firmware)
* [Peter Fleury's interrupt-driven UART library](http://www.peterfleury.epizy.com/doxygen/avr-gcc-libraries/group__pfleury__uart.html)
