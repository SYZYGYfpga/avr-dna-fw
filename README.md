SYZYGY DNA Firmware
===================

## Overview

This project contains a reference firmware implementing the SYZYGY DNA
functionality for SYZYGY peripherals. The firmware is designed to be run
on an Atmel ATTiny 44a microcontroller, though it may be adaptable to
other Atmel microcontrollers.

This firmware is written to comply with the SYZYGY Specification v1.0 along
with the SYZYGY DNA Specification v1.0.

### Features:

- SYZYGY DNA read/write compatible MCU Firmware
- Able to store up to 1KiB of DNA data
- Configurable power supply sequencing for peripherals


## Architecture

This firmware is designed around the use of multiple interrupts, allowing for
consistent operation even when the main running loop may require significant
computation.

Data received over I2C is loaded into a buffer in on-chip RAM. This data can
include the sub-address as well as any data that the host wishes to write
to the device memory. The buffered data is then queried by a routine running
on the Timer 0 interrupt. This routine checks the status of the I2C interface.
When the sub-address is received (the first two bytes of a transaction), the
Timer 0 interrupt routine begins to fill a transmit buffer used to send data
back to the host in the case of a read. In the case of a write, the Timer 0
interrupt routine will wait for the write transaction to complete, then copy
data from the RAM receive buffer to Flash memory.


## Power Supply Sequencing

This firmware includes logic to perform power supply sequencing if desired.

The sequencer measures a series of analog inputs and compares them with a
configurable threshold value. When the analog input passes the threshold
value, the supply on the analog input is considered operational.

Three digital outputs are provided to control power enable signals within the
system. Each enable output defaults to an inactive state. After initializing,
the output from the MCU will represent a digital high signal if configured as
active low, or a digital low signal if configured as active high. Pull-up or
pull-down resistors may be required to achieve to correct signal levels at power
on, prior to the MCU initialization.

Each enable can depend on any configuration of the states of the analog
inputs. Once all analog inputs that an enable depends on meet their threshold
the enable will remain in its initial inactive state for a configurable delay
time. Once the enable delay time passes the output will be set to an active
state representing a digital high signal if configured as active high or a
digital low signal if configured as active low.

Note that the MCU runs off a 3.3V supply. All digital "high" signals will
therefore output this voltage. Conversion to other voltages if necessary is
up to the peripheral designer.


### Sequencer Configuration

The configuration of the sequencer logic is kept in a 9-byte structure starting
at sub-address 0x9000. This data is stored in the internal EEPROM.

| Address   | Data                           |
| :-------- | :----------------------------- |
| `0x9000`  | Sequencer threshold 0          |
| `0x9001`  | Sequencer threshold 1          |
| `0x9002`  | Sequencer threshold 2          |
| `0x9003`  | Delay time for enable output 0 |
| `0x9004`  | Delay time for enable output 1 |
| `0x9005`  | Delay time for enable output 2 |
| `0x9006`  | Enable configuration 0         |
| `0x9007`  | Enable configuration 1         |
| `0x9008`  | Enable configuration 2         |

Threshold values represent an 8-bit ADC value that the analog input will be
compared to. These values are referenced from VCC (3.3V), so each increment of
the threshold represents an increment of 12mV.

Delay times are stored as a single byte value, scaled according to the
following rules:

- Delay values from 0 to 50 represent 0 to 0.5 seconds in 10ms steps
- Delay values from 51 to 55 represent 0.6 to 1.0 seconds in 100ms steps
- Delay values from 56 to 58 represent 1.5 to 2.5 seconds in 500ms steps

Delay values beyond 2.5 seconds will clip to 2.5 seconds.

The enable configuration bytes are bitfields in which each bit is used to
configure a different parameter. For each enable "i" the bitfield contains:

| Bit | Setting                                                    |
| :-- | :--------------------------------------------------------- |
| 0   | Enable[i] depends on analog input 0                        |
| 1   | Enable[i] depends on analog input 1                        |
| 2   | Enable[i] depends on analog input 2                        |
| 3   | Enable[i] is active high (0 = active high, 1 = active low) |
| 4   | Enable[i] is disabled (set to 1 to ignore this pin)        |


## Build Notes

### AtmelStudio

This firmware is written for use with AVR GCC 5.4 and AtmelStudio 7. To compile
the firmware in Atmel Studio, create a new project and add the .c files in the
`src` directory to the project. The project Toolchain settings must be modified
to include the `include` directory containing the firmware header files. With
the project configured correctly it should be possible to build and debug using
tools provided by Atmel Studio.

### CMake

Alternatively the firmware can be built using [CMake](https://cmake.org)
and any AVR GCC version. If [`avrdude`](https://www.nongnu.org/avrdude/)
is detected by CMake also targets for flashing (`flash`), writing
EEPROM (`eeprom`) and setting fuses (`fuse`) are available.

## Memory Usage

Flash Memory Usage:

- 0x000 - 0xBFF - Availalbe for user application
- 0xC00 - 0xFFF - Reserved for DNA storage

EEPROM Usage:

- 0x00 - 0xF7 - Available for use
- 0xF7 - 0xFF - Reserved for power supply sequencing


## Interrupt Usage

**USI START Condition Interrupt**: The USI START condition interrupt is
triggered when an I2C start condition is detected. It is used to configure
the USI counter interrupt and reset various aspects of the USI controller state.

**USI Counter Interrupt**: The USI Counter Interrupt triggers when the USI
data counter fills. This typically occurs once for each byte of data
transferred over I2C, as well as when an ACK or NACK is sent or received.

**Timer 0 Interrupt**: The interrupt routine for timer 0 checks the status of
the USI (I2C) portion of the design. As transactions are completed over I2C
this routine will check the sub-address value, write data to memory if
necessary, and perform reads on the memory, passing read data back to the USI
controller to send back to the host.

**Timer 1 Interrupt**: The 16-bit counter interrupt is used to track time in
milliseconds from the initialization of the timer peripheral. This is used
by the sequencer to determine when delay values are met.


## Pinout

The following pinout is assumed by default:

| Pin | Connection                |
| :-- | :------------------------ |
| PA0 | R\_GA                     |
| PA1 | Sequencer Analog Input 1  |
| PA2 | Sequencer Analog Input 2  |
| PA3 | Sequencer Analog Input 3  |
| PA4 | I2C SCL                   |
| PA5 | Reserved for programming  |
| PA6 | I2C SDA                   |
| PA7 | Unused                    |
| PB0 | Sequencer Enable Output 1 |
| PB1 | Sequencer Enable Output 2 |
| PB2 | Sequencer Enable Output 3 |
| PB3 | Reserved for programming  |
| VCC | 3.3V                      |


## Limitations

Due to the nature of the flash memory available in the AVR microcontroller,
writing new DNA data to the controller requires that the beginning of a page
be written to prior to filling in the rest of the page. Each page consists of
64 bytes. This means that a write to 0x800F must be preceeded by a write to
0x8000. As well, the memory writes are designed to only function on a single
page at a time, writing across page boundaries will result in undefined
behavior. Flash memory pages on the ATTiny44a are 64 bytes long. It is
recommended to make consecutive writes with a power of 2 length.

By default 1k of memory is reserved at the top of the flash memory space. This
limits the program memory space to the lower 3k of flash in the
microcontroller. Exceeding this will result in undefined behavior and likely
require a re-flash of the program on the controller.


## FUSE Settings

The following fuse settings are required for operation of the firmware:

| Register   | Value   | Details                                                         |
| :--------- | :------ | :-------------------------------------------------------------- |
| `EXTENDED` | `0xFE`  | `SELFPRGEN` - Enabled (allows use of the Flash memory for DNA)  |
| `HIGH`     | `0xDD`  | `CKDIV8` - Disabled (do not divide the internal clock by 8)     |
| `LOW`      | `0xE2`  | `BODLEVEL` - Set Brown-out detection at VCC=2.7V                |
