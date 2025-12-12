# bitcrane usbserial Firmware

This repository contains RP2040 USB firmware for the [bitcrane v3](https://github.com/skot/bitcrane/tree/RP2040). 

## Developing

Install Rust:

```Shell
curl --proto '=https' --tlsv1.2 -sSf https://sh.rustup.rs | sh

rustup target add thumbv6m-none-eabi

cargo install probe-rs-tools --locked
cargo install elf2uf2-rs --locked
cargo install cargo-binutils
```

For SWD-based development and debugging:

```Shell
# Build the latest firmware:
cargo build --release

# Build, program, and attach to the device:
cargo run --release

# Just flash the device, don't attach to RTT:
cargo flash --release --chip RP2040

# Erase all flash memory:
probe-rs erase --chip RP2040 --allow-erase-all
```

For UF2-based development:

```Shell
# Build the latest firmware:
cargo build --release

# Convert the ELF to an RP2040-compatible UF2 image:
elf2uf2-rs target/thumbv6m-none-eabi/release/firmware firmware.uf2

# Convert and deploy the UF2 image to an mounted RP2040:
elf2uf2-rs -d target/thumbv6m-none-eabi/release/firmware
```

## Running
When connected the bitcrane usbserial firmware will create two serial ports. Usually the first serial port is "control serial" like I2C, GPIO, ADC and LED. The second serial port is "data serial" and is pass through UART.

### Data Serial
- Second serial port
- All data is passed through, both directions.
- usbserial baudrate is mirrored on the output.


### Control Serial
- First serial port
- baudrate does not matter

**Packet Format**

| 0      | 1      | 2  | 3   | 4    | 5   | 6... |
|--------|--------|----|-----|------|-----|------|
| LEN LO | LEN HI | ID | BUS | PAGE | CMD | DATA |

```
0. length low
1. length high
	- packet length is number of bytes of the whole packet. 
2. command id
	- Whatever byte you want. will be returned in the response 
3. command bus
	- always 0x00 
4. command page
	- I2C:  0x05
	- GPIO: 0x06
	- LED:  0x08 
5. command 
	- varies by command page. See below
6. data
	- data to write. variable length. See below
```

**I2C**

Commands:

- write: 0x20
- read: 0x30
- readwrite: 0x40

Data:

- [I2C address, (bytes to write), (number of bytes to read)]

Example:

- write 0xDE to addr 0x4F: `08 00 01 00 05 20 4F DE`
- read one byte from addr 0x4C: `08 00 01 00 05 30 4C 01`
- readwrite two bytes from addr 0x32, reg 0xFE: `09 00 01 00 05 40 32 FE 02`

**GPIO**

Commands:

- RST0: 0x00
- PLUG0 (read-only): 0x01
- RST1: 0x02
- PLUG1 (read-only): 0x03
- RST2: 0x04
- PLUG2 (read-only): 0x05

Data:

- [pin level] (for set commands)
- none (for get commands)

Examples:

- Set RST0 High: `07 00 00 00 06 00 01`
- Set RST0 Low: `07 00 00 00 06 00 00`
- Get RST1: `06 00 00 00 06 02`
- Get PLUG0: `06 00 00 00 06 01`

**LED**

Commands:

- Set Color: 0x10

Data:

- [R, G, B]

Example:

- Set LED Magenta: `09 00 00 00 08 10 FF 00 FF`
