# bitcrane usbserial Firmware

This repository contains RP2040 USB firmware for the [bitcrane v3](https://github.com/skot/bitcrane/tree/v3). 

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
When connected the bitcrane usbserial firmware will create four serial ports:
1. **Control Serial** - I2C, GPIO, FAN and LED commands
2. **ASIC UART0** - Passthrough UART on GPIO0 (TX) / GPIO1 (RX)
3. **ASIC UART1** - Passthrough UART on GPIO4 (TX) / GPIO5 (RX)
4. **ASIC UART2** - Passthrough PIO UART on GPIO6 (TX) / GPIO7 (RX)

### Control Serial (Port 1)
- Baudrate does not matter
- Packet-based protocol for device control

### ASIC UART Ports (Ports 2, 3 & 4)
- All data is passed through bidirectionally
- USB serial baudrate is mirrored to the UART output
- Supports standard UART baudrates

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
	- PSU:  0x04
	- I2C:  0x05
	- GPIO: 0x06
	- LED:  0x08 
	- Fan:  0x09
5. command 
	- varies by command page. See below
6. data
	- data to write. variable length. See below
```

**PSU** (Bit-banged I2C on GPIO14/GPIO15 at 400Hz)

Commands:

- write: 0x20
- read: 0x30
- readwrite: 0x40

Data:

- [I2C address, (bytes to write), (number of bytes to read)]

Example:

- write 0xDE to addr 0x4F: `08 00 01 00 04 20 4F DE`
- read one byte from addr 0x4C: `08 00 01 00 04 30 4C 01`
- readwrite two bytes from addr 0x32, reg 0xFE: `09 00 01 00 04 40 32 FE 02`

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
- RST1: 0x10
- PLUG1 (read-only): 0x11
- RST2: 0x20
- PLUG2 (read-only): 0x21
- PSU_EN: 0x50

Data:

- [pin level] (for set commands)
- none (for get commands)

Examples:

- Set RST0 High: `07 00 00 00 06 00 01`
- Set RST0 Low: `07 00 00 00 06 00 00`
- Get RST1: `06 00 00 00 06 10`
- Get PLUG0: `06 00 00 00 06 01`
- Set RST1 High: `07 00 00 00 06 10 01`
- Get PLUG1: `06 00 00 00 06 11`
- Set RST2 Low: `07 00 00 00 06 20 00`
- Get PLUG2: `06 00 00 00 06 21`
- Set PSU_EN High: `07 00 00 00 06 50 01`
- Set PSU_EN Low: `07 00 00 00 06 50 00`
- Get PSU_EN: `06 00 00 00 06 50`

**LED**

Commands:

- Set Color: 0x10

Data:

- [R, G, B]

Example:

- Set LED Magenta: `09 00 00 00 08 10 FF 00 FF`

**Fan**

Commands:

- set fan1 speed: 0x11
- get fan1 tachometer: 0x21
- set fan2 speed: 0x12
- get fan2 tachometer: 0x22
- set fan3 speed: 0x13
- get fan3 tachometer: 0x23
- set fan4 speed: 0x14
- get fan4 tachometer: 0x24

Data:

- [speed percentage 0-100] (for set speed commands)

Examples:

- Set fan1 speed to 50%:   `07 00 00 00 09 11 32`
- Set fan1 speed to 100%:  `07 00 00 00 09 11 64`
- Read fan1 tach (RPM):    `06 00 00 00 09 21`
- Set fan2 speed to 75%:   `07 00 00 00 09 12 4B`
- Read fan2 tach (RPM):    `06 00 00 00 09 22`
- Set fan3 speed to 60%:   `07 00 00 00 09 13 3C`
- Read fan3 tach (RPM):    `06 00 00 00 09 23`
- Set fan4 speed to 90%:   `07 00 00 00 09 14 5A`
- Read fan4 tach (RPM):    `06 00 00 00 09 24`

