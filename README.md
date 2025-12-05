# Hello Rust

Published: 2025-09, Updated: 2025-12

## Introduction

Learning Rust by running it on RISC-V.

![esp32c6](/docs/images/hello_world.jpg)

2025 is a great time to give embedded Rust on RISC-V a try.

The Espressif `esp32c6` is well supported (Espressif hired a Rust team), the hardware abstraction layer [esp-hal](https://github.com/esp-rs/esp-hal) is 1.0.

See Links section for links to good web logs and vlogs.

---

## Hello World Workshop

### 0: Setup

- Goals:
  - `cargo run`

#### Bill of Materials

- [esp32c6 Adafruit Feather](https://www.adafruit.com/product/5933)
- [BME688](https://www.adafruit.com/product/5046)
- qwiic cables
- battery

#### Install toolchain

```bash
# dialout (btw)
sudo usermod -a -G uucp $USER

rustup update

# Cross compilation target. The a is for atomic.
rustup target add riscv32imac-unknown-none-elf

# Install nightly for a heap-less allocation, the network stack needs it.
rustup toolchain install nightly --component rust-src --component rust-analyzer

# Rust rewrite of Python flash tool, --locked is not transitive.
cargo install espflash cargo-espflash --locked

```

- details
  - [esp rs book](https://docs.esp-rs.org/book/installation/riscv.html)
    - [components](https://rust-lang.github.io/rustup/concepts/components.html)
      - [toolchains](https://rust-lang.github.io/rustup/concepts/toolchains.html)

#### Set up debug probe

[probe-rs](https://probe.rs/docs/getting-started/probe-setup)

```bash
# Instead of sudo, add rules to /etc/udev/rules.d/
udevadm control --reload
```

```bash
cargo install probe-rs-tools --locked
probe-rs complete install
```

#### Generate project from template

<https://github.com/esp-rs/esp-generate>

See also:

- <https://docs.espressif.com/projects/rust/book/getting-started/using-esp-generate.html>

```bash
cargo install esp-generate --locked
```

```bash
esp-generate \
  --chip esp32c6 \
  --headless \
  hello-world
```

### 1 Hello Blink

- Goals:
  - blink
  - print hello world

```bash
# build and flash
cargo run
```

[img: hello blink]

**Hello world!**

```bash
# monitor serial output
probe-rs attach --chip esp32c6 target/riscv32imac-unknown-none-elf/debug/hello-rust
```

### 2 Read Inputs

- Goals:
  - read input, button
  - attempt to blink and read button
  - understand why we want async

Create a handle to the bus that the peripheral can consume.

What is a `NoopRawMutex`? <https://github.com/embassy-rs/embassy/issues/4034#issuecomment-2774951121>

```rust
#[embassy_executor::task]
async fn i2c_reader_task(i2c_bus: &'static Mutex<NoopRawMutex, I2c<'static, esp_hal::Async>>) {

```

Should I use <https://github.com/Rahix/shared-bus>? No.

References:

- <https://github.com/rust-embedded/embedded-hal/tree/master/embedded-hal-bus>
- <https://github.com/embassy-rs/embassy/blob/main/embassy-embedded-hal/src/shared_bus/mod.rs>
- <https://github.com/esp-rs/esp-hal/blob/main/esp-lp-hal/examples/i2c.rs>
- <https://github.com/esp-rs/esp-hal/blob/main/esp-hal/src/i2c/master/mod.rs>
- for the rp2350: <https://github.com/embassy-rs/embassy/blob/main/examples/rp/src/bin/shared_bus.rs>

### 3 Async with the Embassy Framework

- Goals:
  - embassy
  - multitasking, blink and read button

### 4 Read Sensors

- Goals:
  - read sensors
  - explore ecosystem

### 5 Wifi (IoT)

- Goals:
  - get
    - HTTP
  - post sensor data
    - MQTT
  - Adaftuit IO

<https://github.com/esp-rs/esp-hal/blob/main/examples/src/bin/wifi_embassy_dhcp.rs>

<https://github.com/rust-embedded-community/embedded-nal/tree/master/embedded-nal-async>

<https://github.com/drogue-iot/reqwless>

See also:

- <https://github.com/ivmarkov/edge-net>

---

## Beyond Hello

### 6 testing

goals:

- testing framework
- hardware in the loop

### 7 HAL

goals:

- use another board (rp2350)

### 8 Etc

#### Record sensor readings to low power memory

RTC RAM powered during low power modes.

References:

- <https://github.com/esp-rs/esp-hal/blob/main/examples/src/bin/flash_read_write.rs>
- <https://github.com/esp-rs/esp-hal/blob/main/esp-storage/src/esp32c6.rs>
- Ring buffer for volatile memory
  - <https://github.com/rust-embedded/heapless>
  - See also:
    - <https://github.com/embassy-rs/embassy/blob/main/embassy-sync/src/ring_buffer.rs>
- Ring buffer for non-volatile memory
  - <https://github.com/tweedegolf/sequential-storage>
- Serialization
  - <https://github.com/jamesmunns/postcard>
  - <https://github.com/dtolnay/miniserde>

---

## How-to

### Pull firmware for later analysis

```bash
espflash board-info
espflash read-flash 0x0 0x400000 esp32c6_backup.bin

```

### Monitor serial without flashing

```bash
probe-rs attach --chip esp32c6 target/riscv32imac-unknown-none-elf/debug/hello-rust
```

### Check memory usage

```bash
cargo install cargo-binutils
rustup component add llvm-tools
```

```bash
cargo size
```

```bash
cargo size --release -- -A

hello-rust  :
section                     size         addr
.defmt                        69          0x0
.trap                       1144   0x40800000
.rwtext                     1652   0x40800478
.rwtext.wifi                   0   0x40800aec
.data                       1468   0x40800af0
.bss                       96684   0x408010b0
.noinit                        0   0x40818a5c
.data.wifi                   220   0x40818a5c
.rodata                     9424   0x42000020
.rodata.wifi                   0   0x420024f0
.text_gap                     36   0x420024f0
.text                      54052   0x42002514
.rtc_fast.text                 0   0x50000000
.rtc_fast.data                 0   0x50000000
.rtc_fast.bss                  0   0x50000000
.rtc_fast.persistent           0   0x50000000
.stack                    350936   0x40818b38
.eh_frame                  10508          0x0
.espressif.metadata            7          0x0
.debug_loc                147828          0x0
.debug_abbrev              11228          0x0
.debug_info               907419          0x0
.debug_aranges             24432          0x0
.debug_ranges              45904          0x0
.debug_str               1307403          0x0
.comment                     235          0x0
.riscv.attributes             92          0x0
.debug_line               183716          0x0
.debug_frame                 784          0x0
Total                    3155241
```

### Read boot log

```bash
ESP-ROM:esp32c6-20220919
Build:Sep 19 2022
rst:0x15 (USB_UART_HPSYS),boot:0x8 (SPI_FAST_FLASH_BOOT)
Saved PC:0x408030de
0x408030de - mac_tx_set_plcp0
    at ??:??
SPIWP:0xee
mode:DIO, clock div:2
load:0x4086c410,len:0xd48
load:0x4086e610,len:0x2d68
load:0x40875720,len:0x1800
entry 0x4086c410
I (23) boot: ESP-IDF v5.1-beta1-378-gea5e0ff298-dirt 2nd stage bootloader
I (23) boot: compile time Jun  7 2023 08:02:08
I (24) boot: chip revision: v0.1
I (28) boot.esp32c6: SPI Speed      : 40MHz
I (33) boot.esp32c6: SPI Mode       : DIO
I (37) boot.esp32c6: SPI Flash Size : 4MB
I (42) boot: Enabling RNG early entropy source...
I (48) boot: Partition Table:
I (51) boot: ## Label            Usage          Type ST Offset   Length
I (58) boot:  0 nvs              WiFi data        01 02 00009000 00006000
I (66) boot:  1 phy_init         RF data          01 01 0000f000 00001000
I (73) boot:  2 factory          factory app      00 00 00010000 003f0000
I (81) boot: End of partition table
I (85) esp_image: segment 0: paddr=00010020 vaddr=42000020 size=0b8f8h ( 47352) map
I (103) esp_image: segment 1: paddr=0001b920 vaddr=40800000 size=00014h (    20) load
I (104) esp_image: segment 2: paddr=0001b93c vaddr=4200b93c size=3eadch (256732) map
I (163) esp_image: segment 3: paddr=0005a420 vaddr=40800014 size=0cb10h ( 51984) load
I (176) esp_image: segment 4: paddr=00066f38 vaddr=40827338 size=000ech (   236) load
I (179) boot: Loaded app from partition at offset 0x10000
```

### Create a partition table entry for data

<https://docs.espressif.com/projects/esp-idf/en/latest/esp32/api-guides/partition-tables.html#partition-tables>
<https://docs.rust-embedded.org/embedonomicon/memory-layout.html>

### Link RTC memory

```rust
#![no_std]
use esp_hal::rtc_cntl::Rtc;

// 16KB RTC SRAM on esp32c6
#[link_section = ".rtc.data"]
static mut SENSOR_BUFFER: [u32; 1000] = [0; 1000];

#[link_section = ".rtc.data"]
static mut BUFFER_INDEX: usize = 0;

fn store_sensor_data(value: u32) {
    unsafe {
        if BUFFER_INDEX < SENSOR_BUFFER.len() {
            SENSOR_BUFFER[BUFFER_INDEX] = value;
            BUFFER_INDEX += 1;
        }
    }
}
```

---

## Links

[adafruit board docs](https://learn.adafruit.com/adafruit-esp32-c6-feather)

[esp-hal examples](https://github.com/esp-rs/esp-hal/tree/main/examples)

[esp rust dev](https://mabez.dev/blog/posts/)

vlogs by <https://github.com/therustybits>

- <https://www.youtube.com/watch?v=vT4-bvHCbE0>

---

## Glossary

- bsc: board support crate
- hal: hardware abstraction layer
- pac: peripheral access crate
  - <https://github.com/esp-rs/esp-pacs>
