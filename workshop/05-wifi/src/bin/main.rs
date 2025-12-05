#![no_std]
#![no_main]
#![deny(
    clippy::mem_forget,
    reason = "mem::forget is generally not safe to do with esp_hal types, especially those \
    holding buffers for the duration of a data transfer."
)]

use defmt::info;
use embassy_embedded_hal::shared_bus::asynch::i2c::I2cDevice;
use embassy_executor::Spawner;
use embassy_sync::blocking_mutex::raw::NoopRawMutex;
use embassy_sync::mutex::Mutex;
use embassy_time::{Duration, Timer};
use embedded_hal_async::i2c::I2c as I2cTrait;
use esp_hal::Async;
use esp_hal::clock::CpuClock;
use esp_hal::gpio::{Input, InputConfig, Level, Output, OutputConfig, Pull};
use esp_hal::i2c::master::{Config as I2cConfig, I2c};
use esp_hal::timer::timg::TimerGroup;
use panic_rtt_target as _;
use static_cell::StaticCell;

extern crate alloc;

// This creates a default app-descriptor required by the esp-idf bootloader.
// For more information see: <https://docs.espressif.com/projects/esp-idf/en/stable/esp32/api-reference/system/app_image_format.html#application-description>
esp_bootloader_esp_idf::esp_app_desc!();

// No-op because Embassy is single-threaded
type I2cBus = Mutex<NoopRawMutex, I2c<'static, Async>>;
// Bus inside static cell is initialized at runtime, after peripherals
static I2C_BUS: StaticCell<I2cBus> = StaticCell::new();

const MAX17048_ADDR: u8 = 0x36;
const MAX17048_VCELL: u8 = 0x02;
const MAX17048_SOC: u8 = 0x04;

#[embassy_executor::task]
async fn blink(mut led: Output<'static>, mut button: Input<'static>) {
    loop {
        button.wait_for_low().await;
        led.set_high();
        info!("Button pressed");

        button.wait_for_high().await;
        led.set_low();
        info!("Button released");
    }
}

#[embassy_executor::task]
async fn monitor_battery(i2c_bus: &'static I2cBus) {
    let mut i2c = I2cDevice::new(i2c_bus);

    // https://github.com/eldruin/max170xx-rs/blob/master/src/max170x8_x9.rs
    // voltage
    // f32::from(vcell) * 5.0 / 64000.0
    // percentage
    // f32::from(soc) / 256.0

    loop {
        let mut buf = [0u8; 2];

        let voltage = match i2c
            .write_read(MAX17048_ADDR, &[MAX17048_VCELL], &mut buf)
            .await
        {
            Ok(_) => {
                let vcell = u16::from_be_bytes(buf);
                (vcell as f32) * 5.0 / 64_000.0
            }
            Err(e) => {
                info!("Battery voltage read error: {:?}", e);
                continue;
            }
        };

        let percentage = match i2c
            .write_read(MAX17048_ADDR, &[MAX17048_SOC], &mut buf)
            .await
        {
            Ok(_) => {
                let state_of_charge = u16::from_be_bytes(buf);
                (state_of_charge as f32) / 256.0
            }
            Err(e) => {
                info!("Battery percentage read error: {:?}", e);
                continue;
            }
        };

        info!("Battery: {}V {}%", voltage, percentage);

        Timer::after(Duration::from_secs(5)).await;
    }
}

#[esp_rtos::main]
async fn main(spawner: Spawner) -> ! {
    // generator version: 1.0.1

    rtt_target::rtt_init_defmt!();

    let config = esp_hal::Config::default().with_cpu_clock(CpuClock::max());
    let peripherals = esp_hal::init(config);

    esp_alloc::heap_allocator!(#[esp_hal::ram(reclaimed)] size: 65536);

    let timg0 = TimerGroup::new(peripherals.TIMG0);
    let sw_interrupt =
        esp_hal::interrupt::software::SoftwareInterruptControl::new(peripherals.SW_INTERRUPT);
    esp_rtos::start(timg0.timer0, sw_interrupt.software_interrupt0);

    info!("Embassy initialized!");

    // GPIO config

    let mut led = Output::new(peripherals.GPIO15, Level::Low, OutputConfig::default());

    let led2 = Output::new(peripherals.GPIO8, Level::Low, OutputConfig::default());

    let button_config = InputConfig::default().with_pull(Pull::Up);
    let button = Input::new(peripherals.GPIO7, button_config);

    // I2C config

    let i2c_config = I2cConfig::default();
    let i2c0 = I2c::new(peripherals.I2C0, i2c_config)
        .unwrap()
        .with_scl(peripherals.GPIO18)
        .with_sda(peripherals.GPIO19)
        .into_async();
    // shared bus
    let i2c_bus = I2C_BUS.init(Mutex::new(i2c0));

    let mut i2c_power = Output::new(peripherals.GPIO20, Level::High, OutputConfig::default());

    // delay for power to stabilize
    Timer::after(Duration::from_millis(10)).await;

    // Radio

    let radio_init = esp_radio::init().expect("Failed to initialize Wi-Fi/BLE controller");
    let (mut _wifi_controller, _interfaces) =
        esp_radio::wifi::new(&radio_init, peripherals.WIFI, Default::default())
            .expect("Failed to initialize Wi-Fi controller");

    // Spawn tasks
    spawner.spawn(blink(led2, button)).unwrap();
    spawner.spawn(monitor_battery(i2c_bus)).unwrap();

    loop {
        info!("Hello world!");
        led.toggle();
        Timer::after(Duration::from_secs(2)).await;
    }

    // for inspiration have a look at the examples at https://github.com/esp-rs/esp-hal/tree/esp-hal-v1.0.0/examples/src/bin
}
