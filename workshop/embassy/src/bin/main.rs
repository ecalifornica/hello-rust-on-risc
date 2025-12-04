#![no_std]
#![no_main]
#![deny(
    clippy::mem_forget,
    reason = "mem::forget is generally not safe to do with esp_hal types, especially those \
    holding buffers for the duration of a data transfer."
)]

use defmt::info;
use embassy_executor::Spawner;
use embassy_time::{Duration, Instant, Timer};
use esp_hal::clock::CpuClock;
use esp_hal::gpio::{Input, InputConfig, Level, Output, OutputConfig, Pull};
use esp_hal::timer::timg::TimerGroup;
use panic_rtt_target as _;

// This creates a default app-descriptor required by the esp-idf bootloader.
// For more information see: <https://docs.espressif.com/projects/esp-idf/en/stable/esp32/api-reference/system/app_image_format.html#application-description>
esp_bootloader_esp_idf::esp_app_desc!();

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

#[esp_rtos::main]
async fn main(spawner: Spawner) -> ! {
    // generator version: 1.0.1

    rtt_target::rtt_init_defmt!();

    let config = esp_hal::Config::default().with_cpu_clock(CpuClock::max());
    let peripherals = esp_hal::init(config);

    let timg0 = TimerGroup::new(peripherals.TIMG0);
    let sw_interrupt =
        esp_hal::interrupt::software::SoftwareInterruptControl::new(peripherals.SW_INTERRUPT);
    esp_rtos::start(timg0.timer0, sw_interrupt.software_interrupt0);

    info!("Embassy initialized!");

    let mut led = Output::new(peripherals.GPIO15, Level::Low, OutputConfig::default());

    let led2 = Output::new(peripherals.GPIO8, Level::Low, OutputConfig::default());

    let button_config = InputConfig::default().with_pull(Pull::Up);
    let button = Input::new(peripherals.GPIO7, button_config);

    // Spawn blink task
    spawner.spawn(blink(led2, button)).unwrap();

    loop {
        info!("Hello world!");
        led.toggle();
        Timer::after(Duration::from_secs(2)).await;

        // info!("Blocking for 2 seconds...");
        // let delay_start = Instant::now();
        // while delay_start.elapsed() < Duration::from_secs(2) {}
    }

    // for inspiration have a look at the examples at https://github.com/esp-rs/esp-hal/tree/esp-hal-v1.0.0/examples/src/bin
}
