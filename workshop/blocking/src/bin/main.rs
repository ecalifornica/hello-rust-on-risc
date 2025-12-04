#![no_std]
#![no_main]
#![deny(
    clippy::mem_forget,
    reason = "mem::forget is generally not safe to do with esp_hal types, especially those \
    holding buffers for the duration of a data transfer."
)]

use defmt::info;
use esp_hal::clock::CpuClock;
use esp_hal::gpio::{Input, InputConfig, Level, Output, OutputConfig, Pull};
use esp_hal::main;
use esp_hal::time::{Duration, Instant};
use panic_rtt_target as _;

// Standard app descriptor for bootloader
esp_bootloader_esp_idf::esp_app_desc!();

#[main]
fn main() -> ! {
    // 1. Initialize RTT logging
    rtt_target::rtt_init_defmt!();

    // 2. Configure System
    let config = esp_hal::Config::default().with_cpu_clock(CpuClock::max());
    let peripherals = esp_hal::init(config);

    // 3. Configure LED (Output)
    let mut led = Output::new(peripherals.GPIO15, Level::Low, OutputConfig::default());

    let mut led2 = Output::new(peripherals.GPIO8, Level::Low, OutputConfig::default());

    // 4. Configure Button (Input)
    let button_config = InputConfig::default().with_pull(Pull::Up);
    let button = Input::new(peripherals.GPIO7, button_config);

    info!("System Ready. Starting blocking loop...");

    loop {
        // --- STEP A: Read Inputs ---
        // Check if the button is pressed (Logic Low)
        if button.is_low() {
            info!("BUTTON PRESSED!");
            led2.set_high();
        } else {
            info!("Button is released");
            led2.set_low();
        }

        // --- STEP B: Update Outputs ---
        led.toggle();

        // --- STEP C: The Blocking Delay ---
        // We pause execution for 2 seconds.
        // PROBLEM: The CPU is stuck in this 'while' loop.
        // Try pressing the button NOW. It will be ignored!
        info!("Blocking for 2 seconds...");

        let delay_start = Instant::now();
        while delay_start.elapsed() < Duration::from_secs(2) {
            // CPU is busy checking the time.
            // It cannot check the button here.
        }
    }
}
