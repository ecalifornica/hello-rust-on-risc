#![no_std]
#![no_main]
#![deny(
    clippy::mem_forget,
    reason = "mem::forget is generally not safe to do with esp_hal types, especially those \
    holding buffers for the duration of a data transfer."
)]

use core::net::Ipv4Addr;
use defmt::info;
use embassy_embedded_hal::shared_bus::asynch::i2c::I2cDevice;
use embassy_executor::Spawner;
use embassy_net::{Runner, Stack, StackResources, tcp::TcpSocket};
use embassy_sync::blocking_mutex::raw::NoopRawMutex;
use embassy_sync::mutex::Mutex;
use embassy_time::{Duration, Timer};
use embedded_hal_async::i2c::I2c as I2cTrait;
use embedded_io_async::Write;
use esp_hal::Async;
use esp_hal::clock::CpuClock;
use esp_hal::gpio::{Input, InputConfig, Level, Output, OutputConfig, Pull};
use esp_hal::i2c::master::{Config as I2cConfig, I2c};
use esp_hal::rng::Rng;
use esp_hal::timer::timg::TimerGroup;
use esp_radio::Controller;
use esp_radio::wifi::{
    ModeConfig,
    ScanConfig,
    WifiController,
    WifiDevice,
    WifiEvent,
    WifiStaState,
    ClientConfig,
};
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

macro_rules! mk_static {
    ($t:ty,$val:expr) => {{
        static STATIC_CELL: static_cell::StaticCell<$t> = static_cell::StaticCell::new();
        #[deny(unused_attributes)]
        let x = STATIC_CELL.uninit().write(($val));
        x
    }};
}
// Requires nightly
// macro_rules! make_static {
//     ($val:expr) => { ... };
//     ($val:expr, $(#[$m:meta])*) => { ... };
// }

// Set with env vars or .cargo/config.toml
const SSID: &str = env!("SSID");
const PASSWORD: &str = env!("PASSWORD");

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

#[embassy_executor::task]
async fn connection(mut controller: WifiController<'static>) {
    info!("Start connection task");
    loop {
        match esp_radio::wifi::sta_state() {
            WifiStaState::Connected => {
                controller.wait_for_event(WifiEvent::StaDisconnected).await;
                Timer::after(Duration::from_millis(5000)).await
            }
            _ => {}
        }
        if !matches!(controller.is_started(), Ok(true)) {
            let client_config = ModeConfig::Client(
                ClientConfig::default()
                    .with_ssid(SSID.into())
                    .with_password(PASSWORD.into()),
            );
            controller.set_config(&client_config).unwrap();
            controller.start_async().await.unwrap();
            info!("Wifi started!");

            info!("Scan");
            let scan_config = ScanConfig::default().with_max(10);
            let result = controller
                .scan_with_config_async(scan_config)
                .await
                .unwrap();
            for ap in result {
                info!("{:?}", ap);
            }
        }

        info!("About to connect...");
        match controller.connect_async().await {
            Ok(_) => info!("Wifi connected!"),
            Err(e) => {
                info!("Failed to connect: {:?}", e);
                Timer::after(Duration::from_millis(5000)).await
            }
        }
    }
}

#[embassy_executor::task]
async fn net_task(mut runner: Runner<'static, WifiDevice<'static>>) {
    runner.run().await
}

#[embassy_executor::task]
async fn http_get_task(stack: Stack<'static>) {
    let mut rx_buffer = [0; 4096];
    let mut tx_buffer = [0; 4096];

    info!("waiting for wifi link...");
    loop {
        if stack.is_link_up() { break; }
        Timer::after(Duration::from_millis(500)).await;
    }
    info!("wifi link up");

    info!("waiting for ip address...");
    loop {
        if let Some(config) = stack.config_v4() {
            info!("got ip: {}", config.address);
            break;
        }
        Timer::after(Duration::from_millis(500)).await;
    }

    loop {
        let mut socket = TcpSocket::new(stack, &mut rx_buffer, &mut tx_buffer);
        socket.set_timeout(Some(Duration::from_secs(10)));

        info!("connecting to URL");
        // let remote_endpoint = (Ipv4Addr::new(142, 250, 185, 115), 80);
        let remote_endpoint = (Ipv4Addr::new(104, 26, 6, 224), 80);

        if let Err(e) = socket.connect(remote_endpoint).await {
            info!("connect error: {:?}", e);
            Timer::after(Duration::from_secs(5)).await;
            continue;
        }

        info!("connected! sending GET...");
        // let request = b"GET / HTTP/1.0\r\nHost: www.mobile-j.de\r\n\r\n";
        let request = b"GET /echo HTTP/1.0\r\nHost: reqbin.com\r\n\r\n";

        let mut written = 0;
        while written < request.len() {
            match socket.write(&request[written..]).await {
                Ok(n) => written += n,
                Err(e) => {
                    info!("write error: {:?}", e);
                    break;
                }
            }
        }

        let mut buf = [0; 1024];
        match socket.read(&mut buf).await {
            Ok(0) => info!("read EOF (Server closed)"),
            Ok(n) => {
                let response = core::str::from_utf8(&buf[..n]).unwrap_or("<Binary Data>");
                info!("response received:\n{}", response);
            }
            Err(e) => info!("Read error: {:?}", e),
        };

        info!("waiting 10s before next request...");
        Timer::after(Duration::from_secs(10)).await;
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

    let radio_controller = mk_static!(
        esp_radio::Controller<'static>,
        esp_radio::init().expect("Failed to initialize Wi-Fi/BLE controller")
    );

    let (controller, interfaces) =
        esp_radio::wifi::new(radio_controller, peripherals.WIFI, Default::default())
            .expect("Failed to initialize Wi-Fi controller");

    let wifi_interface = interfaces.sta;

    let rng = Rng::new();  // no args
    let seed = (rng.random() as u64) << 32 | rng.random() as u64;

    // Initialize network stack
    let (stack, runner) = embassy_net::new(
        wifi_interface,
        embassy_net::Config::dhcpv4(Default::default()),
        mk_static!(StackResources<3>, StackResources::<3>::new()),
        seed,
    );

    // Spawn tasks
    spawner.spawn(blink(led2, button)).unwrap();
    spawner.spawn(monitor_battery(i2c_bus)).unwrap();
    spawner.spawn(connection(controller)).ok();
    spawner.spawn(net_task(runner)).ok();
    spawner.spawn(http_get_task(stack)).ok();

    loop {
        info!("Hello world!");
        led.toggle();
        Timer::after(Duration::from_secs(2)).await;
    }

    // for inspiration have a look at the examples at https://github.com/esp-rs/esp-hal/tree/esp-hal-v1.0.0/examples/src/bin
}
