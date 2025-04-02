#![no_std]
#![no_main]

use defmt::*;
use embassy_executor::Spawner;
use embassy_sync::{blocking_mutex::raw::CriticalSectionRawMutex, signal::Signal};
use embassy_time::{Duration, Instant, Timer};
use esp_hal::{
    clock::CpuClock,
    gpio::{
        interconnect::{PeripheralInput, PeripheralOutput},
        Level, Output, OutputPin,
    },
    peripheral::Peripheral,
    peripherals::{Peripherals, TWAI0},
    prelude::*,
    rng::Rng,
    timer::timg::TimerGroup,
    twai::{
        filter::{Filter, SingleExtendedFilter},
        BaudRate, Twai, TwaiConfiguration, TwaiMode,
    },
    Async,
};
use esp_wifi::esp_now::EspNow;
use static_cell::StaticCell;
use {defmt_rtt as _, esp_backtrace as _};

extern crate alloc;

static LED_FLASH_TIME_MS: u64 = 20;

fn peripherals_init(cpu_clock: CpuClock) -> Peripherals {
    let mut config = esp_hal::Config::default();
    config.cpu_clock = cpu_clock;
    esp_hal::init(config)
}

fn twai_init<'a, RX: PeripheralInput, TX: PeripheralOutput>(
    twai: TWAI0,
    filter: impl Filter,
    baud_rate: BaudRate,
    mode: TwaiMode,
    rx_pin: impl Peripheral<P = RX> + 'a,
    tx_pin: impl Peripheral<P = TX> + 'a,
) -> Twai<'a, Async> {
    let mut config = TwaiConfiguration::new(twai, rx_pin, tx_pin, baud_rate, mode).into_async();
    config.set_filter(filter);
    let twai = config.start();
    twai
}

fn led_init(
    spawner: &Spawner,
    led_pin: impl Peripheral<P = impl OutputPin> + 'static,
) -> &Signal<CriticalSectionRawMutex, Instant> {
    static LED_CTRL: StaticCell<Signal<CriticalSectionRawMutex, Instant>> = StaticCell::new();
    let led_ctrl = &*LED_CTRL.init(Signal::new());
    let led = Output::new(led_pin, Level::Low);
    spawner.spawn(led_flash(led, led_ctrl)).unwrap();
    led_ctrl
}

#[embassy_executor::task]
async fn led_flash(
    mut led: Output<'static>,
    control: &'static Signal<CriticalSectionRawMutex, Instant>,
) {
    let mut signal_time = Instant::now();
    loop {
        if control.signaled() {
            signal_time = control.wait().await;
        }
        if Instant::now().duration_since(signal_time) > Duration::from_millis(LED_FLASH_TIME_MS) {
            led.set_low();
        } else {
            led.set_high();
        }
        Timer::after(Duration::from_millis(1)).await;
    }
}

#[main]
async fn main(spawner: Spawner) {
    let peripherals = peripherals_init(CpuClock::max());
    let timg0 = TimerGroup::new(peripherals.TIMG0);

    esp_alloc::heap_allocator!(72 * 1024);

    esp_hal_embassy::init(
        esp_hal::timer::systimer::SystemTimer::new(peripherals.SYSTIMER)
            .split::<esp_hal::timer::systimer::Target>()
            .alarm0,
    );
    info!("Embassy initialized.");

    let wifi_ctrl = esp_wifi::init(
        timg0.timer0,
        Rng::new(peripherals.RNG),
        peripherals.RADIO_CLK,
    )
    .unwrap();
    let esp_now = EspNow::new(&wifi_ctrl, peripherals.WIFI).unwrap();
    info!(
        "ESP Now initialized. version: {}",
        esp_now.version().unwrap()
    );

    let mut twai = twai_init(
        peripherals.TWAI0,
        SingleExtendedFilter::new(b"xxxxxxxxxxxxxxxxxxxxxxxxxxxxx", b"x"),
        BaudRate::B1000K,
        TwaiMode::Normal,
        peripherals.GPIO1,
        peripherals.GPIO2,
    );
    info!("TWAI initialized.");

    let led_ctrl = led_init(&spawner, peripherals.GPIO7);
    info!("LED Indicator initialized.");

    let _ = spawner;

    loop {
        Timer::after(Duration::from_millis(500)).await;
    }
}
