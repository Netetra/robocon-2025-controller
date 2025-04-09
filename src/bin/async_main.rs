#![no_std]
#![no_main]

use core::cell::RefCell;

use alloc::boxed::Box;
use critical_section::Mutex;
use defmt::*;
use embassy_executor::Spawner;
use embassy_sync::{blocking_mutex::raw::CriticalSectionRawMutex, signal::Signal};
use embassy_time::{Duration, Instant, Timer};
use esp_hal::{
    clock::CpuClock,
    gpio::{InputPin, Level, OutputOpenDrain, OutputPin, Pull},
    peripheral::Peripheral,
    peripherals::Peripherals,
    prelude::*,
    rng::Rng,
    timer::timg::TimerGroup,
    uart::{self, Uart},
    Async,
};
use esp_wifi::esp_now::{EspNow, EspNowSender, BROADCAST_ADDRESS};
use heapless::Vec;
use robocon_rs::{
    components::gamepad::Gamepad,
    node::{command::Command, message::EspNowMessage},
    sbtp::Sbtp,
    util::sized_slice,
};
use static_cell::StaticCell;
use {defmt_rtt as _, esp_backtrace as _};

extern crate alloc;

const NODE_ID: u8 = 0xFE;
const ESPNOW_INTERVAL_MS: u64 = 20;
const LED_FLASH_TIME_MS: u64 = 5;

static GAMEPAD: Mutex<RefCell<Option<Gamepad>>> = Mutex::new(RefCell::new(None));

fn peripherals_init(cpu_clock: CpuClock) -> Peripherals {
    let mut config = esp_hal::Config::default();
    config.cpu_clock = cpu_clock;
    esp_hal::init(config)
}

fn led_init(
    spawner: &Spawner,
    led_pin: impl Peripheral<P = impl OutputPin + InputPin> + 'static,
) -> &'static Signal<CriticalSectionRawMutex, Instant> {
    static LED_CTRL: StaticCell<Signal<CriticalSectionRawMutex, Instant>> = StaticCell::new();
    let led_ctrl = &*LED_CTRL.init(Signal::new());
    let led = OutputOpenDrain::new(led_pin, Level::Low, Pull::Up);
    spawner.spawn(led_flash(led, led_ctrl)).unwrap();
    led_ctrl
}

#[embassy_executor::task]
async fn led_flash(
    mut led: OutputOpenDrain<'static>,
    control: &'static Signal<CriticalSectionRawMutex, Instant>,
) {
    let mut signal_time = Instant::now();
    loop {
        if control.signaled() {
            signal_time = control.wait().await;
        }
        if Instant::now().duration_since(signal_time) > Duration::from_millis(LED_FLASH_TIME_MS) {
            led.set_high();
        } else {
            led.set_low();
        }
        Timer::after(Duration::from_millis(1)).await;
    }
}

#[embassy_executor::task]
async fn gamepad_recv(
    mut sbtp: Sbtp<Uart<'static, Async>>,
    led: &'static Signal<CriticalSectionRawMutex, Instant>,
) {
    loop {
        if let Ok(payload) = sbtp.receive().await {
            critical_section::with(|cs| {
                led.signal(Instant::now());
                let mut gamepad = GAMEPAD.borrow_ref_mut(cs);
                let gamepad = gamepad.as_mut().unwrap();
                gamepad.update(&(sized_slice::<9>(&payload).unwrap().into()));
            });
        }
    }
}

#[embassy_executor::task]
async fn gamepad_send(mut esp_now_tx: EspNowSender<'static>) {
    loop {
        let raw_gamepad_data = critical_section::with(|cs| {
            let mut gamepad = GAMEPAD.borrow_ref_mut(cs);
            let gamepad = gamepad.as_mut().unwrap();
            let data = gamepad.into_array();
            gamepad.reset();
            data
        });
        let message = EspNowMessage::new(
            NODE_ID,
            0x00,
            Command::NotifyGamepadState,
            Vec::<u8, 247>::from_slice(&raw_gamepad_data).unwrap(),
        );
        let _ = esp_now_tx
            .send_async(&BROADCAST_ADDRESS, &message.into_esp_now_data())
            .await
            .unwrap();
        Timer::after(Duration::from_millis(ESPNOW_INTERVAL_MS)).await;
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

    let wifi_ctrl = Box::new(
        esp_wifi::init(
            timg0.timer0,
            Rng::new(peripherals.RNG),
            peripherals.RADIO_CLK,
        )
        .unwrap(),
    );
    let esp_now = EspNow::new(Box::leak(wifi_ctrl), peripherals.WIFI).unwrap();
    info!(
        "ESP Now initialized. version: {}",
        esp_now.version().unwrap()
    );
    let (_, esp_now_tx, _esp_now_rx) = esp_now.split();

    let uart_config = uart::Config::default().baudrate(115200);
    let mut uart = Uart::new(peripherals.UART0, peripherals.GPIO44, peripherals.GPIO43)
        .unwrap()
        .into_async();
    uart.apply_config(&uart_config).unwrap();

    let sbtp = Sbtp::new(uart);
    info!("SBTP initialized.");

    let led_ctrl = led_init(&spawner, peripherals.GPIO21);
    info!("LED Indicator initialized.");

    critical_section::with(|cs| {
        GAMEPAD.borrow_ref_mut(cs).replace(Gamepad::default());
    });

    spawner.spawn(gamepad_recv(sbtp, &led_ctrl)).unwrap();
    info!("Gamepad data receive task spawn.");

    spawner.spawn(gamepad_send(esp_now_tx)).unwrap();
    info!("Gamepad data send task spawn.");

    loop {
        Timer::after(Duration::from_millis(1)).await;
    }
}
