#![no_std]
#![no_main]

use alloc::{format, string::String};
use defmt::*;
use embassy_executor::Spawner;
use embassy_sync::{blocking_mutex::raw::CriticalSectionRawMutex, signal::Signal};
use embassy_time::{Duration, Instant, Timer};
use embedded_can::{Frame, Id};
use embedded_graphics::{
    mono_font::{MonoFont, MonoTextStyle},
    pixelcolor,
    prelude::*,
    text::{Baseline, Text},
};
use esp_hal::{
    clock::CpuClock,
    gpio::{
        interconnect::{PeripheralInput, PeripheralOutput},
        Level, Output, OutputPin,
    },
    i2c::{self, master::I2c},
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
use ssd1306::{
    mode::{self, BufferedGraphicsMode},
    prelude::*,
    I2CDisplayInterface, Ssd1306,
};
use static_cell::StaticCell;
use {defmt_rtt as _, esp_backtrace as _};

extern crate alloc;

static LED_FLASH_TIME_MS: u64 = 20;
static FONT: MonoFont = profont::PROFONT_7_POINT;

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

fn i2c_init<'d>(
    i2c: impl Peripheral<P = impl i2c::master::Instance> + 'd,
    sda: impl Peripheral<P = impl PeripheralOutput> + 'd,
    scl: impl Peripheral<P = impl PeripheralOutput> + 'd,
) -> I2c<'d, Async> {
    let config = i2c::master::Config::default();
    I2c::new(i2c, config)
        .with_sda(sda)
        .with_scl(scl)
        .into_async()
}

fn ssd1306_init<'d>(
    i2c: I2c<'d, Async>,
) -> Ssd1306<I2CInterface<I2c<'_, Async>>, DisplaySize128x64, mode::BasicMode> {
    let interface = I2CDisplayInterface::new(i2c);
    let mut display = Ssd1306::new(interface, DisplaySize128x64, DisplayRotation::Rotate0);
    display.init().ok();
    display.clear().ok();
    display
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

    let i2c = i2c_init(peripherals.I2C0, peripherals.GPIO5, peripherals.GPIO6);
    let mut display = ssd1306_init(i2c).into_buffered_graphics_mode();
    info!("SSD1306 initialized.");

    let led_ctrl = led_init(&spawner, peripherals.GPIO7);
    info!("LED Indicator initialized.");

    let _ = spawner;

    loop {
        let recv = twai.receive_async().await;
        match recv {
            Ok(frame) => {
                result_render(&mut display, frame);
                led_ctrl.signal(Instant::now());
            }
            Err(_) => {}
        }
    }
}

fn result_render<'d, SIZE: DisplaySize>(
    display: &mut Ssd1306<I2CInterface<I2c<'d, Async>>, SIZE, BufferedGraphicsMode<SIZE>>,
    frame: impl Frame,
) {
    let style = MonoTextStyle::new(&FONT, pixelcolor::BinaryColor::On);
    let mut output = String::new();
    output += &(match frame.id() {
        Id::Standard(std_id) => format!("ID: {:03x}\n", std_id.as_raw()),
        Id::Extended(ext_id) => format!("ID: {:08x}\n", ext_id.as_raw()),
    });
    output += &(format!("remote: {}\n", frame.is_remote_frame()));
    output += &(format!("len: {}\n", frame.dlc()));
    for n in frame.data() {
        output += &(format!("{:02x},", *n));
    }
    display.clear_buffer();
    Text::with_baseline(&output, Point::zero(), style, Baseline::Top)
        .draw(display)
        .unwrap();
    display.flush().unwrap();
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
