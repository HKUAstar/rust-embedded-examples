//! Lesson 15: three tasks light successive 500 ms red, green and blue slots.

#![no_std]
#![no_main]
#![forbid(unsafe_code)]

use embassy_stm32::gpio::Output;
use panic_rtt_target as _;

#[embassy_executor::task]
async fn red_led(mut led: Output<'static>) {
    loop {
        led.set_high();
        embassy_time::Timer::after_millis(500).await;
        led.set_low();
        embassy_time::Timer::after_millis(1000).await;
    }
}
#[embassy_executor::task]
async fn green_led(mut led: Output<'static>) {
    loop {
        led.set_low();
        embassy_time::Timer::after_millis(500).await;
        led.set_high();
        embassy_time::Timer::after_millis(500).await;
        led.set_low();
        embassy_time::Timer::after_millis(500).await;
    }
}
#[embassy_executor::task]
async fn blue_led(mut led: Output<'static>) {
    loop {
        led.set_low();
        embassy_time::Timer::after_millis(1000).await;
        led.set_high();
        embassy_time::Timer::after_millis(500).await;
    }
}

struct Peripherals {
    red: Output<'static>,
    green: Output<'static>,
    blue: Output<'static>,
}

fn init() -> Peripherals {
    use embassy_stm32::gpio::{Level, Speed};
    use embassy_stm32::rcc::{
        AHBPrescaler, APBPrescaler, Hse, HseMode, Pll, PllMul, PllPDiv, PllPreDiv, PllQDiv,
        PllSource, Sysclk,
    };
    use embassy_stm32::time::Hertz;

    rtt_target::rtt_init_print!();
    // RM-C: 12 MHz crystal, 168 MHz CPU, APB1 42 MHz, APB2 84 MHz.
    let mut config = embassy_stm32::Config::default();
    config.rcc.hse = Some(Hse {
        freq: Hertz(12_000_000),
        mode: HseMode::Oscillator,
    });
    config.rcc.pll_src = PllSource::HSE;
    config.rcc.pll = Some(Pll {
        prediv: PllPreDiv::DIV12,
        mul: PllMul::MUL336,
        divp: Some(PllPDiv::DIV2),
        divq: Some(PllQDiv::DIV7),
        divr: None,
    });
    config.rcc.sys = Sysclk::PLL1_P;
    config.rcc.ahb_pre = AHBPrescaler::DIV1;
    config.rcc.apb1_pre = APBPrescaler::DIV4;
    config.rcc.apb2_pre = APBPrescaler::DIV2;
    let p = embassy_stm32::init(config);
    let red = Output::new(p.PH12, Level::Low, Speed::Low);
    let green = Output::new(p.PH11, Level::Low, Speed::Low);
    let blue = Output::new(p.PH10, Level::Low, Speed::Low);
    Peripherals { red, green, blue }
}

#[embassy_executor::main]
async fn main(spawner: embassy_executor::Spawner) {
    let Peripherals { red, green, blue } = init();
    spawner.spawn(red_led(red).expect("red task"));
    spawner.spawn(green_led(green).expect("green task"));
    spawner.spawn(blue_led(blue).expect("blue task"));
    core::future::pending::<()>().await;
}
