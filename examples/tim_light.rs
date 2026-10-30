//! Lesson 3: use TIM1 update events to toggle the LEDs every 500 ms.

#![no_std]
#![no_main]
#![forbid(unsafe_code)]

use embassy_stm32::gpio::Output;
use embassy_stm32::peripherals::TIM1;
use embassy_stm32::timer::low_level::Timer;
use panic_rtt_target as _;

struct Peripherals {
    red: Output<'static>,
    green: Output<'static>,
    blue: Output<'static>,
    timer: Timer<'static, TIM1>,
}

fn init() -> Peripherals {
    use embassy_stm32::gpio::{Level, Speed};
    use embassy_stm32::rcc::{
        AHBPrescaler, APBPrescaler, Hse, HseMode, Pll, PllMul, PllPDiv, PllPreDiv, PllQDiv,
        PllSource, Sysclk,
    };
    use embassy_stm32::time::Hertz;
    use embassy_stm32::timer::low_level::RoundTo;

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
    let timer = Timer::new(p.TIM1);
    timer.set_frequency(Hertz(2), RoundTo::Slower);
    timer.clear_update_interrupt();
    timer.start();
    Peripherals {
        red,
        green,
        blue,
        timer,
    }
}

#[embassy_executor::main]
async fn main(_spawner: embassy_executor::Spawner) {
    use embassy_time::Timer as Delay;

    let Peripherals {
        mut red,
        mut green,
        mut blue,
        timer,
    } = init();
    loop {
        if timer.clear_update_interrupt() {
            red.toggle();
            green.toggle();
            blue.toggle();
        }
        Delay::after_millis(1).await;
    }
}
