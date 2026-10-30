//! Lesson 6: debounce PA0 edges and light green while the key is pressed.

#![no_std]
#![no_main]
#![forbid(unsafe_code)]

use embassy_stm32::exti::ExtiInput;
use embassy_stm32::gpio::{Level, Output};
use panic_rtt_target as _;

embassy_stm32::bind_interrupts!(struct Irqs {
    EXTI0 => embassy_stm32::exti::InterruptHandler<embassy_stm32::interrupt::typelevel::EXTI0>;
});

struct Peripherals {
    green: Output<'static>,
    key: ExtiInput<'static, embassy_stm32::mode::Async>,
}

fn init() -> Peripherals {
    use embassy_stm32::gpio::{Pull, Speed};
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
    let green = Output::new(p.PH11, Level::Low, Speed::Low);
    let key = ExtiInput::new(p.PA0, p.EXTI0, Pull::Up, Irqs);
    Peripherals { green, key }
}

#[embassy_executor::main]
async fn main(_spawner: embassy_executor::Spawner) {
    use embassy_time::Timer;

    let Peripherals { mut green, mut key } = init();
    loop {
        green.set_level(if key.is_low() {
            Level::High
        } else {
            Level::Low
        });
        key.wait_for_any_edge().await;
        Timer::after_millis(20).await;
    }
}
