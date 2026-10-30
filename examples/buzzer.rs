//! Lesson 5: sweep the onboard TIM4/PD14 buzzer every 250 ms.

#![no_std]
#![no_main]
#![forbid(unsafe_code)]

use embassy_stm32::peripherals::TIM4;
use embassy_stm32::time::Hertz;
use embassy_stm32::timer::simple_pwm::SimplePwm;
use panic_rtt_target as _;

fn init() -> SimplePwm<'static, TIM4> {
    use embassy_stm32::gpio::OutputType;
    use embassy_stm32::rcc::{
        AHBPrescaler, APBPrescaler, Hse, HseMode, Pll, PllMul, PllPDiv, PllPreDiv, PllQDiv,
        PllSource, Sysclk,
    };
    use embassy_stm32::timer::low_level::CountingMode;
    use embassy_stm32::timer::simple_pwm::PwmPin;

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
    let mut buzzer = SimplePwm::new(
        p.TIM4,
        None,
        None,
        Some(PwmPin::new(p.PD14, OutputType::PushPull)),
        None,
        Hertz(4000),
        CountingMode::EdgeAlignedUp,
    );
    buzzer.ch3().enable();
    buzzer
}

#[embassy_executor::main]
async fn main(_spawner: embassy_executor::Spawner) {
    use embassy_time::Timer;

    let mut buzzer = init();
    let mut prescaler = 0_u32;
    let mut compare = 10_000_u32;
    loop {
        prescaler = (prescaler + 1) % 1001;
        compare += 1;
        if compare > 20_000 {
            compare = 10_000;
        }
        buzzer.set_frequency(Hertz(84_000_000 / ((prescaler + 1) * 21_000)));
        buzzer.ch3().set_duty_cycle_fraction(compare, 21_000);
        Timer::after_millis(250).await;
    }
}
