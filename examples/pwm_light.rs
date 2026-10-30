//! Lesson 4: PWM brightness for the RGB LED, color 0x7F123456.

#![no_std]
#![no_main]
#![forbid(unsafe_code)]

use embassy_stm32::peripherals::TIM5;
use embassy_stm32::timer::Channel;
use embassy_stm32::timer::simple_pwm::SimplePwm;
use panic_rtt_target as _;

fn init() -> SimplePwm<'static, TIM5> {
    use embassy_stm32::gpio::OutputType;
    use embassy_stm32::rcc::{
        AHBPrescaler, APBPrescaler, Hse, HseMode, Pll, PllMul, PllPDiv, PllPreDiv, PllQDiv,
        PllSource, Sysclk,
    };
    use embassy_stm32::time::Hertz;
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
    let mut pwm = SimplePwm::new(
        p.TIM5,
        Some(PwmPin::new(p.PH10, OutputType::PushPull)),
        Some(PwmPin::new(p.PH11, OutputType::PushPull)),
        Some(PwmPin::new(p.PH12, OutputType::PushPull)),
        None,
        Hertz(1282),
        CountingMode::EdgeAlignedUp,
    );
    for ch in [Channel::Ch1, Channel::Ch2, Channel::Ch3] {
        pwm.channel(ch).enable();
    }
    pwm
}

#[embassy_executor::main]
async fn main(_spawner: embassy_executor::Spawner) {
    use embassy_time::Timer;

    let mut pwm = init();
    let max = pwm.max_duty_cycle();
    for (ch, color) in [
        (Channel::Ch1, 0x56),
        (Channel::Ch2, 0x34),
        (Channel::Ch3, 0x12),
    ] {
        pwm.channel(ch).set_duty_cycle(max * color * 0x7f / 65536);
    }
    Timer::after_secs(1).await;
    core::future::pending::<()>().await;
}
