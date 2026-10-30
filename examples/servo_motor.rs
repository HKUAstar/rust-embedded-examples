//! Lesson 5: seven 50 Hz servo outputs alternate 500 and 2000 microseconds.

#![no_std]
#![no_main]
#![forbid(unsafe_code)]

use embassy_stm32::peripherals::{TIM1, TIM8};
use embassy_stm32::timer::Channel;
use embassy_stm32::timer::simple_pwm::SimplePwm;
use panic_rtt_target as _;

struct Peripherals {
    pwm1: SimplePwm<'static, TIM1>,
    pwm8: SimplePwm<'static, TIM8>,
}

fn init() -> Peripherals {
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
    let mut pwm1 = SimplePwm::new(
        p.TIM1,
        Some(PwmPin::new(p.PE9, OutputType::PushPull)),
        Some(PwmPin::new(p.PE11, OutputType::PushPull)),
        Some(PwmPin::new(p.PE13, OutputType::PushPull)),
        Some(PwmPin::new(p.PE14, OutputType::PushPull)),
        Hertz(50),
        CountingMode::EdgeAlignedUp,
    );
    let mut pwm8 = SimplePwm::new(
        p.TIM8,
        Some(PwmPin::new(p.PC6, OutputType::PushPull)),
        Some(PwmPin::new(p.PI6, OutputType::PushPull)),
        Some(PwmPin::new(p.PI7, OutputType::PushPull)),
        None,
        Hertz(50),
        CountingMode::EdgeAlignedUp,
    );
    for ch in [Channel::Ch1, Channel::Ch2, Channel::Ch3, Channel::Ch4] {
        pwm1.channel(ch).enable();
    }
    for ch in [Channel::Ch1, Channel::Ch2, Channel::Ch3] {
        pwm8.channel(ch).enable();
    }
    Peripherals { pwm1, pwm8 }
}

#[embassy_executor::main]
async fn main(_spawner: embassy_executor::Spawner) {
    use embassy_time::Timer;

    let Peripherals { mut pwm1, mut pwm8 } = init();
    loop {
        for pulse_us in [500, 2000] {
            let duty1 = pwm1.max_duty_cycle() * pulse_us / 20_000;
            let duty8 = pwm8.max_duty_cycle() * pulse_us / 20_000;
            for ch in [Channel::Ch1, Channel::Ch2, Channel::Ch3, Channel::Ch4] {
                pwm1.channel(ch).set_duty_cycle(duty1);
            }
            for ch in [Channel::Ch1, Channel::Ch2, Channel::Ch3] {
                pwm8.channel(ch).set_duty_cycle(duty8);
            }
            Timer::after_secs(1).await;
        }
    }
}
