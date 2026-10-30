//! Lesson 14: DR16 throttle controls seven 50 Hz ESC outputs with a loss timeout.

#![no_std]
#![no_main]
#![forbid(unsafe_code)]

use embassy_stm32::mode::Async;
use embassy_stm32::peripherals::{DMA1_CH1, TIM1, TIM8, USART3};
use embassy_stm32::timer::Channel;
use embassy_stm32::timer::simple_pwm::SimplePwm;
use embassy_stm32::usart::{self, UartRx};
use embassy_sync::blocking_mutex::raw::CriticalSectionRawMutex;
use embassy_sync::signal::Signal;
use embassy_time::Instant;
use panic_rtt_target as _;
use rust_embedded_examples::remote::Remote;

embassy_stm32::bind_interrupts!(struct Irqs {
    USART3 => embassy_stm32::usart::InterruptHandler<USART3>;
    DMA1_STREAM1 => embassy_stm32::dma::InterruptHandler<DMA1_CH1>;
});
static REMOTE: Signal<CriticalSectionRawMutex, (Remote, Instant)> = Signal::new();

#[embassy_executor::task]
async fn remote(mut rx: UartRx<'static, Async>) {
    let mut buffer = [0; 36];
    loop {
        let n = rx.read_until_idle(&mut buffer).await.unwrap_or(0);
        if let Some(frame) = Remote::decode(&buffer[..n]) {
            REMOTE.signal((frame, Instant::now()));
        }
    }
}

struct Peripherals {
    rx: UartRx<'static, Async>,
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
    let mut rc_config = usart::Config::default();
    rc_config.baudrate = 100_000;
    rc_config.parity = usart::Parity::ParityEven;
    rc_config.stop_bits = usart::StopBits::STOP2;
    let rx = UartRx::new(p.USART3, p.PC11, p.DMA1_CH1, Irqs, rc_config)
        .expect("DR16 UART configuration");
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
    Peripherals { rx, pwm1, pwm8 }
}

#[embassy_executor::main]
async fn main(spawner: embassy_executor::Spawner) {
    use embassy_time::{Duration, Timer};

    let Peripherals {
        rx,
        mut pwm1,
        mut pwm8,
    } = init();
    spawner.spawn(remote(rx).expect("remote task"));
    let mut latest: Option<(Remote, Instant)> = None;
    loop {
        if let Some(frame) = REMOTE.try_take() {
            latest = Some(frame);
        }
        let pulse_us = latest
            .filter(|(rc, at)| (rc.switches[0] == 1) && at.elapsed() < Duration::from_millis(100))
            .map_or(1000, |(rc, _)| {
                1000 + u32::from(rc.channels[3].max(0) as u16)
            });
        let duty1 = pwm1.max_duty_cycle() * pulse_us / 20_000;
        let duty8 = pwm8.max_duty_cycle() * pulse_us / 20_000;
        for ch in [Channel::Ch1, Channel::Ch2, Channel::Ch3, Channel::Ch4] {
            pwm1.channel(ch).set_duty_cycle(duty1);
        }
        for ch in [Channel::Ch1, Channel::Ch2, Channel::Ch3] {
            pwm8.channel(ch).set_duty_cycle(duty8);
        }
        Timer::after_millis(2).await;
    }
}
