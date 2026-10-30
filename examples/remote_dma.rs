//! Lesson 9: receive DR16 DBUS frames with DMA and inspect decoded controls.

#![no_std]
#![no_main]
#![forbid(unsafe_code)]

use embassy_stm32::mode::Async;
use embassy_stm32::peripherals::{DMA1_CH1, USART3};
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

fn init() -> UartRx<'static, Async> {
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
    let mut rc_config = usart::Config::default();
    rc_config.baudrate = 100_000;
    rc_config.parity = usart::Parity::ParityEven;
    rc_config.stop_bits = usart::StopBits::STOP2;
    UartRx::new(p.USART3, p.PC11, p.DMA1_CH1, Irqs, rc_config).expect("DR16 UART configuration")
}

#[embassy_executor::main]
async fn main(spawner: embassy_executor::Spawner) {
    use embassy_time::{Duration, Timer};

    let rx = init();
    spawner.spawn(remote(rx).expect("remote task"));
    loop {
        match embassy_time::with_timeout(Duration::from_millis(100), REMOTE.wait()).await {
            Ok((frame, at)) => {
                rtt_target::rprintln!("{:?}, age {} us", frame, at.elapsed().as_micros());
            }
            Err(_) => {
                rtt_target::rprintln!("remote missing");
            }
        }
        Timer::after_millis(100).await;
    }
}
