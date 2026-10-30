//! Lesson 8: send RoboMaster and receive idle-delimited bytes on USART1 and USART6.

#![no_std]
#![no_main]
#![forbid(unsafe_code)]

use embassy_stm32::mode::Async;
use embassy_stm32::peripherals::{DMA2_CH2, DMA2_CH5, DMA2_CH6, DMA2_CH7, USART1, USART6};
use embassy_stm32::usart::{UartRx, UartTx};
use panic_rtt_target as _;

embassy_stm32::bind_interrupts!(struct Irqs {
    USART1 => embassy_stm32::usart::InterruptHandler<USART1>;
    USART6 => embassy_stm32::usart::InterruptHandler<USART6>;
    DMA2_STREAM7 => embassy_stm32::dma::InterruptHandler<DMA2_CH7>;
    DMA2_STREAM6 => embassy_stm32::dma::InterruptHandler<DMA2_CH6>;
    DMA2_STREAM2 => embassy_stm32::dma::InterruptHandler<DMA2_CH2>;
    DMA2_STREAM5 => embassy_stm32::dma::InterruptHandler<DMA2_CH5>;
});
#[embassy_executor::task(pool_size = 2)]
async fn transmit(mut tx: UartTx<'static, Async>) {
    loop {
        tx.write(b"RoboMaster\r\n").await.expect("UART write");
        embassy_time::Timer::after_millis(200).await;
    }
}
#[embassy_executor::task(pool_size = 2)]
async fn receive(mut rx: UartRx<'static, Async>, port: u8) {
    let mut buffer = [0; 64];
    loop {
        match rx.read_until_idle(&mut buffer).await {
            Ok(n) => {
                rtt_target::rprintln!("USART{}: {:?}", port, &buffer[..n]);
            }
            Err(e) => {
                rtt_target::rprintln!("USART{}: {:?}", port, e);
            }
        }
    }
}

struct Peripherals {
    tx1: UartTx<'static, Async>,
    rx1: UartRx<'static, Async>,
    tx6: UartTx<'static, Async>,
    rx6: UartRx<'static, Async>,
}

fn init() -> Peripherals {
    use embassy_stm32::rcc::{
        AHBPrescaler, APBPrescaler, Hse, HseMode, Pll, PllMul, PllPDiv, PllPreDiv, PllQDiv,
        PllSource, Sysclk,
    };
    use embassy_stm32::time::Hertz;
    use embassy_stm32::usart::{Config, Uart};

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
    let config = Config::default();
    let uart1 = Uart::new(p.USART1, p.PB7, p.PA9, p.DMA2_CH7, p.DMA2_CH5, Irqs, config)
        .expect("USART1 configuration");
    let uart6 = Uart::new(
        p.USART6, p.PG9, p.PG14, p.DMA2_CH6, p.DMA2_CH2, Irqs, config,
    )
    .expect("USART6 configuration");
    let (tx1, rx1) = uart1.split();
    let (tx6, rx6) = uart6.split();
    Peripherals { tx1, rx1, tx6, rx6 }
}

#[embassy_executor::main]
async fn main(spawner: embassy_executor::Spawner) {
    let Peripherals { tx1, rx1, tx6, rx6 } = init();
    spawner.spawn(transmit(tx1).expect("first transmitter"));
    spawner.spawn(receive(rx1, 1).expect("first receiver"));
    embassy_time::Timer::after_millis(100).await;
    spawner.spawn(transmit(tx6).expect("second transmitter"));
    spawner.spawn(receive(rx6, 6).expect("second receiver"));
    core::future::pending::<()>().await;
}
