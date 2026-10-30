//! Lesson 14: send DJI command groups and decode feedback on two CAN buses.

#![no_std]
#![no_main]
#![forbid(unsafe_code)]

use embassy_stm32::can::{self, Can};
use embassy_stm32::mode::Async;
use embassy_stm32::peripherals::{CAN1, CAN2, DMA1_CH1, USART3};
use embassy_stm32::usart::{self, UartRx};
use embassy_sync::blocking_mutex::raw::CriticalSectionRawMutex;
use embassy_sync::signal::Signal;
use embassy_time::{Duration, Instant};
use panic_rtt_target as _;
use rust_embedded_examples::remote::Remote;

embassy_stm32::bind_interrupts!(struct Irqs {
    USART3 => embassy_stm32::usart::InterruptHandler<USART3>;
    DMA1_STREAM1 => embassy_stm32::dma::InterruptHandler<DMA1_CH1>;
    CAN1_TX => can::TxInterruptHandler<CAN1>;
    CAN1_RX0 => can::Rx0InterruptHandler<CAN1>;
    CAN1_RX1 => can::Rx1InterruptHandler<CAN1>;
    CAN1_SCE => can::SceInterruptHandler<CAN1>;
    CAN2_TX => can::TxInterruptHandler<CAN2>;
    CAN2_RX0 => can::Rx0InterruptHandler<CAN2>;
    CAN2_RX1 => can::Rx1InterruptHandler<CAN2>;
    CAN2_SCE => can::SceInterruptHandler<CAN2>;
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
    can1: Can<'static>,
    can2: Can<'static>,
}

async fn init() -> Peripherals {
    use embassy_stm32::can::{Fifo, filter::Mask32};
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
    let rx = UartRx::new(p.USART3, p.PC11, p.DMA1_CH1, Irqs, rc_config)
        .expect("DR16 UART configuration");
    let mut can1 = Can::new(p.CAN1, p.PD0, p.PD1, Irqs);
    let mut can2 = Can::new(p.CAN2, p.PB5, p.PB6, Irqs);
    {
        // CAN1 owns the shared filter banks, including CAN2's half.
        let mut filters = can1.modify_filters();
        filters.set_split(14);
        filters.enable_bank(0, Fifo::Fifo0, Mask32::accept_all());
        filters
            .slave_filters()
            .enable_bank(14, Fifo::Fifo0, Mask32::accept_all());
    }
    for bus in [&mut can1, &mut can2] {
        bus.set_bitrate(1_000_000);
        bus.set_tx_fifo_scheduling(true);
        // Old actuator commands must not retry indefinitely if the bus disappears.
        bus.modify_config().set_automatic_retransmit(false);
        embassy_time::with_timeout(Duration::from_millis(100), bus.enable())
            .await
            .expect("CAN startup timed out after 100 ms");
    }
    Peripherals { rx, can1, can2 }
}

#[embassy_executor::main]
async fn main(spawner: embassy_executor::Spawner) {
    use embassy_time::Timer;
    use embedded_can::Id;
    use rust_embedded_examples::control::{Motor, command};

    let Peripherals {
        rx,
        mut can1,
        mut can2,
    } = init().await;
    spawner.spawn(remote(rx).expect("remote task"));
    let mut latest: Option<(Remote, Instant)> = None;
    let mut chassis_feedback: [Option<Motor>; 4] = [None; 4];
    let mut gimbal_feedback: [Option<Motor>; 3] = [None; 3];
    loop {
        if let Some(frame) = REMOTE.try_take() {
            latest = Some(frame);
        }
        let permitted = latest.is_some_and(|(rc, at)| {
            (rc.switches[0] == 1) && at.elapsed() < Duration::from_millis(100)
        });
        for _ in 0..16 {
            let Ok(envelope) = can1.try_read() else {
                break;
            };
            if let Id::Standard(id) = envelope.frame.id()
                && (0x201..=0x204).contains(&id.as_raw())
            {
                let index = usize::from(id.as_raw() - 0x201);
                if let Some(motor) = Motor::decode(&envelope.frame, envelope.ts) {
                    chassis_feedback[index] = Some(motor);
                }
            }
        }
        for _ in 0..16 {
            let Ok(envelope) = can2.try_read() else {
                break;
            };
            if let Id::Standard(id) = envelope.frame.id()
                && (0x205..=0x207).contains(&id.as_raw())
            {
                let index = usize::from(id.as_raw() - 0x205);
                if let Some(motor) = Motor::decode(&envelope.frame, envelope.ts) {
                    gimbal_feedback[index] = Some(motor);
                }
            }
        }
        // Use 500 raw counts here; upstream continuously commands 4000/10000 counts.
        let wheels = if permitted
            && chassis_feedback
                .iter()
                .all(|m| m.is_some_and(|m| m.fresh(Instant::now())))
        {
            [500; 4]
        } else {
            [0; 4]
        };
        let axes = if permitted
            && gimbal_feedback
                .iter()
                .all(|m| m.is_some_and(|m| m.fresh(Instant::now())))
        {
            [500, 500, 500, 0]
        } else {
            [0; 4]
        };
        let _ = can1.try_write(&command(0x200, wheels));
        let _ = can2.try_write(&command(0x1ff, axes));
        Timer::after_millis(2).await;
    }
}
