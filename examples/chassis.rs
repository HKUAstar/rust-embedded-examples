//! Lesson 17: remote-controlled four-wheel mecanum speed loops, with stale-input stop.

#![no_std]
#![no_main]
#![forbid(unsafe_code)]

use embassy_stm32::can::{self, Can};
use embassy_stm32::mode::Async;
use embassy_stm32::peripherals::{CAN1, DMA1_CH1, USART3};
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
    {
        // Accept all standard data for this lesson; IDs are checked below.
        let mut filters = can1.modify_filters();
        filters.enable_bank(0, Fifo::Fifo0, Mask32::accept_all());
    }
    can1.set_bitrate(1_000_000);
    can1.set_tx_fifo_scheduling(true);
    can1.modify_config().set_automatic_retransmit(false);
    embassy_time::with_timeout(Duration::from_millis(100), can1.enable())
        .await
        .expect("CAN1 startup timed out after 100 ms");
    Peripherals { rx, can1 }
}

#[embassy_executor::main]
async fn main(spawner: embassy_executor::Spawner) {
    use embedded_can::Id;
    use rust_embedded_examples::control::{Motor, Pi, command};

    let Peripherals { rx, mut can1 } = init().await;
    spawner.spawn(remote(rx).expect("remote task"));
    let mut wheels: [Option<Motor>; 4] = [None; 4];
    let mut latest: Option<(Remote, Instant)> = None;
    let mut wheel_pi: [Pi; 4] = core::array::from_fn(|_| Pi::new(4.0, 1.0, 2000.0));
    let mut previous = Instant::now();
    let mut ticker = embassy_time::Ticker::every(Duration::from_millis(2));
    loop {
        ticker.next().await;
        if let Some(frame) = REMOTE.try_take() {
            latest = Some(frame);
        }
        for _ in 0..16 {
            let Ok(envelope) = can1.try_read() else {
                break;
            };
            if let Id::Standard(id) = envelope.frame.id()
                && (0x201..=0x204).contains(&id.as_raw())
            {
                wheels[usize::from(id.as_raw() - 0x201)] =
                    Motor::decode(&envelope.frame, envelope.ts);
            }
        }
        let now = Instant::now();
        let dt_s = now.duration_since(previous).as_micros() as f32 * 1e-6;
        previous = now;
        let operator = latest.filter(|(rc, at)| {
            (rc.switches[0] == 1) && now.duration_since(*at) < Duration::from_millis(100)
        });
        let timing_ok = dt_s > 0.0 && dt_s < 0.01;
        let mut wheel_command = [0_i16; 4];
        let wheels_ready = wheels.iter().all(|m| m.is_some_and(|m| m.fresh(now)));
        if let Some((rc, _)) = operator.filter(|_| wheels_ready && timing_ok) {
            // Mecanum rotor targets in rad/s: FR, FL, RL, RR, with the upstream wiring signs.
            let forward = rc.stick(1) * 30.0;
            let left = -rc.stick(0) * 30.0;
            let spin = rc.stick(2) * 10.0;
            let targets = [
                -forward - left - spin,
                forward - left - spin,
                forward + left - spin,
                -forward + left - spin,
            ];
            for i in 0..4 {
                if let Some(motor) = wheels[i] {
                    wheel_command[i] =
                        wheel_pi[i].update(targets[i] - motor.velocity_rad_s, dt_s) as i16;
                }
            }
        } else {
            for controller in &mut wheel_pi {
                controller.reset();
            }
        }
        let _ = can1.try_write(&command(0x200, wheel_command));
    }
}
