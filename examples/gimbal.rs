//! Lesson 19: two encoder-relative GM6020 angle/rate loops, with stale-input stop.

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
    use embedded_can::Id;
    use rust_embedded_examples::control::{Motor, Pi, angle_error, command};

    let Peripherals {
        rx,
        can1: _can1,
        mut can2,
    } = init().await;
    spawner.spawn(remote(rx).expect("remote task"));
    let mut axes: [Option<Motor>; 2] = [None; 2];
    let mut latest: Option<(Remote, Instant)> = None;
    let mut axis_pi: [Pi; 2] = core::array::from_fn(|_| Pi::new(1200.0, 100.0, 2000.0));
    let mut home: Option<[f32; 2]> = None;
    let mut target = [0.0_f32; 2];
    let mut previous = Instant::now();
    let mut ticker = embassy_time::Ticker::every(Duration::from_millis(2));
    loop {
        ticker.next().await;
        if let Some(frame) = REMOTE.try_take() {
            latest = Some(frame);
        }
        for _ in 0..16 {
            let Ok(envelope) = can2.try_read() else {
                break;
            };
            if let Id::Standard(id) = envelope.frame.id()
                && (0x205..=0x206).contains(&id.as_raw())
            {
                axes[usize::from(id.as_raw() - 0x205)] =
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
        let mut axis_command = [0_i16; 4];
        let gimbal_ready = axes.iter().all(|m| m.is_some_and(|m| m.fresh(now)));
        if let (Some((rc, _)), Some(yaw), Some(pitch)) = (
            operator.filter(|_| gimbal_ready && timing_ok),
            axes[0],
            axes[1],
        ) {
            let reference = *home.get_or_insert([yaw.angle_rad, pitch.angle_rad]);
            target[0] = (target[0] - rc.stick(2) * dt_s * 0.5).clamp(-0.5, 0.5);
            target[1] = (target[1] + rc.stick(3) * dt_s * 0.3).clamp(-0.15, 0.15);
            for (i, motor) in [yaw, pitch].into_iter().enumerate() {
                let velocity_target = 5.0 * angle_error(reference[i] + target[i], motor.angle_rad);
                axis_command[i] =
                    axis_pi[i].update(velocity_target - motor.velocity_rad_s, dt_s) as i16;
            }
        } else {
            home = None;
            target = [0.0; 2];
            for controller in &mut axis_pi {
                controller.reset();
            }
        }
        let _ = can2.try_write(&command(0x1ff, axis_command));
    }
}
