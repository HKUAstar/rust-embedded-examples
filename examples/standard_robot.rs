//! Lesson 20: compose chassis, encoder gimbal and gated friction/loader outputs.

#![no_std]
#![no_main]
#![forbid(unsafe_code)]

use embassy_stm32::can::{self, Can};
use embassy_stm32::mode::Async;
use embassy_stm32::peripherals::{CAN1, CAN2, DMA1_CH1, TIM1, USART3};
use embassy_stm32::timer::simple_pwm::SimplePwm;
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
    friction: SimplePwm<'static, TIM1>,
}

async fn init() -> Peripherals {
    use embassy_stm32::can::{Fifo, filter::Mask32};
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
    let mut friction = SimplePwm::new(
        p.TIM1,
        Some(PwmPin::new(p.PE9, OutputType::PushPull)),
        Some(PwmPin::new(p.PE11, OutputType::PushPull)),
        None,
        None,
        Hertz(50),
        CountingMode::EdgeAlignedUp,
    );
    friction.ch1().enable();
    friction.ch2().enable();
    Peripherals {
        rx,
        can1,
        can2,
        friction,
    }
}

#[embassy_executor::main]
async fn main(spawner: embassy_executor::Spawner) {
    use embedded_can::Id;
    use rust_embedded_examples::control::{Motor, Pi, angle_error, command};

    let Peripherals {
        rx,
        mut can1,
        mut can2,
        mut friction,
    } = init().await;
    spawner.spawn(remote(rx).expect("remote task"));
    let mut wheels: [Option<Motor>; 4] = [None; 4];
    let mut axes: [Option<Motor>; 3] = [None; 3];
    let mut latest: Option<(Remote, Instant)> = None;
    let mut wheel_pi: [Pi; 4] = core::array::from_fn(|_| Pi::new(4.0, 1.0, 2000.0));
    let mut axis_pi: [Pi; 2] = core::array::from_fn(|_| Pi::new(1200.0, 100.0, 2000.0));
    let mut home: Option<[f32; 2]> = None;
    let mut target = [0.0_f32; 2];
    let mut previous = Instant::now();
    let mut ticker = embassy_time::Ticker::every(Duration::from_millis(2));
    let mut friction_since: Option<Instant> = None;
    let mut loader_pi = Pi::new(4.0, 1.0, 1000.0);
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
        for _ in 0..16 {
            let Ok(envelope) = can2.try_read() else {
                break;
            };
            if let Id::Standard(id) = envelope.frame.id()
                && (0x205..=0x207).contains(&id.as_raw())
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
        let mut wheel_command = [0_i16; 4];
        let wheels_ready = wheels.iter().all(|m| m.is_some_and(|m| m.fresh(now)));
        if let Some((rc, _)) = operator.filter(|_| wheels_ready && timing_ok) {
            // Mecanum rotor targets in rad/s: FR, FL, RL, RR, with the upstream wiring signs.
            let forward = rc.stick(1) * 30.0;
            let left = -rc.stick(0) * 30.0;
            let spin = rc.stick(4) * 10.0;
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
        let mut axis_command = [0_i16; 4];
        let gimbal_ready = axes[..2].iter().all(|m| m.is_some_and(|m| m.fresh(now)));
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
        // Left switch up requests friction wheels; mouse-left requests the loader.
        let friction_on =
            operator.is_some_and(|(rc, _)| rc.switches[1] == 1) && gimbal_ready && timing_ok;
        let pulse_us = if friction_on { 1200 } else { 1000 };
        let duty = friction.max_duty_cycle() * pulse_us / 20_000;
        friction.ch1().set_duty_cycle(duty);
        friction.ch2().set_duty_cycle(duty);
        if !friction_on {
            friction_since = None;
        }
        let warmed_up =
            friction_on && friction_since.get_or_insert(now).elapsed() >= Duration::from_secs(1);
        if let (Some((rc, _)), Some(loader)) = (operator, axes[2].filter(|m| m.fresh(now))) {
            if warmed_up && rc.buttons[0] && timing_ok {
                axis_command[2] = loader_pi.update(10.0 - loader.velocity_rad_s, dt_s) as i16;
            } else {
                loader_pi.reset();
            }
        } else {
            loader_pi.reset();
        }
        let _ = can1.try_write(&command(0x200, wheel_command));
        let _ = can2.try_write(&command(0x1ff, axis_command));
    }
}
