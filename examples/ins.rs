//! Lesson 18: integrate BMI088 gyro with accelerometer gravity correction, report Euler radians.

#![no_std]
#![no_main]
#![forbid(unsafe_code)]

use embassy_stm32::gpio::Output;
use embassy_stm32::spi::{self, Spi};
use panic_rtt_target as _;
use rust_embedded_examples::bmi088::Bmi088;

fn init() -> Bmi088<Spi<'static, embassy_stm32::mode::Blocking, spi::mode::Master>, Output<'static>>
{
    use embassy_stm32::gpio::{Level, Speed};
    use embassy_stm32::rcc::{
        AHBPrescaler, APBPrescaler, Hse, HseMode, Pll, PllMul, PllPDiv, PllPreDiv, PllQDiv,
        PllSource, Sysclk,
    };
    use embassy_stm32::time::Hertz;
    use embassy_time::Delay;

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
    let mut spi_config = spi::Config::default();
    spi_config.mode = spi::MODE_3;
    spi_config.frequency = Hertz(10_000_000);
    let spi = Spi::new_blocking(p.SPI1, p.PB3, p.PA7, p.PB4, spi_config);
    let accel_cs = Output::new(p.PA4, Level::High, Speed::VeryHigh);
    let gyro_cs = Output::new(p.PB0, Level::High, Speed::VeryHigh);
    Bmi088::new(spi, accel_cs, gyro_cs, &mut Delay).expect("BMI088 initialize")
}

#[embassy_executor::main]
async fn main(_spawner: embassy_executor::Spawner) {
    use embassy_time::{Duration, Instant, Ticker};

    let mut imu = init();
    let mut attitude = Attitude::new();
    let mut ticker = Ticker::every(Duration::from_hz(1000));
    let mut previous = Instant::now();
    let mut tick = 0_u32;
    loop {
        ticker.next().await;
        let now = Instant::now();
        let dt_s = now.duration_since(previous).as_micros() as f32 * 1e-6;
        previous = now;
        let Ok(sample) = imu.read() else {
            continue;
        };
        if dt_s > 0.01 {
            continue;
        }
        attitude.update(sample.gyro_rad_s, sample.accel_m_s2, dt_s);
        tick = tick.wrapping_add(1);
        if tick.is_multiple_of(100) {
            rtt_target::rprintln!("roll pitch yaw {:?} rad", attitude.euler_rad());
        }
    }
}

/// Quaternion in scalar-first order, rotating sensor coordinates into the world frame.
struct Attitude {
    q: [f32; 4],
}

impl Attitude {
    const fn new() -> Self {
        Self {
            q: [1.0, 0.0, 0.0, 0.0],
        }
    }

    /// Integrates rad/s and corrects tilt from acceleration. Large time gaps are ignored.
    fn update(&mut self, gyro_rad_s: [f32; 3], accel_m_s2: [f32; 3], dt_s: f32) {
        if !dt_s.is_finite()
            || !(0.0..=0.01).contains(&dt_s)
            || gyro_rad_s
                .iter()
                .chain(accel_m_s2.iter())
                .any(|v| !v.is_finite())
        {
            return;
        }
        let [w, x, y, z] = self.q;
        let mut rate = gyro_rad_s;
        let magnitude = libm::sqrtf(accel_m_s2.iter().map(|v| v * v).sum());
        // Avoid treating strong linear acceleration as gravity.
        if (7.0..=12.0).contains(&magnitude) {
            let a = accel_m_s2.map(|v| v / magnitude);
            let gravity = [
                2.0 * (x * z - w * y),
                2.0 * (w * x + y * z),
                1.0 - 2.0 * (x * x + y * y),
            ];
            let error = [
                a[1] * gravity[2] - a[2] * gravity[1],
                a[2] * gravity[0] - a[0] * gravity[2],
                a[0] * gravity[1] - a[1] * gravity[0],
            ];
            for i in 0..3 {
                rate[i] += error[i];
            }
        }
        let [gx, gy, gz] = rate.map(|v| v * dt_s * 0.5);
        let next = [
            w - x * gx - y * gy - z * gz,
            x + w * gx + y * gz - z * gy,
            y + w * gy - x * gz + z * gx,
            z + w * gz + x * gy - y * gx,
        ];
        let norm = libm::sqrtf(next.iter().map(|v| v * v).sum());
        if norm.is_finite() && norm > 0.0 {
            self.q = next.map(|v| v / norm);
        }
    }

    /// Roll, pitch and yaw in radians, with a Z-Y-X Euler decomposition.
    fn euler_rad(&self) -> [f32; 3] {
        let [w, x, y, z] = self.q;
        [
            libm::atan2f(2.0 * (w * x + y * z), 1.0 - 2.0 * (x * x + y * y)),
            libm::asinf((2.0 * (w * y - z * x)).clamp(-1.0, 1.0)),
            libm::atan2f(2.0 * (w * z + x * y), 1.0 - 2.0 * (y * y + z * z)),
        ]
    }
}
