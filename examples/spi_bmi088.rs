//! Lesson 13: read onboard BMI088 acceleration, angular rate and temperature.

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
    use embassy_time::{Duration, Timer};

    let mut imu = init();
    loop {
        match imu.read() {
            Ok(sample) => {
                rtt_target::rprintln!(
                    "a {:?} m/s2, w {:?} rad/s, T {} C",
                    sample.accel_m_s2,
                    sample.gyro_rad_s,
                    sample.temperature_c
                );
            }
            Err(e) => {
                rtt_target::rprintln!("BMI088 read: {:?}", e);
            }
        }
        Timer::after(Duration::from_millis(10)).await;
    }
}
