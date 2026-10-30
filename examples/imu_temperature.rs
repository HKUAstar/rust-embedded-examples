//! Lesson 16: report BMI088 temperature on data-ready edges; heater control is disabled.

#![no_std]
#![no_main]
#![forbid(unsafe_code)]

use embassy_stm32::exti::ExtiInput;
use embassy_stm32::gpio::Output;
// use embassy_stm32::peripherals::TIM10;
use embassy_stm32::spi::{self, Spi};
// use embassy_stm32::timer::low_level::Timer as HardwareTimer;
// use embassy_stm32::timer::simple_pwm::PwmPin;
use panic_rtt_target as _;
use rust_embedded_examples::bmi088::Bmi088;

embassy_stm32::bind_interrupts!(struct Irqs {
    EXTI4 => embassy_stm32::exti::InterruptHandler<embassy_stm32::interrupt::typelevel::EXTI4>;
});

struct Peripherals {
    _heater_off: Output<'static>,
    // heater: HardwareTimer<'static, TIM10>,
    // _heater_pin: PwmPin<'static, TIM10, embassy_stm32::timer::Ch1>,
    imu: Bmi088<Spi<'static, embassy_stm32::mode::Blocking, spi::mode::Master>, Output<'static>>,
    ready: ExtiInput<'static, embassy_stm32::mode::Async>,
}

fn init() -> Peripherals {
    use embassy_stm32::gpio::{Level, Pull, Speed};
    // use embassy_stm32::gpio::OutputType;
    use embassy_stm32::rcc::{
        AHBPrescaler, APBPrescaler, Hse, HseMode, Pll, PllMul, PllPDiv, PllPreDiv, PllQDiv,
        PllSource, Sysclk,
    };
    use embassy_stm32::time::Hertz;
    // use embassy_stm32::timer::low_level::{OutputCompareMode, RoundTo};
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
    // Keep the heater gate low. Leave this guard alive while the example runs.
    let heater_off = Output::new(p.PF6, Level::Low, Speed::Low);
    // Heater PWM is disabled. Remove heater_off before restoring the setup below.
    // let heater = HardwareTimer::new(p.TIM10);
    // let _heater_pin = PwmPin::<TIM10, embassy_stm32::timer::Ch1>::new(p.PF6, OutputType::PushPull);
    // heater.set_frequency(Hertz(33_600), RoundTo::Slower);
    // heater.set_output_compare_mode(
    //     embassy_stm32::timer::Channel::Ch1,
    //     OutputCompareMode::PwmMode1,
    // );
    // heater.set_compare_value(embassy_stm32::timer::Channel::Ch1, 0);
    // heater.enable_channel(embassy_stm32::timer::Channel::Ch1, true);
    // heater.start();
    let mut spi_config = spi::Config::default();
    spi_config.mode = spi::MODE_3;
    spi_config.frequency = Hertz(10_000_000);
    let spi = Spi::new_blocking(p.SPI1, p.PB3, p.PA7, p.PB4, spi_config);
    let accel_cs = Output::new(p.PA4, Level::High, Speed::VeryHigh);
    let gyro_cs = Output::new(p.PB0, Level::High, Speed::VeryHigh);
    let mut imu = Bmi088::new(spi, accel_cs, gyro_cs, &mut Delay).expect("BMI088 initialize");
    imu.enable_data_ready()
        .expect("BMI088 data-ready configuration");
    let ready = ExtiInput::new(p.PC4, p.EXTI4, Pull::Up, Irqs);
    Peripherals {
        _heater_off: heater_off,
        // heater,
        // _heater_pin,
        imu,
        ready,
    }
}

#[embassy_executor::main]
async fn main(_spawner: embassy_executor::Spawner) {
    use embassy_time::Duration;
    // use rust_embedded_examples::control::Pi;

    let Peripherals {
        _heater_off,
        // heater,
        // _heater_pin,
        mut imu,
        mut ready,
    } = init();
    const SAMPLE_TIMEOUT: Duration = Duration::from_millis(20);

    // let mut controller = Pi::new(1600.0, 320.0, 4500.0);
    let mut last = embassy_time::Instant::now();
    let mut report_at = last;
    loop {
        if embassy_time::with_timeout(SAMPLE_TIMEOUT, ready.wait_for_falling_edge())
            .await
            .is_err()
        {
            // heater.set_compare_value(embassy_stm32::timer::Channel::Ch1, 0);
            // controller.reset();
            continue;
        }
        let now = embassy_time::Instant::now();
        let elapsed = now.duration_since(last);
        // let dt_s = elapsed.as_micros() as f32 * 1e-6;
        last = now;
        // Skip the first edge after a long sample gap.
        if elapsed >= SAMPLE_TIMEOUT {
            // heater.set_compare_value(embassy_stm32::timer::Channel::Ch1, 0);
            // controller.reset();
            continue;
        }
        let Ok(sample) = imu.read() else {
            // heater.set_compare_value(embassy_stm32::timer::Channel::Ch1, 0);
            // controller.reset();
            continue;
        };
        if !(-40.0..=60.0).contains(&sample.temperature_c) {
            // heater.set_compare_value(embassy_stm32::timer::Channel::Ch1, 0);
            // controller.reset();
            continue;
        }
        if now.duration_since(report_at) >= Duration::from_millis(250) {
            rtt_target::rprintln!("temperature {:.2} C", sample.temperature_c);
            report_at = now;
        }
        // Heater control is retained for reference and does not run.
        // let raw = controller
        //     .update(40.0 - sample.temperature_c, dt_s)
        //     .clamp(0.0, 4500.0);
        // let duty = (raw / 5000.0 * heater.get_max_compare_value() as f32) as u16;
        // heater.set_compare_value(embassy_stm32::timer::Channel::Ch1, duty);
    }
}
