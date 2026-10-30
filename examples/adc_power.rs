//! Lesson 7: VREF-calibrated supply voltage, die temperature and board revision.

#![no_std]
#![no_main]
#![forbid(unsafe_code)]

use embassy_stm32::adc::{Adc, SampleTime};
use embassy_stm32::gpio::Input;
use embassy_stm32::peripherals::{ADC1, ADC3, PF10};
use embassy_time::Timer;
use panic_rtt_target as _;

struct Peripherals {
    adc1: Adc<'static, ADC1>,
    adc3: Adc<'static, ADC3>,
    temperature: embassy_stm32::adc::Temperature,
    battery: embassy_stm32::Peri<'static, PF10>,
    hw0: Input<'static>,
    hw1: Input<'static>,
    hw2: Input<'static>,
    volts_per_count: f32,
}

async fn init() -> Peripherals {
    use embassy_stm32::gpio::Pull;
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
    let mut adc1 = Adc::new(p.ADC1);
    let adc3 = Adc::new(p.ADC3);
    let mut vref = adc1.enable_vrefint();
    let temperature = adc1.enable_temperature();
    let battery = p.PF10;
    let hw0 = Input::new(p.PC0, Pull::None);
    let hw1 = Input::new(p.PC1, Pull::None);
    let hw2 = Input::new(p.PC2, Pull::None);
    Timer::after_millis(1).await;
    let mut sum = 0_u32;
    for _ in 0..200 {
        sum += u32::from(adc1.blocking_read(&mut vref, SampleTime::CYCLES480));
    }
    assert!(sum != 0, "VREFINT reads zero");
    let volts_per_count = 200.0 * 1.2 / sum as f32;
    Peripherals {
        adc1,
        adc3,
        temperature,
        battery,
        hw0,
        hw1,
        hw2,
        volts_per_count,
    }
}

#[embassy_executor::main]
async fn main(_spawner: embassy_executor::Spawner) {
    let Peripherals {
        mut adc1,
        mut adc3,
        mut temperature,
        mut battery,
        hw0,
        hw1,
        hw2,
        volts_per_count,
    } = init().await;
    loop {
        let raw = adc3.blocking_read(&mut battery, SampleTime::CYCLES480);
        let voltage_v = f32::from(raw) * volts_per_count * (222.0 / 22.0);
        let temp_raw = adc1.blocking_read(&mut temperature, SampleTime::CYCLES480);
        let temperature_c = (f32::from(temp_raw) * volts_per_count - 0.76) * 400.0 + 25.0;
        let revision = u8::from(hw0.is_high())
            | (u8::from(hw1.is_high()) << 1)
            | (u8::from(hw2.is_high()) << 2);
        rtt_target::rprintln!(
            "supply {:.2} V, die {:.1} C, revision {}",
            voltage_v,
            temperature_c,
            revision
        );
        Timer::after_millis(100).await;
    }
}
