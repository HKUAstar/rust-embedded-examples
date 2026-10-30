//! Lesson 11: configure the onboard magnetometer and report microtesla.

#![no_std]
#![no_main]
#![forbid(unsafe_code)]

use embassy_stm32::gpio::Output;
use embassy_stm32::i2c::{self, I2c};
use embassy_time::Timer;
use panic_rtt_target as _;

const ADDRESS: u8 = 0x0e;

struct Peripherals {
    i2c: I2c<'static, embassy_stm32::mode::Blocking, i2c::Master>,
    reset: Output<'static>,
}

async fn init() -> Peripherals {
    use embassy_stm32::gpio::{Level, Speed};
    use embassy_stm32::rcc::{
        AHBPrescaler, APBPrescaler, Hse, HseMode, Pll, PllMul, PllPDiv, PllPreDiv, PllQDiv,
        PllSource, Sysclk,
    };
    use embassy_stm32::time::Hertz;
    use embassy_time::Duration;

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
    let mut reset = Output::new(p.PG6, Level::Low, Speed::Low);
    Timer::after_millis(50).await;
    reset.set_high();
    Timer::after_millis(50).await;
    let mut config = i2c::Config::default();
    config.frequency = Hertz(400_000);
    config.timeout = Duration::from_millis(10);
    let mut i2c = I2c::new_blocking(p.I2C3, p.PA8, p.PC9, config);
    let mut id = [0];
    i2c.blocking_write_read(ADDRESS, &[0], &mut id)
        .expect("IST8310 ID read");
    assert_eq!(id[0], 0x10, "IST8310 ID");
    for (register, value) in [(0x0b, 0x08), (0x41, 0x09), (0x42, 0xc0), (0x0a, 0x0b)] {
        i2c.blocking_write(ADDRESS, &[register, value])
            .expect("IST8310 configure");
        Timer::after_micros(150).await;
        i2c.blocking_write_read(ADDRESS, &[register], &mut id)
            .expect("IST8310 readback");
        assert_eq!(id[0], value, "IST8310 register readback");
    }
    Peripherals { i2c, reset }
}

#[embassy_executor::main]
async fn main(_spawner: embassy_executor::Spawner) {
    let Peripherals {
        mut i2c,
        reset: _reset,
    } = init().await;
    loop {
        let mut data = [0; 7];
        i2c.blocking_write_read(ADDRESS, &[2], &mut data)
            .expect("IST8310 sample");
        if data[0] & 1 != 0 {
            let field_ut: [f32; 3] = core::array::from_fn(|i| {
                f32::from(i16::from_le_bytes([data[1 + 2 * i], data[2 + 2 * i]])) * 0.3
            });
            rtt_target::rprintln!("magnetic field {:?} uT", field_ut);
        }
        Timer::after_millis(10).await;
    }
}
