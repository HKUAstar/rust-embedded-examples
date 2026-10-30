//! Lesson 12: display a moving checkerboard on the I2C2 128x64 OLED.

#![no_std]
#![no_main]
#![forbid(unsafe_code)]

use embassy_stm32::i2c::{self, I2c};
use embassy_time::Timer;
use panic_rtt_target as _;

const ADDRESS: u8 = 0x3c;

async fn init() -> I2c<'static, embassy_stm32::mode::Blocking, i2c::Master> {
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
    let mut config = i2c::Config::default();
    config.frequency = Hertz(400_000);
    config.timeout = Duration::from_millis(10);
    let mut i2c = I2c::new_blocking(p.I2C2, p.PF1, p.PF0, config);
    Timer::after_millis(100).await;
    // Page addressing, 64 rows, internal charge pump, normal display.
    for byte in [
        0xae, 0x20, 0x02, 0xc8, 0x40, 0x81, 0xff, 0xa1, 0xa6, 0xa8, 0x3f, 0xa4, 0xd3, 0x00, 0xd5,
        0xf0, 0xd9, 0x22, 0xda, 0x12, 0xdb, 0x20, 0x8d, 0x14, 0xaf,
    ] {
        i2c.blocking_write(ADDRESS, &[0, byte])
            .expect("OLED command");
    }
    i2c
}

#[embassy_executor::main]
async fn main(_spawner: embassy_executor::Spawner) {
    let mut i2c = init().await;
    let mut phase = false;
    loop {
        for page in 0..8_u8 {
            i2c.blocking_write(ADDRESS, &[0, 0xb0 + page, 0, 0x10])
                .expect("OLED page");
            let mut line = [0; 129];
            line[0] = 0x40;
            for (x, pixel) in line[1..].iter_mut().enumerate() {
                *pixel = if (x / 8 + usize::from(page) + usize::from(phase)).is_multiple_of(2) {
                    0xff
                } else {
                    0
                };
            }
            i2c.blocking_write(ADDRESS, &line).expect("OLED pixels");
        }
        phase = !phase;
        Timer::after_secs(1).await;
    }
}
