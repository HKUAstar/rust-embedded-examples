//! Lesson 10: erase, write and verify the reserved final 128 KiB flash sector.

#![no_std]
#![no_main]
#![forbid(unsafe_code)]

use embassy_stm32::flash::Flash;
use panic_rtt_target as _;

// memory.x excludes sector 11 from all executable images.
const START: u32 = 0xE0000;
const END: u32 = 0x100000;

fn init() -> Flash<'static, embassy_stm32::flash::Blocking> {
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
    Flash::new_blocking(p.FLASH)
}

#[embassy_executor::main]
async fn main(_spawner: embassy_executor::Spawner) {
    let mut flash = init();
    let mut erased = [0; 12];
    let mut written = [0; 12];
    flash.blocking_erase(START, END).expect("erase sector 11");
    flash
        .blocking_read(START, &mut erased)
        .expect("read erased sector");
    assert_eq!(erased, [0xff; 12], "sector is not erased");
    flash
        .blocking_write(START, b"RoboMaster\r\n")
        .expect("write sector 11");
    flash
        .blocking_read(START, &mut written)
        .expect("read written data");
    assert_eq!(&written, b"RoboMaster\r\n", "flash verification failed");
    rtt_target::rprintln!("sector 11 erased and verified: {:?}", written);
    core::future::pending::<()>().await;
}
