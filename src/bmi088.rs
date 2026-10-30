//! The BMI088 IMU: an accelerometer and a gyroscope in one package, each its own SPI device with
//! its own chip select, on one bus. `Bmi088::new` checks both chips, resets and configures them,
//! and reads the configuration back; `read` returns one sample in SI units, on the chip's axes as
//! marked on the package, with the chip's temperature.
//!
//! Generic over embedded-hal's `SpiBus` and `OutputPin`, so every board with the chip uses this one
//! driver: the board crate names the types and opens the bus in mode 3 at up to 10 MHz.

use core::convert::Infallible;
use embedded_hal::delay::DelayNs;
use embedded_hal::digital::OutputPin;
use embedded_hal::spi::SpiBus;

/// Set on a register address to read it.
const READ: u8 = 0x80;

const ACC_CHIP_ID: u8 = 0x00;
const ACC_X_LSB: u8 = 0x12;
const TEMP_MSB: u8 = 0x22;
const ACC_CONF: u8 = 0x40;
const ACC_RANGE: u8 = 0x41;
const ACC_PWR_CONF: u8 = 0x7C;
const ACC_PWR_CTRL: u8 = 0x7D;
const ACC_SOFTRESET: u8 = 0x7E;
const GYRO_CHIP_ID: u8 = 0x00;
const RATE_X_LSB: u8 = 0x02;
const GYRO_RANGE: u8 = 0x0F;
const GYRO_BANDWIDTH: u8 = 0x10;
const GYRO_SOFTRESET: u8 = 0x14;
/// Written to either chip's soft-reset register, resets it.
const SOFTRESET: u8 = 0xB6;

/// The accelerometer's configuration, in the order written, each with the wait after it, ms:
/// active, then on, after which its data is valid in 50 ms (datasheet 3); then 1600 Hz with the
/// normal filter, at ±6 g.
const ACCEL_CONFIG: [(u8, u8, u32); 4] = [
    (ACC_PWR_CONF, 0x00, 5),
    (ACC_PWR_CTRL, 0x04, 50),
    (ACC_CONF, 0xAC, 5),
    (ACC_RANGE, 0x01, 5),
];
/// The gyroscope's: ±2000 °/s; 2000 Hz with a 230 Hz filter.
const GYRO_CONFIG: [(u8, u8); 2] = [(GYRO_RANGE, 0x00), (GYRO_BANDWIDTH, 0x01)];
/// GYRO_BANDWIDTH's bit 7 reads as 1 whatever was written.
const GYRO_BANDWIDTH_READS_SET: u8 = 0x80;

const INT1_IO_CTRL: u8 = 0x53;
const INT1_INT2_MAP_DATA: u8 = 0x58;
const GYRO_INT_CTRL: u8 = 0x15;
const INT3_INT4_IO_CONF: u8 = 0x16;
const INT3_INT4_IO_MAP: u8 = 0x18;

/// ±6 g full scale (ACC_RANGE 0x01), set by `new`.
const ACCEL_M_S2_PER_LSB: f32 = 6.0 * 9.80665 / 32768.0;
/// ±2000 °/s full scale (GYRO_RANGE 0x00), set by `new`.
const GYRO_RAD_S_PER_LSB: f32 = 2000.0_f32.to_radians() / 32768.0;

/// Which chip did not answer with its ID, could not be written, or did not read back what was.
#[derive(Clone, Copy, Debug, Eq, PartialEq)]
pub enum Error {
    Accelerometer,
    Gyroscope,
}

impl core::fmt::Display for Error {
    fn fmt(&self, f: &mut core::fmt::Formatter<'_>) -> core::fmt::Result {
        match self {
            Self::Accelerometer => f.write_str("accelerometer initialization failed"),
            Self::Gyroscope => f.write_str("gyroscope initialization failed"),
        }
    }
}

impl core::error::Error for Error {}

/// One acceleration, angular-rate and temperature reading on the sensor axes.
#[derive(Clone, Copy, Debug)]
pub struct Sample {
    pub accel_m_s2: [f32; 3],
    pub gyro_rad_s: [f32; 3],
    /// The chip's own sensor, in 0.125 °C steps, updated every 1.28 s.
    pub temperature_c: f32,
}

/// The chip selects are GPIOs that cannot fail, as a microcontroller's are.
pub struct Bmi088<S, P> {
    spi: S,
    accel_cs: P,
    gyro_cs: P,
}

impl<S: SpiBus, P: OutputPin<Error = Infallible>> Bmi088<S, P> {
    /// SPI mode 3, up to 10 MHz, chip selects idle high. Initialization blocks for about 97 ms and
    /// soft-resets both chips before configuring and reading back each register.
    ///
    /// A board reset alone does not reset the sensor: on the bench, a leftover ±24 g range made
    /// readings one quarter of the true acceleration.
    ///
    /// Accelerometer: 1600 Hz, normal filter about 235 Hz, ±6 g. Gyroscope: 2000 Hz, 230 Hz filter,
    /// ±2000 °/s.
    pub fn new(spi: S, accel_cs: P, gyro_cs: P, delay: &mut impl DelayNs) -> Result<Self, Error> {
        let mut imu = Self {
            spi,
            accel_cs,
            gyro_cs,
        };
        imu.accel_to_spi(delay);
        let mut id = [READ | ACC_CHIP_ID, 0, 0];
        if imu.accel(&mut id).is_err() || id[2] != 0x1E {
            return Err(Error::Accelerometer);
        }
        let mut id = [READ | GYRO_CHIP_ID, 0];
        if imu.gyro(&mut id).is_err() || id[1] != 0x0F {
            return Err(Error::Gyroscope);
        }
        imu.accel(&mut [ACC_SOFTRESET, SOFTRESET])
            .map_err(|_| Error::Accelerometer)?;
        // A soft reset is "largely equivalent to a power cycle" (datasheet 4.8), so the
        // accelerometer is switched to SPI again, as after power-on; it restarts suspended.
        imu.accel_to_spi(delay);
        imu.gyro(&mut [GYRO_SOFTRESET, SOFTRESET])
            .map_err(|_| Error::Gyroscope)?;
        // The gyroscope's start-up time after a reset.
        delay.delay_ms(30);
        for (reg, value, wait_ms) in ACCEL_CONFIG {
            imu.accel(&mut [reg, value])
                .map_err(|_| Error::Accelerometer)?;
            delay.delay_ms(wait_ms);
        }
        for (reg, value) in GYRO_CONFIG {
            imu.gyro(&mut [reg, value]).map_err(|_| Error::Gyroscope)?;
        }
        for (reg, value, _) in ACCEL_CONFIG {
            let mut read = [READ | reg, 0, 0];
            if imu.accel(&mut read).is_err() || read[2] != value {
                return Err(Error::Accelerometer);
            }
        }
        for (reg, value) in GYRO_CONFIG {
            let mut read = [READ | reg, 0];
            let set = if reg == GYRO_BANDWIDTH {
                GYRO_BANDWIDTH_READS_SET
            } else {
                0
            };
            if imu.gyro(&mut read).is_err() || read[1] != value | set {
                return Err(Error::Gyroscope);
            }
        }
        Ok(imu)
    }

    /// The accelerometer starts in I2C mode until a rising edge on its chip select: this read's,
    /// which is thrown away. It also waits the 1 ms the chip needs after power-on or a reset.
    fn accel_to_spi(&mut self, delay: &mut impl DelayNs) {
        let _ = self.accel(&mut [READ | ACC_CHIP_ID, 0, 0]);
        delay.delay_ms(1);
    }

    /// Active-low data-ready interrupts: accelerometer INT1 at 1600 Hz, gyro INT3 at 2000 Hz.
    pub fn enable_data_ready(&mut self) -> Result<(), Error> {
        // INT1 an output, push-pull, active low; data ready on INT1.
        for (reg, value) in [(INT1_IO_CTRL, 0x08), (INT1_INT2_MAP_DATA, 0x04)] {
            self.accel(&mut [reg, value])
                .map_err(|_| Error::Accelerometer)?;
        }
        // New-data interrupt on; INT3 push-pull, active low; data ready on INT3.
        for (reg, value) in [
            (GYRO_INT_CTRL, 0x80),
            (INT3_INT4_IO_CONF, 0x0C),
            (INT3_INT4_IO_MAP, 0x01),
        ] {
            self.gyro(&mut [reg, value]).map_err(|_| Error::Gyroscope)?;
        }
        Ok(())
    }

    /// Both chips' latest data and the temperature. Each chip's six bytes are read in one burst,
    /// which the chip keeps from mixing two samples; the two chips sample on their own clocks. The
    /// temperature is reported without a range check; heater users must validate it.
    pub fn read(&mut self) -> Result<Sample, S::Error> {
        // The accelerometer returns a dummy byte before the data; the gyroscope does not.
        let mut accel = [READ | ACC_X_LSB, 0, 0, 0, 0, 0, 0, 0];
        self.accel(&mut accel)?;
        let mut gyro = [READ | RATE_X_LSB, 0, 0, 0, 0, 0, 0];
        self.gyro(&mut gyro)?;
        let mut temperature = [READ | TEMP_MSB, 0, 0, 0];
        self.accel(&mut temperature)?;
        let temperature_c = temperature_c(temperature[2], temperature[3]);
        Ok(Sample {
            accel_m_s2: axes(&accel[2..]).map(|raw| raw * ACCEL_M_S2_PER_LSB),
            gyro_rad_s: axes(&gyro[1..]).map(|raw| raw * GYRO_RAD_S_PER_LSB),
            temperature_c,
        })
    }

    fn accel(&mut self, buf: &mut [u8]) -> Result<(), S::Error> {
        transfer(&mut self.spi, &mut self.accel_cs, buf)
    }

    fn gyro(&mut self, buf: &mut [u8]) -> Result<(), S::Error> {
        transfer(&mut self.spi, &mut self.gyro_cs, buf)
    }
}

/// One transaction with `cs` held low: `buf` goes out and is replaced by what comes back. The bus
/// is flushed before `cs` goes high, since a `SpiBus` may return before its last byte is out.
fn transfer<S: SpiBus>(
    spi: &mut S,
    cs: &mut impl OutputPin<Error = Infallible>,
    buf: &mut [u8],
) -> Result<(), S::Error> {
    let Ok(()) = cs.set_low();
    let result = spi.transfer_in_place(buf).and_then(|()| spi.flush());
    let Ok(()) = cs.set_high();
    result
}

/// TEMP_MSB and TEMP_LSB: an 11-bit two's-complement count, the MSB's 8 bits over the LSB's top 3,
/// of 0.125 °C from 23 °C.
fn temperature_c(msb: u8, lsb: u8) -> f32 {
    let count = i16::from_be_bytes([msb, lsb]) >> 5;
    f32::from(count) * 0.125 + 23.0
}

/// Three little-endian i16s, X, Y and Z.
fn axes(bytes: &[u8]) -> [f32; 3] {
    core::array::from_fn(|i| f32::from(i16::from_le_bytes([bytes[2 * i], bytes[2 * i + 1]])))
}
