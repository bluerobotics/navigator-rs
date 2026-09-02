use std::error::Error;
use std::time::{Duration, Instant};

use embedded_hal::blocking::spi::{Transfer, Write};
use linux_embedded_hal::spidev::{SpiModeFlags, SpidevOptions};
use linux_embedded_hal::Spidev;

use crate::peripherals::{
    AnyHardware, MagnetometerSensor, PeripheralClass, PeripheralInfo, Peripherals,
};

const DEFAULT_SPI_DEVICE: &str = "/dev/spidev1.1";
const SPI_READ_FLAG: u8 = 0x80;
const PRODUCT_ID: u8 = 0x30;

/// Output of a null field in 16 bit mode.
const ZERO_OFFSET: f32 = 32768.0;
/// 4096 counts per gauss in 16 bit mode, one gauss being 100 µT.
const COUNTS_TO_UT: f32 = 100.0 / 4096.0;
/// How often the bridge offset is measured again. The offset drifts with
/// temperature, and a SET/RESET pair costs two measurements.
const DEGAUSS_INTERVAL: Duration = Duration::from_secs(10);

/// Wrapper type for results
pub type Result<T> = std::result::Result<T, Box<dyn Error>>;

/// MMC5983MA Registers
enum Register {
    /// Magnetic field data registers
    Xout0 = 0x00,
    Status = 0x08,
    Control0 = 0x09,
    Control1 = 0x0A,
    ProductId = 0x2F,
}

/// Bits of [`Register::Control0`]
const CONTROL0_TAKE_MEASUREMENT: u8 = 0x01;
const CONTROL0_SET: u8 = 0x08;
const CONTROL0_RESET: u8 = 0x10;

/// Maps the raw sensor axes into the board frame, x forward, y right, z down.
///
/// U15 sits rotated 90° on the bottom side of the board. This is the
/// `ROTATION_YAW_180` that ArduPilot applies to this part on Navigator, and it
/// goes with the sign that [`Mmc5983ma::degauss`] takes the difference in.
fn to_board_frame(field: [f32; 3]) -> (f32, f32, f32) {
    (-field[0], -field[1], field[2])
}

pub struct Mmc5983ma {
    spi_device: Spidev,
    offset: Option<[f32; 3]>,
    last_degauss: Instant,
}

pub struct Mmc5983maDevice {
    mmc5983ma: Mmc5983ma,
    info: PeripheralInfo,
}

impl Mmc5983maDevice {
    pub fn builder() -> Mmc5983maBuilder {
        Mmc5983maBuilder::new()
    }

    pub fn get_peripheral_info(&self) -> &PeripheralInfo {
        &self.info
    }
}

impl AnyHardware for Mmc5983maDevice {
    fn peripheral(&self) -> Option<Peripherals> {
        Some(Peripherals::Mmc5983ma)
    }

    fn as_magnetometer_sensor(&mut self) -> Option<&mut dyn MagnetometerSensor> {
        Some(self)
    }
}

pub struct Mmc5983maBuilder {
    spi_device: String,
    info: PeripheralInfo,
}

impl Mmc5983maBuilder {
    pub fn new() -> Self {
        Mmc5983maBuilder {
            spi_device: DEFAULT_SPI_DEVICE.to_string(),
            info: PeripheralInfo {
                peripheral: Peripherals::Mmc5983ma,
                class: vec![PeripheralClass::Magnetometer],
            },
        }
    }

    /// Sets the SPI device to be used.
    ///
    /// # Arguments
    ///
    /// * `device` - The SPI device (e.g., "/dev/spidev1.1").
    pub fn with_spi_device(mut self, device: &str) -> Self {
        self.spi_device = device.to_string();
        self
    }

    pub fn with_peripheral_info(mut self, info: PeripheralInfo) -> Self {
        self.info = info;
        self
    }

    pub fn build(self) -> Result<Mmc5983maDevice> {
        let mut spi = Spidev::open(self.spi_device)?;
        let options = SpidevOptions::new()
            .bits_per_word(8)
            .max_speed_hz(10_000_000)
            .mode(SpiModeFlags::SPI_MODE_0)
            .build();
        spi.configure(&options)?;

        let mut sensor = Mmc5983ma {
            spi_device: spi,
            offset: None,
            last_degauss: Instant::now(),
        };

        sensor.begin()?;

        Ok(Mmc5983maDevice {
            mmc5983ma: sensor,
            info: self.info,
        })
    }
}

impl Default for Mmc5983maBuilder {
    fn default() -> Self {
        Self::new()
    }
}

impl Mmc5983ma {
    fn write_reg(&mut self, reg: Register, value: u8) -> Result<()> {
        self.spi_device.write(&[reg as u8, value])?;
        Ok(())
    }

    fn read_reg(&mut self, reg: Register) -> Result<u8> {
        let mut frame = [reg as u8 | SPI_READ_FLAG, 0];
        self.spi_device.transfer(&mut frame)?;
        Ok(frame[1])
    }

    fn begin(&mut self) -> Result<()> {
        // Reading the product ID over SPI fails on the first attempts, the
        // part needs a few milliseconds before it answers
        let mut product_id = 0;
        for _ in 0..10 {
            std::thread::sleep(Duration::from_millis(5));
            product_id = self.read_reg(Register::ProductId)?;
            if product_id == PRODUCT_ID {
                break;
            }
        }
        if product_id != PRODUCT_ID {
            return Err(format!("MMC5983MA: Invalid product ID: 0x{:02X}", product_id).into());
        }

        self.write_reg(Register::Control1, 0x80)?;
        std::thread::sleep(Duration::from_millis(15));

        // Widest bandwidth, 100 Hz
        self.write_reg(Register::Control1, 0x00)?;

        self.degauss()?;

        Ok(())
    }

    /// Takes a single measurement, in µT, without removing the bridge offset.
    fn measure(&mut self) -> Result<[f32; 3]> {
        self.write_reg(Register::Control0, CONTROL0_TAKE_MEASUREMENT)?;

        let mut ready = false;
        for _ in 0..20 {
            std::thread::sleep(Duration::from_millis(1));
            if self.read_reg(Register::Status)? & 0x01 != 0 {
                ready = true;
                break;
            }
        }
        if !ready {
            return Err("MMC5983MA: Timed out waiting for a measurement".into());
        }

        let mut frame = [0u8; 7];
        frame[0] = Register::Xout0 as u8 | SPI_READ_FLAG;
        self.spi_device.transfer(&mut frame)?;

        Ok([
            u16::from_be_bytes([frame[1], frame[2]]) as f32 - ZERO_OFFSET,
            u16::from_be_bytes([frame[3], frame[4]]) as f32 - ZERO_OFFSET,
            u16::from_be_bytes([frame[5], frame[6]]) as f32 - ZERO_OFFSET,
        ]
        .map(|counts| counts * COUNTS_TO_UT))
    }

    /// Drives the SET and RESET coils and measures on both, which flips the
    /// field but not the bridge offset. Half the sum is then the offset and
    /// half the difference the field, so this both measures and recalibrates.
    ///
    /// The difference is taken as reset minus set, the same way ArduPilot does
    /// it, which inverts the field and is why [`to_board_frame`] undoes it.
    fn degauss(&mut self) -> Result<[f32; 3]> {
        let set = self.magnetize(CONTROL0_SET)?;
        let reset = self.magnetize(CONTROL0_RESET)?;

        let field = std::array::from_fn(|axis| (reset[axis] - set[axis]) * 0.5);
        let offset: [f32; 3] = std::array::from_fn(|axis| (set[axis] + reset[axis]) * 0.5);

        // Low pass the offset, a single pair is noisier than the field itself
        self.offset = Some(match self.offset {
            Some(previous) => std::array::from_fn(|axis| previous[axis] * 0.5 + offset[axis] * 0.5),
            None => offset,
        });

        self.last_degauss = Instant::now();
        Ok(field)
    }

    /// Drives one of the coils and measures once it has settled.
    fn magnetize(&mut self, coil: u8) -> Result<[f32; 3]> {
        self.write_reg(Register::Control0, coil)?;
        // The datasheet asks for 1 ms between driving a coil and measuring
        std::thread::sleep(Duration::from_millis(1));
        self.measure()
    }

    fn read_field(&mut self) -> Result<[f32; 3]> {
        if self.last_degauss.elapsed() >= DEGAUSS_INTERVAL {
            return self.degauss();
        }

        let field = self.measure()?;
        let offset = self.offset.unwrap_or_default();
        Ok(std::array::from_fn(|axis| field[axis] - offset[axis]))
    }
}

impl MagnetometerSensor for Mmc5983maDevice {
    fn read_magnetic_field(&mut self) -> Result<(f32, f32, f32)> {
        let field = self.mmc5983ma.read_field()?;
        Ok(to_board_frame(field))
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use std::thread::sleep;

    #[test]
    fn board_frame_flips_x_and_y() {
        assert_eq!(to_board_frame([30.0, -12.0, 45.0]), (-30.0, 12.0, 45.0));
    }

    #[ignore]
    #[test]
    fn test_mmc5983ma_pi_4() {
        let mut mag = Mmc5983maDevice::builder()
            .build()
            .expect("Failed to build MMC5983MA");
        for _ in 0..10 {
            println!("MMC5983MA readings: {:?}", mag.read_magnetic_field());
            sleep(Duration::from_millis(100));
        }
    }
}
