use std::error::Error;

use embedded_hal::blocking::spi::{Transfer, Write};
use linux_embedded_hal::spidev::{SpiModeFlags, SpidevOptions};
use linux_embedded_hal::Spidev;

use crate::peripherals::{
    AccelerometerSensor, AnyHardware, GyroscopeSensor, PeripheralClass, PeripheralInfo, Peripherals,
};

const DEFAULT_SPI_DEVICE: &str = "/dev/spidev1.2";
const SPI_READ_FLAG: u8 = 0x80;
const CHIP_ID: u8 = 0x70;

/// Standard gravity, the accelerometer reports in g.
const GRAVITY: f32 = 9.80665;
/// Accelerometer sensitivity at ±16 g, in m/s² per LSB.
const ACCEL_SCALE: f32 = GRAVITY * 16.0 / 32768.0;
/// Gyroscope sensitivity at ±2000 dps, 70 mdps per LSB turned into rad/s.
const GYRO_SCALE: f32 = 70e-3 * std::f32::consts::PI / 180.0;

/// Wrapper type for results
pub type Result<T> = std::result::Result<T, Box<dyn Error>>;

/// LSM6DSV Registers
enum Register {
    WhoAmI = 0x0F,
    /// Accelerometer operating mode and output data rate
    Ctrl1 = 0x10,
    /// Gyroscope operating mode and output data rate
    Ctrl2 = 0x11,
    Ctrl3 = 0x12,
    /// Gyroscope full scale
    Ctrl6 = 0x15,
    /// Accelerometer full scale
    Ctrl8 = 0x17,
    Status = 0x1E,
    /// Gyroscope data registers
    OutxLG = 0x22,
    /// Accelerometer data registers
    OutxLA = 0x28,
    HaodrCfg = 0x62,
}

/// Maps the raw sensor axes into the board frame, x forward, y right, z down.
///
/// U6 sits rotated 270° on the bottom side of the board, which is the
/// `ROTATION_YAW_270` that ArduPilot applies to this part on Navigator.
fn to_board_frame(axes: [i16; 3], scale: f32) -> (f32, f32, f32) {
    let [x, y, z] = axes.map(|axis| axis as f32 * scale);
    (y, -x, z)
}

pub struct Lsm6dsv {
    spi_device: Spidev,
}

pub struct Lsm6dsvDevice {
    lsm6dsv: Lsm6dsv,
    info: PeripheralInfo,
}

impl Lsm6dsvDevice {
    pub fn builder() -> Lsm6dsvBuilder {
        Lsm6dsvBuilder::new()
    }

    pub fn get_peripheral_info(&self) -> &PeripheralInfo {
        &self.info
    }
}

impl AnyHardware for Lsm6dsvDevice {
    fn peripheral(&self) -> Option<Peripherals> {
        Some(Peripherals::Lsm6dsv)
    }

    fn as_gyroscope_sensor(&mut self) -> Option<&mut dyn GyroscopeSensor> {
        Some(self)
    }

    fn as_accelerometer_sensor(&mut self) -> Option<&mut dyn AccelerometerSensor> {
        Some(self)
    }
}

pub struct Lsm6dsvBuilder {
    spi_device: String,
    info: PeripheralInfo,
}

impl Lsm6dsvBuilder {
    pub fn new() -> Self {
        Lsm6dsvBuilder {
            spi_device: DEFAULT_SPI_DEVICE.to_string(),
            info: PeripheralInfo {
                peripheral: Peripherals::Lsm6dsv,
                class: vec![PeripheralClass::Accelerometer, PeripheralClass::Gyroscope],
            },
        }
    }

    /// Sets the SPI device to be used.
    ///
    /// # Arguments
    ///
    /// * `device` - The SPI device (e.g., "/dev/spidev1.2").
    pub fn with_spi_device(mut self, device: &str) -> Self {
        self.spi_device = device.to_string();
        self
    }

    pub fn with_peripheral_info(mut self, info: PeripheralInfo) -> Self {
        self.info = info;
        self
    }

    pub fn build(self) -> Result<Lsm6dsvDevice> {
        let mut spi = Spidev::open(self.spi_device)?;
        let options = SpidevOptions::new()
            .bits_per_word(8)
            .max_speed_hz(10_000_000)
            .mode(SpiModeFlags::SPI_MODE_0)
            .build();
        spi.configure(&options)?;

        let mut sensor = Lsm6dsv { spi_device: spi };
        sensor.begin()?;

        Ok(Lsm6dsvDevice {
            lsm6dsv: sensor,
            info: self.info,
        })
    }
}

impl Default for Lsm6dsvBuilder {
    fn default() -> Self {
        Self::new()
    }
}

impl Lsm6dsv {
    fn write_reg(&mut self, reg: Register, value: u8) -> Result<()> {
        self.spi_device.write(&[reg as u8, value])?;
        Ok(())
    }

    fn read_reg(&mut self, reg: Register) -> Result<u8> {
        let mut frame = [reg as u8 | SPI_READ_FLAG, 0];
        self.spi_device.transfer(&mut frame)?;
        Ok(frame[1])
    }

    /// Reads the three consecutive 16 bit outputs starting at `reg`.
    fn read_axes(&mut self, reg: Register) -> Result<[i16; 3]> {
        let mut frame = [0u8; 7];
        frame[0] = reg as u8 | SPI_READ_FLAG;
        self.spi_device.transfer(&mut frame)?;
        Ok([
            i16::from_le_bytes([frame[1], frame[2]]),
            i16::from_le_bytes([frame[3], frame[4]]),
            i16::from_le_bytes([frame[5], frame[6]]),
        ])
    }

    fn begin(&mut self) -> Result<()> {
        let chip_id = self.read_reg(Register::WhoAmI)?;
        if chip_id != CHIP_ID {
            return Err(format!("LSM6DSV: Invalid chip ID: 0x{:02X}", chip_id).into());
        }

        self.reset()?;

        // ±2000 dps and ±16 g, the ranges ArduPilot runs this part at
        self.write_reg(Register::Ctrl6, 0x04)?;
        self.write_reg(Register::Ctrl8, 0x03)?;

        // High-accuracy output data rate, mode 1, at 1000 Hz
        self.write_reg(Register::HaodrCfg, 0x01)?;
        self.write_reg(Register::Ctrl1, 0x19)?;
        self.write_reg(Register::Ctrl2, 0x19)?;

        // Block data update, so a burst read never mixes two samples,
        // and address auto-increment
        self.write_reg(Register::Ctrl3, 0x44)?;

        self.wait_for_data()?;

        Ok(())
    }

    fn reset(&mut self) -> Result<()> {
        self.write_reg(Register::Ctrl3, 0x01)?;

        for _ in 0..100 {
            std::thread::sleep(std::time::Duration::from_millis(1));
            if self.read_reg(Register::Ctrl3)? & 0x01 == 0 {
                return Ok(());
            }
        }

        Err("LSM6DSV: Timed out waiting for the software reset".into())
    }

    /// Waits for the first accelerometer and gyroscope samples, the output
    /// registers read zero until then.
    fn wait_for_data(&mut self) -> Result<()> {
        for _ in 0..20 {
            std::thread::sleep(std::time::Duration::from_millis(1));
            if self.read_reg(Register::Status)? & 0x03 == 0x03 {
                return Ok(());
            }
        }

        Err("LSM6DSV: Timed out waiting for the first sample".into())
    }
}

impl GyroscopeSensor for Lsm6dsvDevice {
    fn read_angular_velocity(&mut self) -> Result<(f32, f32, f32)> {
        let axes = self.lsm6dsv.read_axes(Register::OutxLG)?;
        Ok(to_board_frame(axes, GYRO_SCALE))
    }
}

impl AccelerometerSensor for Lsm6dsvDevice {
    fn read_acceleration(&mut self) -> Result<(f32, f32, f32)> {
        let axes = self.lsm6dsv.read_axes(Register::OutxLA)?;
        Ok(to_board_frame(axes, ACCEL_SCALE))
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use std::{thread::sleep, time::Duration};

    #[test]
    fn board_frame_rotates_the_sensor_by_270_degrees() {
        // At ±16 g one g is 2048 counts, and the sensor x axis points
        // along -y of the board
        for (axes, expected) in [
            ([2048, 0, 0], (0.0, -GRAVITY, 0.0)),
            ([0, 2048, 0], (GRAVITY, 0.0, 0.0)),
            ([0, 0, 2048], (0.0, 0.0, GRAVITY)),
        ] {
            let (x, y, z) = to_board_frame(axes, ACCEL_SCALE);
            approx::assert_relative_eq!(x, expected.0);
            approx::assert_relative_eq!(y, expected.1);
            approx::assert_relative_eq!(z, expected.2);
        }
    }

    #[ignore]
    #[test]
    fn test_lsm6dsv_pi_4() {
        let mut imu = Lsm6dsvDevice::builder()
            .build()
            .expect("Failed to build LSM6DSV");
        for _ in 0..10 {
            println!("LSM6DSV gyroscope: {:?}", imu.read_angular_velocity());
            println!("LSM6DSV accelerometers: {:?}", imu.read_acceleration());
            sleep(Duration::from_millis(100));
        }
    }
}
