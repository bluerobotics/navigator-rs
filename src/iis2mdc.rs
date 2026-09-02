use std::error::Error;
use std::path::PathBuf;

use embedded_hal::blocking::i2c::{Write, WriteRead};
use linux_embedded_hal::I2cdev;

use crate::peripherals::{
    AnyHardware, MagnetometerSensor, PeripheralClass, PeripheralInfo, Peripherals,
};

const DEFAULT_I2C_ADDRESS: u8 = 0x1E;
const DEFAULT_I2C_PATH: &str = "/dev/i2c-1";
const CHIP_ID: u8 = 0x40;

/// Sensitivity, 1.5 mgauss per LSB expressed in µT.
const RANGE_SCALE: f32 = 0.15;

/// Wrapper type for results
pub type Result<T> = std::result::Result<T, Box<dyn Error>>;

/// IIS2MDC Registers
enum Register {
    WhoAmI = 0x4F,
    CfgRegA = 0x60,
    CfgRegB = 0x61,
    CfgRegC = 0x62,
    StatusReg = 0x67,
    /// Magnetic field data registers
    OutxLReg = 0x68,
}

/// Maps the raw sensor axes into the board frame, x forward, y right, z down.
///
/// U16 sits rotated 270° on the bottom side of the board. ArduPilot reaches
/// the same frame on Navigator by negating z in the driver and then applying
/// `ROTATION_ROLL_180_YAW_270`, which together swap x with y and leave z.
fn to_board_frame(axes: [i16; 3]) -> (f32, f32, f32) {
    let [x, y, z] = axes.map(|axis| axis as f32 * RANGE_SCALE);
    (-y, -x, z)
}

pub struct Iis2mdc {
    i2c_device: I2cdev,
    address: u8,
}

pub struct Iis2mdcDevice {
    iis2mdc: Iis2mdc,
    info: PeripheralInfo,
}

impl Iis2mdcDevice {
    pub fn builder() -> Iis2mdcBuilder {
        Iis2mdcBuilder::new()
    }

    pub fn get_peripheral_info(&self) -> &PeripheralInfo {
        &self.info
    }
}

impl AnyHardware for Iis2mdcDevice {
    fn peripheral(&self) -> Option<Peripherals> {
        Some(Peripherals::Iis2mdc)
    }

    fn as_magnetometer_sensor(&mut self) -> Option<&mut dyn MagnetometerSensor> {
        Some(self)
    }
}

pub struct Iis2mdcBuilder {
    i2c_address: u8,
    i2c_path: PathBuf,
}

impl Iis2mdcBuilder {
    pub fn new() -> Self {
        Iis2mdcBuilder {
            i2c_address: DEFAULT_I2C_ADDRESS,
            i2c_path: PathBuf::from(DEFAULT_I2C_PATH),
        }
    }

    pub fn address(&mut self, address: u16) -> &mut Self {
        self.i2c_address = address as u8;
        self
    }

    pub fn path(&mut self, path: impl Into<PathBuf>) -> &mut Self {
        self.i2c_path = path.into();
        self
    }

    pub fn build(&self) -> Result<Iis2mdcDevice> {
        let dev = I2cdev::new(&self.i2c_path)?;
        let mut sensor = Iis2mdc {
            i2c_device: dev,
            address: self.i2c_address,
        };

        sensor.begin()?;

        Ok(Iis2mdcDevice {
            iis2mdc: sensor,
            info: PeripheralInfo {
                peripheral: Peripherals::Iis2mdc,
                class: vec![PeripheralClass::Magnetometer],
            },
        })
    }
}

impl Default for Iis2mdcBuilder {
    fn default() -> Self {
        Self::new()
    }
}

impl Iis2mdc {
    fn write_reg(&mut self, reg: Register, value: u8) -> Result<()> {
        self.i2c_device.write(self.address, &[reg as u8, value])?;
        Ok(())
    }

    fn read_reg(&mut self, reg: Register) -> Result<u8> {
        let mut buf = [0u8; 1];
        self.i2c_device
            .write_read(self.address, &[reg as u8], &mut buf)?;
        Ok(buf[0])
    }

    fn begin(&mut self) -> Result<()> {
        let chip_id = self.read_reg(Register::WhoAmI)?;
        if chip_id != CHIP_ID {
            return Err(format!("IIS2MDC: Invalid chip ID: 0x{:02X}", chip_id).into());
        }

        // Continuous mode at 100 Hz, with temperature compensation
        self.write_reg(Register::CfgRegA, 0x8C)?;
        // Offset cancellation
        self.write_reg(Register::CfgRegB, 0x02)?;
        // Block data update, so a burst read never mixes two samples
        self.write_reg(Register::CfgRegC, 0x10)?;

        Ok(())
    }

    fn read_axes(&mut self) -> Result<[i16; 3]> {
        // Bit 3 of the status register is set once all three axes are ready.
        // A caller reading faster than the 100 Hz output rate waits here
        // instead of getting an error
        let mut ready = false;
        for _ in 0..20 {
            if self.read_reg(Register::StatusReg)? & 0x08 != 0 {
                ready = true;
                break;
            }
            std::thread::sleep(std::time::Duration::from_millis(1));
        }
        if !ready {
            return Err("IIS2MDC: Timed out waiting for a sample".into());
        }

        // A burst read wraps around inside the six axis registers rather than
        // running on into the temperature output, so only ask for those
        let mut buf = [0u8; 6];
        self.i2c_device
            .write_read(self.address, &[Register::OutxLReg as u8], &mut buf)?;

        Ok([
            i16::from_le_bytes([buf[0], buf[1]]),
            i16::from_le_bytes([buf[2], buf[3]]),
            i16::from_le_bytes([buf[4], buf[5]]),
        ])
    }
}

impl MagnetometerSensor for Iis2mdcDevice {
    fn read_magnetic_field(&mut self) -> Result<(f32, f32, f32)> {
        let axes = self.iis2mdc.read_axes()?;
        Ok(to_board_frame(axes))
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use std::{thread::sleep, time::Duration};

    #[test]
    fn board_frame_swaps_x_with_y() {
        let (x, y, z) = to_board_frame([200, -100, 300]);
        approx::assert_relative_eq!(x, 15.0);
        approx::assert_relative_eq!(y, -30.0);
        approx::assert_relative_eq!(z, 45.0);
    }

    #[ignore]
    #[test]
    fn test_iis2mdc_pi_4() {
        let mut mag = Iis2mdcDevice::builder()
            .build()
            .expect("Failed to build IIS2MDC");
        for _ in 0..10 {
            println!("IIS2MDC readings: {:?}", mag.read_magnetic_field());
            sleep(Duration::from_millis(100));
        }
    }
}
