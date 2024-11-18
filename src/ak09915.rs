use std::error::Error;

use ak09915_rs::{Ak09915 as Ak, Mode as MagMode};
use linux_embedded_hal::I2cdev;

use crate::peripherals::{
    AnyHardware, MagnetometerSensor, PeripheralClass, PeripheralInfo, Peripherals,
};

pub struct Ak09915Device {
    mag: Ak<I2cdev>,
    info: PeripheralInfo,
}

impl Ak09915Device {
    pub fn builder() -> Ak09915Builder {
        Ak09915Builder::new()
    }

    pub fn get_peripheral_info(&self) -> &PeripheralInfo {
        &self.info
    }
}

impl AnyHardware for Ak09915Device {
    fn as_magnetometer_sensor(&mut self) -> Option<&mut dyn MagnetometerSensor> {
        Some(self)
    }
}

pub struct Ak09915Builder {
    i2c_bus: String,
    info: PeripheralInfo,
}

impl Ak09915Builder {
    pub fn new() -> Self {
        Ak09915Builder {
            i2c_bus: "/dev/i2c-1".into(),
            info: PeripheralInfo {
                peripheral: Peripherals::Ak09915,
                class: vec![PeripheralClass::Magnetometer],
            },
        }
    }

    /// Sets the I²C bus to be used.
    ///
    /// # Arguments
    ///
    /// * `bus` - The I²C bus (e.g., "/dev/i2c-1").
    pub fn with_i2c_bus(mut self, bus: &str) -> Self {
        self.i2c_bus = bus.to_string();
        self
    }

    pub fn with_peripheral_info(mut self, info: PeripheralInfo) -> Self {
        self.info = info;
        self
    }

    pub fn build(self) -> Result<Ak09915Device, Box<dyn Error>> {
        let i2c = I2cdev::new(self.i2c_bus)?;

        let mut mag = Ak::new(i2c);

        mag.init().unwrap();
        mag.set_mode(MagMode::Cont200Hz).unwrap();

        Ok(Ak09915Device {
            mag,
            info: self.info,
        })
    }
}

impl MagnetometerSensor for Ak09915Device {
    fn read_magnetic_field(&mut self) -> Result<(f32, f32, f32), Box<dyn Error>> {
        Ok(self.mag.read().unwrap())
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use std::{thread::sleep, time::Duration};

    #[test]
    fn test_ak09915_pi_4() {
        let mut mag = Ak09915Device::builder().build().unwrap();
        for _ in 0..10 {
            println!("AK09915 readings: {:?}", mag.read_magnetic_field());
            sleep(Duration::from_millis(100));
        }
    }
}
