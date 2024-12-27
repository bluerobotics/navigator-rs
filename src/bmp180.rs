use std::error::Error;

//use bmp180_embedded_hal::blocking::{UninitBMP180, BMP180}
use bmp180_driver::{Common, InitializedBMP180, Resolution, BMP180};
use linux_embedded_hal::{Delay, I2cdev};

use crate::peripherals::{
    AnyHardware, BarometerSensor, PeripheralClass, PeripheralInfo, Peripherals, TemperatureSensor,
};

pub struct Bmp180Device {
    bmp180: InitializedBMP180<I2cdev, Delay>,
    info: PeripheralInfo,
}

impl Bmp180Device {
    pub fn builder() -> BarometerBuilder {
        BarometerBuilder::new()
    }

    pub fn get_peripheral_info(&self) -> &PeripheralInfo {
        &self.info
    }
}

impl AnyHardware for Bmp180Device {
    fn as_temperature_sensor(&mut self) -> Option<&mut dyn TemperatureSensor> {
        Some(self)
    }

    fn as_barometer_sensor(&mut self) -> Option<&mut dyn BarometerSensor> {
        Some(self)
    }
}

pub struct BarometerBuilder {
    i2c_bus: Result<I2cdev, Box<dyn Error>>,
    address: u8,
    info: PeripheralInfo,
}

impl BarometerBuilder {
    pub fn new() -> Self {
        BarometerBuilder {
            i2c_bus: I2cdev::new("/dev/i2c-1").map_err(|e| Box::new(e) as Box<dyn Error>),
            address: 0x76,
            info: PeripheralInfo {
                peripheral: Peripherals::Bmp180,
                class: vec![PeripheralClass::Pressure, PeripheralClass::Temperature],
            },
        }
    }

    /// Sets the I²C bus to be used.
    ///
    /// # Arguments
    ///
    /// * `bus` - The I²C bus (e.g., "/dev/i2c-1").
    pub fn with_i2c_bus(mut self, bus: &str) -> Self {
        self.i2c_bus = I2cdev::new(bus).map_err(|e| Box::new(e) as Box<dyn Error>);
        self
    }

    pub fn with_i2c(mut self, i2c: Result<I2cdev, Box<dyn Error>>) -> Self {
        self.i2c_bus = i2c;
        self
    }

    /// Sets the I²C address
    ///
    /// # Arguments
    ///
    /// * `address` - The I²C address (e.g., 0x76).
    pub fn with_address(mut self, address: u8) -> Self {
        self.address = address;
        self
    }

    pub fn with_peripheral_info(mut self, info: PeripheralInfo) -> Self {
        self.info = info;
        self
    }

    pub fn build(self) -> Result<Bmp180Device, Box<dyn Error>> {
        let i2c = self.i2c_bus?;
        let delay = Delay {};

        let mut bmp180 = BMP180::new(i2c, delay);
        bmp180.check_connection()?;
        let bmp180 = bmp180.initialize()?;

        Ok(Bmp180Device {
            bmp180,
            info: self.info,
        })
    }
}

impl TemperatureSensor for Bmp180Device {
    fn read_temperature(&mut self) -> Result<f32, Box<dyn Error>> {
        let temperature = self.bmp180.temperature()?;
        Ok(temperature / 10.0)
    }
}

impl BarometerSensor for Bmp180Device {
    fn read_pressure(&mut self) -> Result<f32, Box<dyn Error>> {
        let pressure = self.bmp180.pressure(Resolution::Standard)? as f32;
        Ok(pressure)
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use ftdi_embedded_hal as hal;

    #[test]
    fn ftdi() {
        let device = ftdi::find_by_vid_pid(0x0403, 0x6011)
            .interface(ftdi::Interface::A)
            .open()
            .unwrap();

        let hal = hal::FtHal::init_default(device).unwrap();
        let mut i2c = hal
            .i2c()
            .map_err(|e| Box::new(e) as Box<dyn Error>)
            .unwrap();

        let mut bmp180 = BMP180::new(i2c, Delay {});
        bmp180.check_connection().unwrap();
        let mut bmp180 = bmp180.initialize().unwrap();

        loop {
            let temperature = bmp180.temperature().unwrap();
            let pressure = bmp180.pressure(Resolution::Standard).unwrap() as f32;
            println!(
                "Temperature: {:.2}°C, Pressure: {:.2}hPa",
                temperature, pressure
            );
        }
    }
}
