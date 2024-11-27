use std::error::Error;

use bmp280::{Bmp280, Bmp280Builder};

use crate::peripherals::{
    AnyHardware, BarometerSensor, PeripheralClass, PeripheralInfo, Peripherals, TemperatureSensor,
};

pub struct Bmp280Device {
    bmp280: Bmp280,
    info: PeripheralInfo,
}

impl Bmp280Device {
    pub fn builder() -> BarometerBuilder {
        BarometerBuilder::new()
    }

    pub fn get_peripheral_info(&self) -> &PeripheralInfo {
        &self.info
    }
}

impl AnyHardware for Bmp280Device {
    fn as_temperature_sensor(&mut self) -> Option<&mut dyn TemperatureSensor> {
        Some(self)
    }

    fn as_barometer_sensor(&mut self) -> Option<&mut dyn BarometerSensor> {
        Some(self)
    }
}

pub struct BarometerBuilder {
    i2c_bus: String,
    address: u8,
    info: PeripheralInfo,
}

impl BarometerBuilder {
    pub fn new() -> Self {
        BarometerBuilder {
            i2c_bus: "/dev/i2c-1".into(),
            address: 0x76,
            info: PeripheralInfo {
                peripheral: Peripherals::Bmp280,
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
        self.i2c_bus = bus.to_string();
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

    pub fn build(self) -> Result<Bmp280Device, Box<dyn Error>> {
        let mut bmp280 = Bmp280Builder::new()
            .path(&self.i2c_bus)
            .address(self.address as u16)
            .build()?;

        bmp280.zero()?;

        Ok(Bmp280Device {
            bmp280,
            info: self.info,
        })
    }
}

impl TemperatureSensor for Bmp280Device {
    fn read_temperature(&mut self) -> Result<f32, Box<dyn Error>> {
        let temperature = self.bmp280.temperature_celsius()?;
        Ok(temperature)
    }
}

impl BarometerSensor for Bmp280Device {
    fn read_pressure(&mut self) -> Result<f32, Box<dyn Error>> {
        let pressure = self.bmp280.pressure_kpa()?;
        Ok(pressure)
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use std::{thread::sleep, time::Duration};

    #[test]
    fn test_bmp280_pi_4() {
        let mut baro = Bmp280Device::builder()
            .build()
            .expect("Failed to build BMP280");
        for _ in 0..10 {
            println!("BMP280 temperature: {:?}", baro.read_temperature());
            println!("BMP280 pressure: {:?}", baro.read_pressure());
            sleep(Duration::from_millis(100));
        }
    }
}
