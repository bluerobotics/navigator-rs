use std::error::Error;

use ads1x1x::{
    ic::{Ads1115, Resolution16Bit},
    Ads1x1x, TargetAddr,
};
use linux_embedded_hal::I2cdev;

use crate::peripherals::{AdcSensor, AnyHardware, PeripheralClass, PeripheralInfo, Peripherals};

pub struct Ads1115Device {
    adc: Ads1x1x<I2cdev, Ads1115, Resolution16Bit, ads1x1x::mode::OneShot>,
    info: PeripheralInfo,
}

impl Ads1115Device {
    pub fn builder() -> Ads1115DeviceBuilder {
        Ads1115DeviceBuilder::new()
    }

    pub fn get_peripheral_info(&self) -> &PeripheralInfo {
        &self.info
    }
}

impl AnyHardware for Ads1115Device {
    fn as_adc_sensor(&mut self) -> Option<&mut dyn AdcSensor> {
        Some(self)
    }
}

pub struct Ads1115DeviceBuilder {
    i2c_bus: String,
    address: TargetAddr,
    info: PeripheralInfo,
}

impl Ads1115DeviceBuilder {
    pub fn new() -> Self {
        Ads1115DeviceBuilder {
            i2c_bus: "/dev/i2c-1".into(),
            address: TargetAddr::default(),
            info: PeripheralInfo {
                peripheral: Peripherals::Ads1115,
                class: vec![PeripheralClass::Adc],
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
    /// * `address` - The I²C address (e.g., 0x48).
    pub fn with_address(mut self, address: TargetAddr) -> Self {
        self.address = address;
        self
    }

    pub fn with_peripheral_info(mut self, info: PeripheralInfo) -> Self {
        self.info = info;
        self
    }

    pub fn build(self) -> Result<Ads1115Device, Box<dyn Error>> {
        let i2c = I2cdev::new(self.i2c_bus)?;
        let mut adc = Ads1x1x::new_ads1115(i2c, self.address);

        adc.set_full_scale_range(ads1x1x::FullScaleRange::Within4_096V)
            .unwrap();
        adc.set_data_rate(ads1x1x::DataRate16Bit::Sps860).unwrap();

        Ok(Ads1115Device {
            adc,
            info: self.info,
        })
    }
}

impl AdcSensor for Ads1115Device {
    fn read_channel(&mut self, channel: usize) -> Result<f32, Box<dyn Error>> {
        // Keep trying to read until the conversion is complete
        loop {
            let channel_value = match channel {
                0 => self.adc.read(ads1x1x::channel::SingleA0),
                1 => self.adc.read(ads1x1x::channel::SingleA1),
                2 => self.adc.read(ads1x1x::channel::SingleA2),
                3 => self.adc.read(ads1x1x::channel::SingleA3),
                _ => return Err("Invalid ADC channel".into()),
            };

            match channel_value {
                Ok(raw_value) => {
                    // TODO: Hold configuration over having it hardcoded
                    let gain = ads1x1x::FullScaleRange::Within4_096V;

                    // Datasheet Table 3: https://www.ti.com/lit/ds/symlink/ads1115.pdf
                    let voltage = match gain {
                        ads1x1x::FullScaleRange::Within6_144V => raw_value as f32 * 0.0001875,
                        ads1x1x::FullScaleRange::Within4_096V => raw_value as f32 * 0.000125,
                        ads1x1x::FullScaleRange::Within2_048V => raw_value as f32 * 0.0000625,
                        ads1x1x::FullScaleRange::Within1_024V => raw_value as f32 * 0.00003125,
                        ads1x1x::FullScaleRange::Within0_512V => raw_value as f32 * 0.000015625,
                        ads1x1x::FullScaleRange::Within0_256V => raw_value as f32 * 0.0000078125,
                    };

                    return Ok(voltage);
                }
                Err(nb::Error::WouldBlock) => {
                    // TODO: configure this
                    std::thread::sleep(std::time::Duration::from_micros(10));
                    continue;
                }
                Err(nb::Error::Other(error)) => {
                    return Err(format!("Failed to read: {error:?}").into())
                }
            }
        }
    }

    fn read_all_channels(&mut self) -> Result<Vec<f32>, Box<dyn Error>> {
        Ok((0..4_usize)
            .map(|index| self.read_channel(index).unwrap())
            .collect())
    }

    fn channels(self) -> u32 {
        4
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use std::{thread::sleep, time::Duration};

    #[test]
    fn test_ads1115_pi_4() {
        let mut adc = Ads1115Device::builder().build().unwrap();
        println!("ADS1115 channel 0: {:?}", adc.read_channel(1));
        for _ in 0..10 {
            println!("ADS1115 readings: {:?}", adc.read_all_channels());
            sleep(Duration::from_millis(100));
        }
    }
}
