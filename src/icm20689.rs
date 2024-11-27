use std::error::Error;

use icm20689::{self, Builder as ImuBuilder, SpiInterface, ICM20689};
use linux_embedded_hal::spidev::{SpiModeFlags, SpidevOptions};
use linux_embedded_hal::sysfs_gpio::Direction;
use linux_embedded_hal::{Delay, Pin, Spidev};

use crate::peripherals::{
    AccelerometerSensor, AnyHardware, GyroscopeSensor, PeripheralClass, PeripheralInfo, Peripherals,
};

pub struct Icm20689Device {
    imu: ICM20689<SpiInterface<Spidev, Pin>>,
    info: PeripheralInfo,
}

impl Icm20689Device {
    pub fn builder() -> Icm20689Builder {
        Icm20689Builder::new()
    }

    pub fn get_peripheral_info(&self) -> &PeripheralInfo {
        &self.info
    }
}

impl AnyHardware for Icm20689Device {
    fn as_gyroscope_sensor(&mut self) -> Option<&mut dyn GyroscopeSensor> {
        Some(self)
    }

    fn as_accelerometer_sensor(&mut self) -> Option<&mut dyn AccelerometerSensor> {
        Some(self)
    }
}

pub struct Icm20689Builder {
    spi_device: String,
    cs_pin_number: u64,
    info: PeripheralInfo,
}

impl Icm20689Builder {
    pub fn new() -> Self {
        Icm20689Builder {
            spi_device: "/dev/spidev1.0".to_string(),
            cs_pin_number: 16,
            info: PeripheralInfo {
                peripheral: Peripherals::Icm20689,
                class: vec![PeripheralClass::Accelerometer, PeripheralClass::Gyroscope],
            },
        }
    }

    /// Sets the SPI device to be used.
    ///
    /// # Arguments
    ///
    /// * `device` - The SPI device (e.g., "/dev/spidev1.0").
    pub fn with_spi_device(mut self, device: &str) -> Self {
        self.spi_device = device.to_string();
        self
    }

    /// Sets the Chip Select (CS) pin number.
    ///
    /// # Arguments
    ///
    /// * `pin_number` - The SPI pin number (e.g., 16).
    pub fn with_cs_pin(mut self, pin_number: u64) -> Self {
        self.cs_pin_number = pin_number;
        self
    }

    pub fn with_peripheral_info(mut self, info: PeripheralInfo) -> Self {
        self.info = info;
        self
    }

    pub fn build(self) -> Result<Icm20689Device, Box<dyn Error>> {
        let mut spi = Spidev::open(self.spi_device)?;
        let options = SpidevOptions::new()
            .bits_per_word(8)
            .max_speed_hz(10_000_000)
            .mode(SpiModeFlags::SPI_MODE_0)
            .build();
        spi.configure(&options)?;

        let cs_pin = Pin::new(self.cs_pin_number);
        cs_pin.export()?;
        std::thread::sleep(std::time::Duration::from_millis(30));
        cs_pin.set_direction(Direction::Out)?;
        cs_pin.set_value(1)?;

        let mut imu = ImuBuilder::new_spi(spi, cs_pin);

        imu.setup(&mut Delay {}).unwrap();

        Ok(Icm20689Device {
            imu,
            info: self.info,
        })
    }
}

impl GyroscopeSensor for Icm20689Device {
    fn read_angular_velocity(&mut self) -> Result<(f32, f32, f32), Box<dyn Error>> {
        let gyro_data = self.imu.get_scaled_gyro().unwrap();
        Ok((gyro_data[0], gyro_data[1], gyro_data[2]))
    }
}
impl AccelerometerSensor for Icm20689Device {
    fn read_acceleration(&mut self) -> Result<(f32, f32, f32), Box<dyn Error>> {
        let accel_data = self.imu.get_scaled_accel().unwrap();
        Ok((accel_data[0], accel_data[1], accel_data[2]))
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use std::{thread::sleep, time::Duration};

    #[test]
    fn test_icm20689_pi_4() {
        let mut imu = Icm20689Device::builder()
            .build()
            .expect("Failed to build ICM20689");
        for _ in 0..10 {
            println!("ICM20689 gyroscope: {:?}", imu.read_angular_velocity());
            println!("ICM20689 accelerometers: {:?}", imu.read_acceleration());
            sleep(Duration::from_millis(100));
        }
    }
}
