use crate::peripherals::{
    AccelerometerSensor, AnyHardware, GyroscopeSensor, PeripheralClass, PeripheralInfo,
    Peripherals, TemperatureSensor,
};
use embedded_hal::i2c::I2c;
use linux_embedded_hal::{Delay, I2cdev};
use mpu6050::{Mpu6050, Mpu6050Error};
use std::error::Error;

pub struct Mpu6050Device {
    mpu: Mpu6050<I2cdev>,
    info: PeripheralInfo,
}

impl Mpu6050Device {
    pub fn builder() -> Mpu6050Builder {
        Mpu6050Builder::new()
    }

    pub fn get_peripheral_info(&self) -> &PeripheralInfo {
        &self.info
    }
}

impl AnyHardware for Mpu6050Device {
    fn as_gyroscope_sensor(&mut self) -> Option<&mut dyn GyroscopeSensor> {
        Some(self)
    }

    fn as_accelerometer_sensor(&mut self) -> Option<&mut dyn AccelerometerSensor> {
        Some(self)
    }

    fn as_temperature_sensor(&mut self) -> Option<&mut dyn TemperatureSensor> {
        Some(self)
    }
}

pub struct Mpu6050Builder {
    i2c_device: String,
    info: PeripheralInfo,
}

impl Mpu6050Builder {
    pub fn new() -> Self {
        Mpu6050Builder {
            i2c_device: "/dev/i2c-1".to_string(),
            info: PeripheralInfo {
                peripheral: Peripherals::Mpu6050,
                class: vec![PeripheralClass::Accelerometer, PeripheralClass::Gyroscope],
            },
        }
    }

    pub fn with_i2c_device(mut self, device: &str) -> Self {
        self.i2c_device = device.to_string();
        self
    }

    pub fn with_peripheral_info(mut self, info: PeripheralInfo) -> Self {
        self.info = info;
        self
    }

    pub fn build(self) -> Result<Mpu6050Device, Box<dyn Error>> {
        let i2c = I2cdev::new(&self.i2c_device)?;
        let mut mpu = Mpu6050::new(i2c);
        mpu.init(&mut Delay).unwrap();
        Ok(Mpu6050Device {
            mpu,
            info: self.info,
        })
    }
}

impl GyroscopeSensor for Mpu6050Device {
    fn read_angular_velocity(&mut self) -> Result<(f32, f32, f32), Box<dyn Error>> {
        let gyro = self.mpu.get_gyro().unwrap();
        Ok((gyro.x, gyro.y, gyro.z))
    }
}

impl AccelerometerSensor for Mpu6050Device {
    fn read_acceleration(&mut self) -> Result<(f32, f32, f32), Box<dyn Error>> {
        let acc = self.mpu.get_acc().unwrap();
        Ok((acc.x, acc.y, acc.z))
    }
}

impl TemperatureSensor for Mpu6050Device {
    fn read_temperature(&mut self) -> Result<f32, Box<dyn Error>> {
        let temp = self.mpu.get_temp().unwrap();
        Ok(temp)
    }
}

#[cfg(test)]
mod tests {
    use ftdi_embedded_hal as hal;
    use linux_embedded_hal::Delay;
    use mpu6050::Mpu6050;
    use std::error::Error;

    #[test]
    fn ftdi() -> Result<(), Box<dyn Error>> {
        let device = ftdi::find_by_vid_pid(0x0403, 0x6011)
            .interface(ftdi::Interface::A)
            .open()?;

        let hal = hal::FtHal::init_default(device)?;
        let i2c = hal.i2c().map_err(|e| Box::new(e) as Box<dyn Error>)?;

        let mut mpu = Mpu6050::new(i2c);
        mpu.init(&mut Delay).unwrap();

        // Necessary for GY87 board
        // i2c detect will work after this
        mpu.set_master_interrupt_enabled(false).unwrap();
        mpu.set_bypass_enabled(true).unwrap();
        mpu.set_sleep_enabled(false).unwrap();

        loop {
            let gyro = mpu.get_gyro().unwrap();
            let acc = mpu.get_acc().unwrap();
            println!(
                "Gyro: x={:.2} y={:.2} z={:.2}, Acc: x={:.2} y={:.2} z={:.2}",
                gyro.x, gyro.y, gyro.z, acc.x, acc.y, acc.z
            );
        }

        Ok(())
    }
}
