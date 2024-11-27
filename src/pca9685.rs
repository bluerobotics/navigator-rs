use std::{error::Error, thread::sleep, time::Duration};

use linux_embedded_hal::{
    gpio_cdev::{Chip, LineHandle, LineRequestFlags},
    I2cdev,
};
use pwm_pca9685::{Address as PwmAddress, Channel, Pca9685};

use crate::peripherals::{AnyHardware, PeripheralClass, PeripheralInfo, Peripherals, PwmBehaviour};

pub struct Pca9685Device {
    pwm: Pca9685<I2cdev>,
    oe_pin: LineHandle,
    info: PeripheralInfo,
}

impl Pca9685Device {
    pub fn builder() -> Pca9685DeviceBuilder {
        Pca9685DeviceBuilder::new()
    }

    pub fn get_peripheral_info(&self) -> &PeripheralInfo {
        &self.info
    }
}

impl AnyHardware for Pca9685Device {
    fn as_pwm_behaviour(&mut self) -> Option<&mut dyn PwmBehaviour> {
        Some(self)
    }
}

pub struct Pca9685DeviceBuilder {
    i2c_bus: String,
    address: PwmAddress,
    oe_pin_number: u32,
    info: PeripheralInfo,
}

impl Pca9685DeviceBuilder {
    pub fn new() -> Self {
        Pca9685DeviceBuilder {
            i2c_bus: "/dev/i2c-4".into(),
            address: PwmAddress::default(),
            oe_pin_number: 26,
            info: PeripheralInfo {
                peripheral: Peripherals::Pca9685,
                class: vec![PeripheralClass::Pwm],
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

    /// Sets the I²C address of the PWM device.
    ///
    /// # Arguments
    ///
    /// * `address` - The I²C address (default is 0x40).
    pub fn with_address(mut self, address: u8) -> Self {
        self.address = PwmAddress::from(address);
        self
    }

    /// Sets the OE (Output Enable) pin number.
    ///
    /// # Arguments
    ///
    /// * `pin_number` - The GPIO pin number for OE.
    pub fn with_oe_pin(mut self, pin_number: u32) -> Self {
        self.oe_pin_number = pin_number;
        self
    }

    /// Sets the `PeripheralInfo` for the PWM controller.
    pub fn with_peripheral_info(mut self, info: PeripheralInfo) -> Self {
        self.info = info;
        self
    }

    /// Builds the `Pca9685Device`.
    pub fn build(self) -> Result<Pca9685Device, Box<dyn Error>> {
        let device = I2cdev::new(self.i2c_bus)?;
        let mut pwm = Pca9685::new(device, self.address).expect("Failed to open PWM controller");

        let oe_pin = {
            let mut chip = Chip::new("/dev/gpiochip0")?;
            let pin = chip
                .get_line(self.oe_pin_number)
                .unwrap()
                .request(LineRequestFlags::OUTPUT, 1, "oe-pin-pca9685")
                .expect("Failed to request OE pin");
            sleep(Duration::from_millis(30));
            pin.set_value(1)?;
            pin
        };

        pwm.reset_internal_driver_state();
        pwm.use_external_clock().unwrap();
        pwm.enable().unwrap();

        Ok(Pca9685Device {
            pwm,
            oe_pin,
            info: self.info,
        })
    }
}

impl PwmBehaviour for Pca9685Device {
    fn enable_output(&mut self, enable: bool) -> Result<(), Box<dyn Error>> {
        // Active low OE pin
        let value = if enable { 0 } else { 1 };
        self.oe_pin.set_value(value).unwrap();
        Ok(())
    }

    fn set_frequency(&mut self, freq_hz: f32) -> Result<(), Box<dyn Error>> {
        const EXTERNAL_CLOCK: f32 = 24_576_000.0;
        let prescale: u8 = (EXTERNAL_CLOCK / (4096.0 * freq_hz)).round() as u8 - 1;
        if prescale < 3
        /*|| prescale > 255*/
        {
            eprintln!("Invalid prescale value: {freq_hz}");
            return Ok(());
        }
        self.pwm.set_prescale(prescale).unwrap();
        Ok(())
    }

    fn set_duty_cycle(&mut self, channel: usize, duty_cycle: f32) -> Result<(), Box<dyn Error>> {
        if channel > 15 {
            eprintln!("Invalid channel: {channel}");
            return Ok(());
        }
        let duty_cycle = duty_cycle.clamp(0.0, 1.0);
        let channel = Channel::try_from(channel).unwrap();

        if approx::relative_eq!(duty_cycle, 1.0) {
            self.pwm.set_channel_full_on(channel, 0).unwrap();
            return Ok(());
        }

        const MAX_VALUE: u16 = 4095;
        let duty_cycle = duty_cycle.clamp(0.0, 1.0);
        let duty_cycle = (duty_cycle * MAX_VALUE as f32) as u16;
        self.pwm.set_channel_on_off(channel, 0, duty_cycle).unwrap();
        Ok(())
    }

    fn set_duty_cycle_all(&mut self, duty_cycle: f32) -> Result<(), Box<dyn Error>> {
        let duty_cycle = duty_cycle.clamp(0.0, 1.0);

        const MAX_VALUE: u16 = 4095;
        let duty_cycle = duty_cycle.clamp(0.0, 1.0);
        let duty_cycle = (duty_cycle * MAX_VALUE as f32) as u16;

        self.pwm
            .set_all_on_off(&[0; 16], &[duty_cycle; 16])
            .unwrap();
        Ok(())
    }

    fn set_duty_cycle_multiple(
        &mut self,
        channels: &[usize],
        duty_cycle: f32,
    ) -> Result<(), Box<dyn Error>> {
        for &channel in channels {
            self.set_duty_cycle(channel, duty_cycle)?;
        }
        Ok(())
    }
}

impl Drop for Pca9685Device {
    fn drop(&mut self) {
        let _ = self.enable_output(false);
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use std::{thread::sleep, time::Duration};

    //TODO: Not working
    #[test]
    fn test_pca9685_pi_4() {
        let mut pwm = Pca9685Device::builder()
            .build()
            .expect("Error building PCA9685");

        println!("PCA9685: Start initial configuration");

        pwm.enable_output(false)
            .expect("Error in while enabling output");
        pwm.set_frequency(60.0).expect("Error in setting frequency");
        pwm.set_duty_cycle_all(0.5)
            .expect("Error in configuring duty cycle");

        println!("PCA9685: Enabling PWM output");
        pwm.enable_output(true).expect("Error in enabling output");
        sleep(Duration::from_millis(1000));
        for duty_cycle in [0.0, 1.0, 0.0, 1.0, 0.5] {
            for channel in 0..16 {
                println!("PCA9685: Channel {channel} value {duty_cycle:.1}");
                pwm.set_duty_cycle(channel, duty_cycle)
                    .expect("Error in setting duty cycle");
                sleep(Duration::from_millis(100));
            }
        }
    }
}
