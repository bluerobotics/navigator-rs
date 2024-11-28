use std::{error::Error, thread::sleep, time::Duration};

use linux_embedded_hal::sysfs_gpio::{Direction, Pin};

use crate::peripherals::{AnyHardware, LeakSensor, PeripheralClass, PeripheralInfo, Peripherals};

pub struct LeakDetector {
    pin: Pin,
    info: PeripheralInfo,
}

impl LeakDetector {
    pub fn builder() -> LeakBuilder {
        LeakBuilder::new()
    }

    pub fn get_peripheral_info(&self) -> &PeripheralInfo {
        &self.info
    }
}

impl AnyHardware for LeakDetector {
    fn as_leak_sensor(&mut self) -> Option<&mut dyn LeakSensor> {
        Some(self)
    }
}

pub struct LeakBuilder {
    pin_number: u64,
    info: PeripheralInfo,
}

impl LeakBuilder {
    pub fn new() -> Self {
        LeakBuilder {
            pin_number: 27,
            info: PeripheralInfo {
                peripheral: Peripherals::Leak,
                class: vec![PeripheralClass::DigitalInput],
            },
        }
    }

    /// Sets the GPIO pin number for the leak sensor.
    ///
    /// # Arguments
    ///
    /// * `pin_number` - The GPIO pin number.
    pub fn with_pin(mut self, pin_number: u64) -> Self {
        self.pin_number = pin_number;
        self
    }

    pub fn with_peripheral_info(mut self, info: PeripheralInfo) -> Self {
        self.info = info;
        self
    }

    pub fn build(self) -> Result<LeakDetector, Box<dyn Error>> {
        let pin = Pin::new(self.pin_number);
        pin.export()?;
        sleep(Duration::from_millis(60));
        pin.set_direction(Direction::In)?;
        Ok(LeakDetector {
            pin,
            info: self.info,
        })
    }
}

impl LeakSensor for LeakDetector {
    fn is_leak_detected(&self) -> Result<bool, Box<dyn Error>> {
        let value = self.pin.get_value()?;
        Ok(value == 1)
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use std::{thread::sleep, time::Duration};

    #[test]
    fn test_leak_pi_4() {
        let leak = LeakDetector::builder()
            .build()
            .expect("Failed to build leak detector");
        for _ in 0..10 {
            println!("Leak detector: {}", leak.is_leak_detected().unwrap());
            sleep(Duration::from_millis(1000));
        }
    }
}
