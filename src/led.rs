use std::{path::PathBuf, thread::sleep, time::Duration};

use linux_embedded_hal::gpio_cdev::{Chip, LineHandle, LineRequestFlags};

use crate::peripherals::{AnyHardware, LedBehaviour, PeripheralClass, PeripheralInfo, Peripherals};

pub struct LedController {
    pub leds: Vec<LineHandle>,
    pub info: PeripheralInfo,
}

impl LedController {
    pub fn builder() -> LedControllerBuilder {
        LedControllerBuilder::new()
    }

    pub fn get_peripheral_info(&self) -> &PeripheralInfo {
        &self.info
    }
}

impl AnyHardware for LedController {
    fn as_led_behaviour(&mut self) -> Option<&mut dyn LedBehaviour> {
        Some(self)
    }
}

pub struct LedControllerBuilder {
    pub pins: Vec<u32>,
    pub info: PeripheralInfo,
    pub gpiochip: PathBuf,
}

impl LedControllerBuilder {
    pub fn new() -> Self {
        LedControllerBuilder {
            pins: Vec::new(),
            info: PeripheralInfo {
                peripheral: Peripherals::Gpio,
                class: vec![PeripheralClass::Led],
            },
            gpiochip: "/dev/gpiochip0".into(),
        }
    }

    /// Adds a GPIO pin to control an LED.
    pub fn add_led_pin(mut self, pin_number: u32) -> Self {
        self.pins.push(pin_number);
        self
    }

    pub fn with_peripheral_info(mut self, info: PeripheralInfo) -> Self {
        self.info = info;
        self
    }

    pub fn with_gpiochip(mut self, gpiochip: &str) -> Self {
        self.gpiochip = gpiochip.into();
        self
    }

    /// Configures the GPIO pins for the default Navigator board.
    pub fn configure_navigator(self) -> Self {
        self.add_led_pin(11).add_led_pin(24).add_led_pin(25)
    }

    /// Builds the `LedController`.
    pub fn build(mut self) -> LedController {
        let mut leds = Vec::new();

        if self.pins.is_empty() {
            self = self.configure_navigator();
        }

        let mut chip = Chip::new(self.gpiochip).expect("Failed to open GPIO chip");
        for &pin_number in &self.pins {
            let pin = chip
                .get_line(pin_number)
                .unwrap()
                .request(
                    LineRequestFlags::OUTPUT,
                    1,
                    &format!("user-led-{pin_number}"),
                )
                .expect("Failed to request LED pin");
            sleep(Duration::from_millis(30));
            pin.set_value(1).expect("Failed to set initial LED value");
            leds.push(pin);
        }

        LedController {
            leds,
            info: self.info,
        }
    }
}

impl LedBehaviour for LedController {
    fn set_led(&mut self, index: usize, state: bool) {
        if let Some(led) = self.leds.get_mut(index) {
            led.set_value((!state).into())
                .expect("Failed to set LED value");
        } else {
            eprintln!("Invalid LED index: {}", index);
        }
    }

    fn get_led(&self, index: usize) -> bool {
        if let Some(led) = self.leds.get(index) {
            led.get_value()
                .unwrap_or_else(|_| panic!("Error: Get {} LED value", index))
                == 0
        } else {
            eprintln!("Invalid LED index: {}", index);
            false
        }
    }

    fn toggle_led(&mut self, index: usize) {
        self.set_led(index, !self.get_led(index));
    }

    fn number_of_leds(&self) -> usize {
        self.leds.len()
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use std::{thread::sleep, time::Duration};

    #[test]
    fn test_user_led_pi_4() {
        let mut led = LedController::builder().build();
        for _ in 0..10 {
            println!("User LED: Toggling..");
            for index in 0..led.number_of_leds() {
                led.toggle_led(index);
            }
            sleep(Duration::from_millis(1000));
        }
    }
}
