use std::error::Error;

use sk6812_rpi::{
    led::Led as StripLed,
    strip::{Bus, Strip},
};

use crate::peripherals::{AnyHardware, PeripheralClass, PeripheralInfo, Peripherals, RgbBehaviour};

pub struct RgbController {
    strip: Strip,
    info: PeripheralInfo,
}

impl RgbController {
    pub fn builder() -> RgbBuilder {
        RgbBuilder::new()
    }

    /// Gets information about the peripheral using the provided abstraction.
    pub fn get_peripheral_info(&self) -> &PeripheralInfo {
        &self.info
    }
}

pub struct RgbBuilder {
    bus: Bus,
    led_count: usize,
    info: PeripheralInfo,
}

impl RgbBuilder {
    pub fn new() -> Self {
        RgbBuilder {
            bus: Bus::Spi0,
            led_count: 1, // Default to 1 neopixel
            info: PeripheralInfo {
                peripheral: Peripherals::Sk6812,
                class: vec![PeripheralClass::RgbLed],
            },
        }
    }

    /// Sets the SPI bus to be used.
    ///
    /// # Arguments
    ///
    /// * `bus` - The SPI bus (`Bus::Spi0` or `Bus::Spi1`).
    pub fn with_bus(mut self, bus: Bus) -> Self {
        self.bus = bus;
        self
    }

    /// Sets the number of LEDs in the strip.
    ///
    /// # Arguments
    ///
    /// * `count` - The number of LEDs.
    pub fn with_led_count(mut self, count: usize) -> Self {
        self.led_count = count;
        self
    }

    pub fn with_peripheral_info(mut self, info: PeripheralInfo) -> Self {
        self.info = info;
        self
    }

    pub fn build(self) -> Result<RgbController, Box<dyn Error>> {
        let strip = Strip::new(self.bus, self.led_count)?;
        Ok(RgbController {
            strip,
            info: self.info,
        })
    }
}

impl AnyHardware for RgbController {
    fn as_rgb_behaviour(&mut self) -> Option<&mut dyn RgbBehaviour> {
        Some(self)
    }
}

impl RgbBehaviour for RgbController {
    fn set_colors(&mut self, colors: &[[u8; 3]]) -> Result<(), Box<dyn Error>> {
        for (index, color) in colors.iter().enumerate() {
            if index < self.strip.leds.len() {
                self.strip.leds[index] = StripLed::from_rgb(color[0], color[1], color[2]);
            } else {
                eprintln!("LED index {} out of bounds", index);
            }
        }
        self.strip.update()?;
        Ok(())
    }

    fn set_colors_rgbw(&mut self, colors: &[[u8; 4]]) -> Result<(), Box<dyn Error>> {
        for (index, color) in colors.iter().enumerate() {
            if index < self.strip.leds.len() {
                self.strip.leds[index] =
                    StripLed::from_rgbw(color[0], color[1], color[2], color[3]);
            } else {
                eprintln!("LED index {} out of bounds", index);
            }
        }
        self.strip.update()?;
        Ok(())
    }

    fn clear(&mut self) -> Result<(), Box<dyn Error>> {
        self.strip.clear();
        self.strip.update()?;
        Ok(())
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use std::{thread::sleep, time::Duration};

    #[test]
    fn test_rgb_pi_4() {
        let mut rgb = RgbController::builder().build().unwrap();
        for _ in 0..10 {
            for color in [[255_u8, 0, 0], [0, 255, 0], [0, 0, 255]] {
                println!("Rgb changing color: {:?}", color);
                rgb.set_colors(&[color]).unwrap();
                sleep(Duration::from_millis(300));
            }
        }
    }
}
