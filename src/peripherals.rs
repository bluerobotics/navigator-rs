use std::error::Error;

pub enum Peripherals {
    Ads1115, // ADC
    Ak09915, // 3-axis magnetometer
    Bmp280,  // Pressure and Temperature
    Bmp390,  // Pressure and Temperature
    Gpio,
    Icm20689, // Accelerometer and Gyroscope
    Leak,
    Pca9685, // PWM controller
    Sk6812,  // Neopixel RGB LED
}

pub enum PeripheralClass {
    Accelerometer,
    Adc,
    DigitalInput,
    DigitalOutput,
    Gyroscope,
    Led,
    Magnetometer,
    Pressure,
    Pwm,
    RgbLed,
    Temperature,
}

pub struct PeripheralInfo {
    pub peripheral: Peripherals,
    pub class: Vec<PeripheralClass>,
}

pub trait AnyHardwareExt {
    fn as_any(&self) -> &dyn std::any::Any;
    fn as_any_mut(&mut self) -> &mut dyn std::any::Any;
}

impl<T: AnyHardware + 'static> AnyHardwareExt for T {
    fn as_any(&self) -> &dyn std::any::Any {
        self
    }
    fn as_any_mut(&mut self) -> &mut dyn std::any::Any {
        self
    }
}

pub trait AnyHardware: AnyHardwareExt + Send {
    fn as_adc_sensor(&mut self) -> Option<&mut dyn AdcSensor> {
        None
    }
    fn as_temperature_sensor(&mut self) -> Option<&mut dyn TemperatureSensor> {
        None
    }
    fn as_magnetometer_sensor(&mut self) -> Option<&mut dyn MagnetometerSensor> {
        None
    }
    fn as_gyroscope_sensor(&mut self) -> Option<&mut dyn GyroscopeSensor> {
        None
    }
    fn as_accelerometer_sensor(&mut self) -> Option<&mut dyn AccelerometerSensor> {
        None
    }
    fn as_barometer_sensor(&mut self) -> Option<&mut dyn BarometerSensor> {
        None
    }
    fn as_leak_sensor(&mut self) -> Option<&mut dyn LeakSensor> {
        None
    }
    fn as_led_behaviour(&mut self) -> Option<&mut dyn LedBehaviour> {
        None
    }
    fn as_rgb_behaviour(&mut self) -> Option<&mut dyn RgbBehaviour> {
        None
    }
    fn as_pwm_behaviour(&mut self) -> Option<&mut dyn PwmBehaviour> {
        None
    }
}

/// Trait for analog sensors
pub trait AdcSensor: AnyHardware {
    /// Reads the voltage of the channel
    fn read_channel(&mut self, channel: usize) -> Result<f32, Box<dyn Error>>;

    /// Reads all channels and returns their voltages
    fn read_all_channels(&mut self) -> Result<Vec<f32>, Box<dyn Error>>;

    /// Returns the total number of channels
    fn channels(self) -> u32;
}

/// Trait for temperature sensors
pub trait TemperatureSensor: AnyHardware {
    /// Reads the temperature in degrees Celsius.
    fn read_temperature(&mut self) -> Result<f32, Box<dyn Error>>;
}

/// Trait for magnetometers
pub trait MagnetometerSensor: AnyHardware {
    /// Reads the magnetic field data.
    ///
    /// # Returns
    ///
    /// A tuple `(x, y, z)` representing the magnetic field in microteslas (µT).
    fn read_magnetic_field(&mut self) -> Result<(f32, f32, f32), Box<dyn Error>>;
}

/// Trait for gyroscopes
pub trait GyroscopeSensor: AnyHardware {
    /// Reads the angular velocity data.
    ///
    /// # Returns
    ///
    /// A tuple `(x, y, z)` representing the angular velocity in radians per second (rad/s).
    fn read_angular_velocity(&mut self) -> Result<(f32, f32, f32), Box<dyn Error>>;
}

/// Trait for accelerometers
pub trait AccelerometerSensor: AnyHardware {
    /// Reads the acceleration data.
    ///
    /// # Returns
    ///
    /// A tuple `(x, y, z)` representing the acceleration in meters per second squared (m/s²).
    fn read_acceleration(&mut self) -> Result<(f32, f32, f32), Box<dyn Error>>;
}

/// Trait for barometers
pub trait BarometerSensor: AnyHardware {
    /// Reads the atmospheric pressure.
    ///
    /// # Returns
    ///
    /// The atmospheric pressure in kilopascals (kPa).
    fn read_pressure(&mut self) -> Result<f32, Box<dyn Error>>;
}

/// Trait for leak detection sensors
pub trait LeakSensor: AnyHardware {
    /// Checks if a leak is detected.
    ///
    /// # Returns
    ///
    /// * `true` if a leak is detected, `false` otherwise.
    fn is_leak_detected(&self) -> Result<bool, Box<dyn Error>>;
}

/// Trait for LED controllers
pub trait LedBehaviour: AnyHardware {
    /// Sets the LED state
    fn set_led(&mut self, index: usize, state: bool);

    /// Gets the LED state
    fn get_led(&self, index: usize) -> bool;

    /// Toggles the LED state
    fn toggle_led(&mut self, index: usize);

    /// Returns the total number of LEDs that can be controlled
    fn number_of_leds(&self) -> usize;
}

/// Trait for RGB LEDs
pub trait RgbBehaviour: AnyHardware {
    /// Sets the RGB values for the LEDs in the strip.
    fn set_colors(&mut self, colors: &[[u8; 3]]) -> Result<(), Box<dyn Error>>;

    /// Sets the RGBW values for the LEDs in the strip.
    fn set_colors_rgbw(&mut self, colors: &[[u8; 4]]) -> Result<(), Box<dyn Error>>;

    /// Clears the LED strip (turns off all LEDs).
    fn clear(&mut self) -> Result<(), Box<dyn Error>>;
}

/// Trait for PWM control
pub trait PwmBehaviour: AnyHardware {
    /// Enables or disables output.
    fn enable_output(&mut self, enable: bool) -> Result<(), Box<dyn Error>>;

    /// Sets the PWM frequency in Hertz.
    fn set_frequency(&mut self, freq_hz: f32) -> Result<(), Box<dyn Error>>;

    /// Sets the duty cycle for a specific channel.
    fn set_duty_cycle(&mut self, channel: usize, duty_cycle: f32) -> Result<(), Box<dyn Error>>;

    /// Sets the duty cycle for all channels.
    fn set_duty_cycle_all(&mut self, duty_cycle: f32) -> Result<(), Box<dyn Error>>;

    /// Sets the duty cycle for multiple channels.
    fn set_duty_cycle_multiple(
        &mut self,
        channels: &[usize],
        duty_cycle: f32,
    ) -> Result<(), Box<dyn Error>>;
}
