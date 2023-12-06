#![deny(unsafe_code)]
#![doc(html_logo_url = "https://upload.wikimedia.org/wikipedia/commons/1/12/Bluerobotics-logo.svg")]
#![doc = include_str!("../README.md")]

use ads1x1x::{
    ic::{Ads1115, Resolution16Bit},
    interface::I2cInterface,
    Ads1x1x, DynamicOneShot, SlaveAddr as adc_Address,
};
use ak09915_rs::{Ak09915, Mode as mag_Mode};
use bmp280::{Bmp280, Bmp280Builder};
use embedded_hal::prelude::_embedded_hal_blocking_delay_DelayMs;
use icm20689::{self, AccelRange, Builder as imu_Builder, GyroRange, SpiInterface, ICM20689};
use linux_embedded_hal::spidev::{self, SpidevOptions};
use linux_embedded_hal::sysfs_gpio::Direction;
use linux_embedded_hal::I2cdev;
use linux_embedded_hal::{Delay, Pin, Spidev};
use log::{info, warn};
use nb::block;
use pwm_pca9685::{Address as pwm_Address, Pca9685};
use sk6812_rpi::led::Led as StripLed;
use sk6812_rpi::strip::{Bus, Strip};
use std::{
    fmt,
    ops::{Deref, DerefMut},
};

/// Navigator's default crystal clock for PWM, with a value of 24.5760 MHz
const NAVIGATOR_PWM_XTAL_CLOCK_FREQ: f32 = 24_576_000.0;

impl From<AdcChannel> for ads1x1x::ChannelSelection {
    fn from(channel: AdcChannel) -> Self {
        match channel {
            AdcChannel::Ch0 => ads1x1x::ChannelSelection::SingleA0,
            AdcChannel::Ch1 => ads1x1x::ChannelSelection::SingleA1,
            AdcChannel::Ch2 => ads1x1x::ChannelSelection::SingleA2,
            AdcChannel::Ch3 => ads1x1x::ChannelSelection::SingleA3,
        }
    }
}

impl From<PwmChannel> for pwm_pca9685::Channel {
    fn from(channel: PwmChannel) -> Self {
        match channel {
            PwmChannel::Ch1 => pwm_pca9685::Channel::C0,
            PwmChannel::Ch2 => pwm_pca9685::Channel::C1,
            PwmChannel::Ch3 => pwm_pca9685::Channel::C2,
            PwmChannel::Ch4 => pwm_pca9685::Channel::C3,
            PwmChannel::Ch5 => pwm_pca9685::Channel::C4,
            PwmChannel::Ch6 => pwm_pca9685::Channel::C5,
            PwmChannel::Ch7 => pwm_pca9685::Channel::C6,
            PwmChannel::Ch8 => pwm_pca9685::Channel::C7,
            PwmChannel::Ch9 => pwm_pca9685::Channel::C8,
            PwmChannel::Ch10 => pwm_pca9685::Channel::C9,
            PwmChannel::Ch11 => pwm_pca9685::Channel::C10,
            PwmChannel::Ch12 => pwm_pca9685::Channel::C11,
            PwmChannel::Ch13 => pwm_pca9685::Channel::C12,
            PwmChannel::Ch14 => pwm_pca9685::Channel::C13,
            PwmChannel::Ch15 => pwm_pca9685::Channel::C14,
            PwmChannel::Ch16 => pwm_pca9685::Channel::C15,
            PwmChannel::All => pwm_pca9685::Channel::All,
        }
    }
}

/// Set of available options to select ADC's channel.
#[derive(Debug, Clone, Copy)]
pub enum AdcChannel {
    Ch0,
    Ch1,
    Ch2,
    Ch3,
}

/// Set of options to control navigator's LEDs.
#[derive(Debug, Clone, Copy)]
pub enum UserLed {
    /// Attached to green LED through GPIO 24, labelled LED_1.
    Led1,
    /// Attached to blue LED through GPIO 25, labelled LED_2.
    Led2,
    /// Attached to red LED through GPIO 11, labelled LED_3.
    Led3,
}

impl fmt::Display for UserLed {
    fn fmt(&self, f: &mut fmt::Formatter) -> fmt::Result {
        match self {
            UserLed::Led1 => write!(f, "LED_1"),
            UserLed::Led2 => write!(f, "LED_2"),
            UserLed::Led3 => write!(f, "LED_3"),
        }
    }
}

/// Set of available options to select PWM's channel.
#[derive(Debug, Clone, Copy, PartialEq)]
pub enum PwmChannel {
    Ch1,
    Ch2,
    Ch3,
    Ch4,
    Ch5,
    Ch6,
    Ch7,
    Ch8,
    Ch9,
    Ch10,
    Ch11,
    Ch12,
    Ch13,
    Ch14,
    Ch15,
    Ch16,
    All,
}

/// The `AxisData` struct encapsulate values for the x, y, and z axes.
#[derive(Debug, Default, Clone, Copy)]
pub struct AxisData {
    pub x: f32,
    pub y: f32,
    pub z: f32,
}

/// Encapsulates the value of ADC's four channels.
#[derive(Debug, Default, Clone, Copy)]
pub struct ADCData {
    pub channel: [f32; 4],
}

/// Encapsulates the value of all sensors on the board.
#[derive(Debug, Default, Clone, Copy)]
pub struct SensorData {
    pub adc: ADCData,
    pub temperature: f32,
    pub pressure: f32,
    pub accelerometer: AxisData,
    pub magnetometer: AxisData,
    pub gyro: AxisData,
}

/// The `Led` struct represents the 3 LEDs on navigator board.
///
/// All this components were abstracted to be used directly from navigator module.
pub struct Led {
    first: Pin,
    second: Pin,
    third: Pin,
}

/// The `Navigator` struct contains various components used for navigator. It includes PWM control,
/// pressure and temperature sensing, analog-to-digital conversion, inertial measurement unit,
/// magnetometer, and LEDs control.
///
/// All this components were integrated and abstracted to be used directly from navigator module.
///
/// Please check [`Implementations`](struct.Navigator.html#implementations) and it's examples, then start to coding your applications!
pub struct Navigator {
    pwm: Pwm,
    bmp: Bmp280,
    adc: Ads1x1x<I2cInterface<I2cdev>, Ads1115, Resolution16Bit, ads1x1x::mode::OneShot>,
    imu: ICM20689<SpiInterface<Spidev, Pin>>,
    mag: Ak09915<I2cdev>,
    led: Led,
    neopixel: Strip,
}

impl Deref for Pwm {
    type Target = Pca9685<I2cdev>;

    fn deref(&self) -> &Self::Target {
        &self.pca
    }
}

impl DerefMut for Pwm {
    fn deref_mut(&mut self) -> &mut Self::Target {
        &mut self.pca
    }
}

/// The `Pwm` struct represents a PWM (Pulse Width Modulation) controller with a PCA9685 chip and it's
/// output enable pin.
///
pub struct Pwm {
    pca: Pca9685<I2cdev>,
    /// * `oe_pin`: The `oe_pin` component is a pin that is used to enable or disable the output of the PWM
    /// signal. It is connected to the Output Enable (OE) pin of the PCA9685 PWM controller.
    /// The default initialization of the navigator sets oe_pin to a digital high state, which disables the PCA9685's PWM.
    oe_pin: Pin,
}

/// Build pattern structure
pub struct NavigatorBuilder {
    rgb_led_strip_size: usize,
}

impl Default for Led {
    fn default() -> Self {
        Self::new()
    }
}

impl Led {
    pub fn new() -> Led {
        let mut led = Led {
            first: Pin::new(24),
            second: Pin::new(25),
            third: Pin::new(11),
        };

        for pin in led.as_mut_array().iter_mut() {
            pin.export().expect("Error: Error during led pins export");
            Delay {}.delay_ms(30_u16);
            pin.set_direction(Direction::High)
                .expect("Error: Setting led pins as output");
        }
        led
    }

    pub fn as_mut_array(&mut self) -> [&mut Pin; 3] {
        [&mut self.first, &mut self.second, &mut self.third]
    }

    pub fn select(&mut self, select: UserLed) -> &mut Pin {
        match select {
            UserLed::Led1 => &mut self.first,
            UserLed::Led2 => &mut self.second,
            UserLed::Led3 => &mut self.third,
        }
    }

    pub fn get_led(&mut self, select: UserLed) -> bool {
        let pin_struct = self.select(select);

        pin_struct
            .get_value()
            .unwrap_or_else(|_| panic!("Error: Get {} LED value", select))
            == 0
    }

    pub fn set_led(&mut self, select: UserLed, state: bool) {
        let pin_struct = self.select(select);

        pin_struct
            .set_value((!state).into())
            .unwrap_or_else(|_| panic!("Error: Set {} LED value to {}", select, state));
    }

    pub fn set_led_all(&mut self, state: bool) {
        for pin in self.as_mut_array().iter_mut() {
            pin.set_value((!state).into())
                .unwrap_or_else(|_| panic!("Error: Set LED value to {state}"));
        }
    }

    pub fn set_led_toggle(&mut self, select: UserLed) {
        let state = self.get_led(select);

        self.set_led(select, !state)
    }
}

impl Default for Navigator {
    fn default() -> Self {
        Self::new()
    }
}

impl NavigatorBuilder {
    pub fn with_rgb_led_strip_size(mut self, size: usize) -> Self {
        self.rgb_led_strip_size = size;
        self
    }

    pub fn build(self) -> Navigator {
        let dev = I2cdev::new("/dev/i2c-4").unwrap();
        let address = pwm_Address::default();
        let pwm = Pca9685::new(dev, address).unwrap();

        let dev = I2cdev::new("/dev/i2c-1").unwrap();
        let mag = Ak09915::new(dev);

        let dev = I2cdev::new("/dev/i2c-1").unwrap();
        let address = adc_Address::default();
        let adc = Ads1x1x::new_ads1115(dev, address);

        let mut bmp = Bmp280Builder::new()
            .path("/dev/i2c-1")
            .address(0x76)
            .build()
            .expect("Error: Failed to build BMP280 device");
        bmp.zero().unwrap();

        let mut neopixel = Strip::new(Bus::Spi0, self.rgb_led_strip_size).unwrap();
        // Clear RGB led strip before starting using it
        neopixel.clear();

        let mut spi = Spidev::open("/dev/spidev1.0").expect("Error: Failed during setting up SPI");
        let options = SpidevOptions::new()
            .bits_per_word(8)
            .max_speed_hz(10_000_000)
            .mode(spidev::SpiModeFlags::SPI_MODE_0)
            .build();
        spi.configure(&options)
            .expect("Error: Failed to configure SPI");

        //Define CS2 pin ICM-20602
        let cs_2 = Pin::new(16);
        cs_2.export().expect("Error: Error during CS2 export");
        Delay {}.delay_ms(30_u16);
        cs_2.set_direction(Direction::High)
            .expect("Error: Setting CS2 pin as output");

        //not using yet, define CS1 pin for MMC5983
        let cs_1 = Pin::new(17);
        cs_1.export().expect("Error: Error during CS1 export");
        Delay {}.delay_ms(30_u16);
        cs_1.set_direction(Direction::High)
            .expect("Error: Setting CS2 pin as output");

        //Define pwm OE_Pin - PWM initialize disabled
        let oe_pin = Pin::new(26);
        oe_pin.export().expect("Error: Error during oe_pin export");
        Delay {}.delay_ms(30_u16);
        oe_pin
            .set_direction(Direction::High)
            .expect("Error: Setting oe_pin pin as output");

        let imu = imu_Builder::new_spi(spi, cs_2);

        let led = Led::new();

        Navigator {
            adc: (adc),
            bmp: (bmp),
            pwm: Pwm { pca: pwm, oe_pin },
            mag: (mag),
            imu: (imu),
            led: (led),
            neopixel: (neopixel),
        }
    }
}

impl Navigator {
    pub fn new() -> Navigator {
        Self::create().build()
    }

    pub fn create() -> NavigatorBuilder {
        NavigatorBuilder {
            rgb_led_strip_size: 1, // There is only a single LED on the board
        }
    }

    pub fn init(&mut self) {
        self.self_test();
        //Initialize devices on navigator's default settings,
        //read more on ./navigator-api.pdf
        self.mag.init().unwrap();
        self.mag.set_mode(mag_Mode::Cont200Hz).unwrap();

        self.imu
            .setup(&mut Delay {})
            .expect("Error: Failed on IMU setup");
        self.imu.set_accel_range(AccelRange::Range_2g).unwrap();
        self.imu.set_gyro_range(GyroRange::Range_250dps).unwrap();

        self.adc.reset_internal_driver_state();
        self.adc
            .set_full_scale_range(ads1x1x::FullScaleRange::Within4_096V)
            .unwrap();
        self.adc
            .set_data_rate(ads1x1x::DataRate16Bit::Sps860)
            .unwrap();

        self.pwm.reset_internal_driver_state();
        self.pwm.use_external_clock().unwrap();
        self.pwm.set_prescale(100).unwrap();
        self.pwm.enable().unwrap();

        self.bmp.zero().unwrap();

        self.led.set_led_all(false);
    }

    pub fn self_test(&mut self) -> bool {
        //Check if the sensors are attached by it's IDs,
        //run self-test if they have.
        self.imu.check_identity(&mut Delay {}).unwrap();
        self.mag
            .self_test()
            .expect("Error : Error on magnetometer during self-test")
    }

    /// Sets the PWM IC to be enabled through firmware and OE_pin.
    ///
    /// # Arguments
    ///
    /// * `state` - The state of PWM output, it's enabled with a true logic value.
    ///
    /// # Examples
    ///
    /// Please check [`set_pwm_channel_value`](struct.Navigator.html#method.set_pwm_channel_value).
    pub fn set_pwm_enable(&mut self, state: bool) {
        if state {
            self.pwm.oe_pin.set_direction(Direction::Low).unwrap();
        } else {
            self.pwm.oe_pin.set_direction(Direction::High).unwrap();
        }
    }

    pub fn get_pwm_enable(&mut self) -> bool {
        self.pwm.oe_pin.get_value().expect("Error: Get PWM value") == 1
    }

    /// Sets the Duty Cycle (high value time) of selected channel.
    ///
    /// On PCA9685, this function sets the `OFF` counter and uses ON value as 0.
    ///
    /// # Further info
    /// Check **[7.3.3 LED output and PWM control](https://www.nxp.com/docs/en/data-sheet/PCA9685.pdf#page=16)**
    ///
    /// # Examples
    ///
    /// ```no_run
    /// use navigator_rs::{Navigator, PwmChannel};
    ///
    /// let mut nav = Navigator::new();
    ///
    /// nav.init();
    /// nav.set_pwm_enable(true);
    ///
    /// nav.set_pwm_freq_prescale(99); // sets the pwm frequency to 60 Hz
    /// nav.set_pwm_channel_value(PwmChannel::Ch1, 2048); // sets the duty cycle to 50%
    /// ```
    pub fn set_pwm_channel_value(&mut self, channel: PwmChannel, mut value: u16) {
        let max_value = 4095;
        if value > max_value {
            warn!("Invalid value. Value must be less than or equal {max_value}.");
            value = max_value;
        }
        self.pwm.set_channel_on(channel.into(), 0).unwrap();
        self.pwm.set_channel_off(channel.into(), value).unwrap();
    }

    /// Like [`set_pwm_channel_value`](struct.Navigator.html#method.set_pwm_channel_value). This function
    /// sets the Duty Cycle for a list of multiple channels.
    ///
    /// # Examples
    ///
    /// ```no_run
    /// use navigator_rs::{Navigator, PwmChannel};
    ///
    /// let mut nav = Navigator::new();
    ///
    /// nav.init();
    /// nav.set_pwm_enable(true);
    /// nav.set_pwm_freq_prescale(99); // sets the pwm frequency to 60 Hz
    ///
    /// let channels: [PwmChannel; 3] = [PwmChannel::Ch1, PwmChannel::Ch1, PwmChannel::Ch2];
    /// let values: [u16; 3] = [200, 1000, 300];
    ///
    /// nav.set_pwm_channels_value(&channels, 2048); // sets the duty cycle according to the list.
    /// ```
    pub fn set_pwm_channels_value<const N: usize>(
        &mut self,
        channels: &[PwmChannel; N],
        value: u16,
    ) {
        for &channel in channels.iter().take(N) {
            self.set_pwm_channel_value(channel, value)
        }
    }

    /// Like [`set_pwm_channel_value`](struct.Navigator.html#method.set_pwm_channel_value). This function
    /// sets the Duty Cycle for a list of multiple channels with multiple values.
    ///
    /// # Examples
    ///
    /// ```no_run
    /// use navigator_rs::{Navigator, PwmChannel};
    ///
    /// let mut nav = Navigator::new();
    ///
    /// nav.init();
    /// nav.set_pwm_enable(true);
    /// nav.set_pwm_freq_prescale(99); // sets the pwm frequency to 60 Hz
    ///
    /// let channels: [PwmChannel; 3] = [PwmChannel::Ch1, PwmChannel::Ch1, PwmChannel::Ch2];
    /// let values: [u16; 3] = [200, 1000, 300];
    ///
    /// nav.set_pwm_channels_values(&channels, &values); // sets the duty cycle according to the lists.
    /// ```
    pub fn set_pwm_channels_values<const N: usize>(
        &mut self,
        channels: &[PwmChannel; N],
        values: &[u16; N],
    ) {
        for i in 0..N {
            self.set_pwm_channel_value(channels[i], values[i])
        }
    }

    /// Sets the PWM frequency of [`Navigator`].
    ///
    /// It changes the PRE_SCALE value on PCA9685.
    ///
    /// The prescaler value can be calculated for an update rate using the formula:
    ///
    /// `prescale_value = round(clock_freq / (4096 * desired_freq)) - 1`.
    ///
    /// The minimum prescaler value is 3, which corresponds to 1526 Hz.
    /// The maximum prescaler value is 255, which corresponds to 24 Hz.
    ///
    /// If you want to control a servo, set a prescaler value of 100. This will
    /// correspond to a frequency of about 60 Hz, which is the frequency at
    /// which servos work.
    ///
    /// Internally, this function stops the oscillator and restarts it after
    /// setting the prescaler value if it was running.
    ///
    /// Re-run the set_pwm_channel_value() is required.
    ///
    /// # Further info
    /// Check **[7.3.5 - PWM frequency PRE_SCALE](https://www.nxp.com/docs/en/data-sheet/PCA9685.pdf#page=25)**
    ///
    /// # Examples
    ///
    /// ```no_run
    /// use navigator_rs::{Navigator, PwmChannel};
    ///
    /// let mut nav = Navigator::new();
    ///
    /// nav.init();
    /// nav.set_pwm_enable(true);
    ///
    /// nav.set_pwm_freq_prescale(99); // sets the pwm frequency to 60 Hz
    ///
    /// nav.set_pwm_channel_value(PwmChannel::Ch1, 2048); // sets the duty cycle to 50%
    /// ```
    pub fn set_pwm_freq_prescale(&mut self, mut value: u8) {
        let min_prescale = 3;
        if value < min_prescale {
            warn!("Invalid value. Value must be greater than {min_prescale}.");
            value = min_prescale;
        }
        self.pwm.set_prescale(value).unwrap();

        let clamped_freq = NAVIGATOR_PWM_XTAL_CLOCK_FREQ / (4_096.0 * (value as f32 + 1.0));
        info!("PWM frequency set to {clamped_freq:.2} Hz. Prescaler value: {value}");
    }

    /// Sets the pwm frequency in Hertz of [`Navigator`].
    ///
    /// The navigator module uses a crystal with a 24.5760 MHz clock. You can set a value for a frequency between 24 and 1526 Hz.
    ///
    /// # Examples
    ///
    /// ```no_run
    /// use navigator_rs::{Navigator, PwmChannel};
    /// use std::thread::sleep;
    /// use std::time::Duration;
    ///
    /// let mut nav = Navigator::new();
    ///
    /// nav.init();
    /// nav.set_pwm_enable(true);
    ///
    /// let mut i: f32 = 10.0;
    ///
    /// loop {
    ///     nav.set_pwm_freq_hz(i);
    ///     nav.set_pwm_channel_value(PwmChannel::Ch1, 2048); // sets the duty cycle to 50%
    ///     i = i + 10.0;
    ///     sleep(Duration::from_millis(1000));
    /// }
    /// ```
    pub fn set_pwm_freq_hz(&mut self, mut freq: f32) {
        let min_freq = 24.0;
        if freq < min_freq {
            warn!("Invalid value. Value must be greater than or equal to {min_freq}.");
            freq = min_freq;
        }

        let max_freq = 1526.0;
        if freq > max_freq {
            warn!("Invalid value. Value must be less than or equal to {max_freq}.");
            freq = max_freq;
        }

        let prescale_clamped_value =
            (NAVIGATOR_PWM_XTAL_CLOCK_FREQ / (4_096.0 * freq)).round() as u8 - 1;

        self.set_pwm_freq_prescale(prescale_clamped_value);
    }
    /// Gets the selected navigator LED output state. The true state means the LED is on.
    ///
    /// # Arguments
    ///
    /// * `select` - A pin selected from [`UserLed`](enum.UserLed.html).
    ///
    /// # Examples
    ///
    /// ```no_run
    /// use navigator_rs::{UserLed, Navigator};
    /// use std::thread::sleep;
    /// use std::time::Duration;
    ///
    /// let mut nav = Navigator::new();
    ///
    /// nav.init();
    /// loop {
    ///     let logic_value: bool = nav.get_led(UserLed::Led2);
    ///     println!("Blue LED logic value is {logic_value}.");
    ///     nav.set_led_toggle(UserLed::Led2);
    ///     sleep(Duration::from_millis(1000));
    /// }
    /// ```
    pub fn get_led(&mut self, select: UserLed) -> bool {
        self.led.get_led(select)
    }

    /// Sets the selected navigator LED output.
    ///
    /// # Arguments
    ///
    /// * `select` - A pin selected from [`UserLed`](enum.UserLed.html).
    /// * `state` - The value of output, LED is on with a true logic value.
    ///
    /// # Examples
    ///
    /// ```no_run
    /// use navigator_rs::{UserLed, Navigator};
    /// use std::thread::sleep;
    /// use std::time::Duration;
    ///
    /// let mut nav = Navigator::new();
    ///
    /// nav.init();
    /// loop {
    ///     nav.set_led(UserLed::Led1, true);
    ///     sleep(Duration::from_millis(1000));
    ///     nav.set_led(UserLed::Led1, false);
    ///     sleep(Duration::from_millis(1000));
    /// }
    /// ```
    pub fn set_led(&mut self, select: UserLed, state: bool) {
        self.led.set_led(select, state)
    }

    /// Toggle the output of selected LED.
    ///
    /// # Arguments
    ///
    /// * `select` - A pin selected from [`UserLed`](enum.UserLed.html).
    ///
    /// # Examples
    ///
    /// ```no_run
    /// use navigator_rs::{UserLed, Navigator};
    /// use std::thread::sleep;
    /// use std::time::Duration;
    ///
    /// let mut nav = Navigator::new();
    ///
    /// nav.init();
    /// loop {
    ///     nav.set_led_toggle(UserLed::Led1);
    ///     sleep(Duration::from_millis(1000));
    /// }
    /// ```
    pub fn set_led_toggle(&mut self, select: UserLed) {
        self.led.set_led_toggle(select)
    }

    /// Set all LEDs on desired state ( Blue, Green and Red ).
    ///
    /// # Arguments
    ///
    /// * `state` - The value of output, LED is on with a true logic value.
    ///
    /// # Examples
    ///
    /// ```no_run
    /// use navigator_rs::{Navigator};
    /// use std::thread::sleep;
    /// use std::time::Duration;
    ///
    /// let mut nav = Navigator::new();
    ///
    /// nav.init();
    /// loop {
    ///     nav.set_led_all(true);
    ///     sleep(Duration::from_millis(1000));
    ///     nav.set_led_all(false);
    ///     sleep(Duration::from_millis(1000));
    /// }
    /// ```
    pub fn set_led_all(&mut self, state: bool) {
        self.led.set_led_all(state)
    }

    /// Set the values of the neopixel LED array.
    ///
    /// # Arguments
    ///
    /// * `array` - A 2D array containing RGB values for each LED.
    /// Each inner array is a [u8; 3] representing the Red, Green and Blue from a LED.
    ///
    /// # Example
    ///
    /// ```no_run
    /// use navigator_rs::{Navigator};
    ///
    /// let mut nav = Navigator::new();
    ///
    /// nav.init();
    /// let mut leds = [[0, 0, 255], [0, 255, 0], [255, 0, 0]];
    /// nav.set_neopixel(&mut leds);
    /// ```
    ///
    /// This will set the first LED to blue, second to green, and third to red.
    pub fn set_neopixel(&mut self, array: &[[u8; 3]]) {
        for (index, value) in array.iter().enumerate() {
            self.neopixel.leds[index] = StripLed::from_rgb(value[0], value[1], value[2]);
        }
        self.neopixel.update().unwrap();
    }

    /// Reads the magnetometer Ak09915 of [`Navigator`].
    ///
    /// Measurements in \[µT\]
    ///
    /// # Examples
    ///
    /// ```no_run
    /// use navigator_rs::{Navigator};
    /// use std::thread::sleep;
    /// use std::time::Duration;
    ///
    /// let mut nav = Navigator::new();
    /// nav.init();
    ///
    /// loop {
    ///     let mag = nav.read_mag();
    ///     println!("mag values: X={}, Y={}, Z={} [µT]", mag.x, mag.y, mag.z);
    ///     sleep(Duration::from_millis(1000));
    /// }
    /// ```
    pub fn read_mag(&mut self) -> AxisData {
        let (x, y, z) = self.mag.read().unwrap();
        // Change the axes to navigator's standard. Right-handed, Z-axis down (aeronautical frame, NED).
        AxisData {
            x: y,
            y: x * -1.0,
            z,
        }
    }

    /// Reads the temperature using BMP280 of [`Navigator`].
    ///
    /// Measurements in \[˚C\]
    ///
    /// # Examples
    ///
    /// ```no_run
    /// use navigator_rs::{Navigator};
    /// use std::thread::sleep;
    /// use std::time::Duration;
    ///
    /// let mut nav = Navigator::new();
    /// nav.init();
    ///
    /// loop {
    ///     let temperature = nav.read_temperature();
    ///     println!("temperature value: {temperature} [˚C]");
    ///     sleep(Duration::from_millis(1000));
    /// }
    /// ```
    pub fn read_temperature(&mut self) -> f32 {
        self.bmp.temperature_celsius().unwrap()
    }

    /// Reads the altitude based on pressure values measured by BMP280 of [`Navigator`].
    ///
    /// Measurements in \[m\]
    ///
    /// # Examples
    ///
    /// ```no_run
    /// use navigator_rs::{Navigator};
    /// use std::thread::sleep;
    /// use std::time::Duration;
    ///
    /// let mut nav = Navigator::new();
    /// nav.init();
    ///
    /// loop {
    ///     let altitude = nav.read_altitude();
    ///     println!("altitude value: {altitude} [m]");
    ///     sleep(Duration::from_millis(1000));
    /// }
    /// ```
    pub fn read_altitude(&mut self) -> f32 {
        self.bmp.altitude_m().unwrap()
    }

    /// Reads the pressure based on BMP280 of [`Navigator`].
    ///
    /// Measurements in \[kPa\]
    ///
    /// # Examples
    ///
    /// ```no_run
    /// use navigator_rs::{Navigator};
    /// use std::thread::sleep;
    /// use std::time::Duration;
    ///
    /// let mut nav = Navigator::new();
    /// nav.init();
    ///
    /// loop {
    ///     let pressure = nav.read_pressure();
    ///     println!("pressure value: {pressure} [kPa]");
    ///     sleep(Duration::from_millis(1000));
    /// }
    /// ```
    pub fn read_pressure(&mut self) -> f32 {
        self.bmp.pressure_kpa().unwrap()
    }

    /// Reads the ADC based on ADS1115 of [`Navigator`].
    ///
    /// Measurements in \[V\]
    ///
    /// Same as [`read_adc`](struct.Navigator.html#method.read_adc), but it returns an array with all channel readings
    ///
    /// # Examples
    ///
    /// ```no_run
    /// use navigator_rs::{Navigator};
    /// use std::thread::sleep;
    /// use std::time::Duration;
    ///
    /// let mut nav = Navigator::new();
    /// nav.init();
    ///
    /// loop {
    ///     let adc_data = nav.read_adc_all();
    ///     println!(
    ///         "adc values per channel: 1: {}, 2: {}, 3: {}, 4: {}",
    ///         adc_data.channel[0], adc_data.channel[1], adc_data.channel[2], adc_data.channel[3]
    ///     );
    ///     sleep(Duration::from_millis(1000));
    /// }
    /// ```
    pub fn read_adc_all(&mut self) -> ADCData {
        ADCData {
            channel: [
                self.read_adc(AdcChannel::Ch0),
                self.read_adc(AdcChannel::Ch1),
                self.read_adc(AdcChannel::Ch2),
                self.read_adc(AdcChannel::Ch3),
            ],
        }
    }

    /// Reads the ADC based on ADS1115 of [`Navigator`].
    ///
    /// Measurements in \[V\]
    ///
    /// # Examples
    ///
    /// ```no_run
    /// use navigator_rs::{AdcChannel, Navigator};
    /// use std::thread::sleep;
    /// use std::time::Duration;
    ///
    /// let mut nav = Navigator::new();
    /// nav.init();
    ///
    /// loop {
    ///     println!("ADC channel 1 value: {} [V]", nav.read_adc(AdcChannel::Ch1));
    ///     sleep(Duration::from_millis(1000));
    /// }
    pub fn read_adc(&mut self, channel: AdcChannel) -> f32 {
        let conversion_volts: f32 = 0.000_125; // According to data-sheet, LSB = 125 μV for ±4.096 scale register, navigator's default
        block!(self.adc.read(channel.into())).unwrap() as f32 * conversion_volts
    }

    /// Reads acceleration based on ICM20689 of [`Navigator`].
    ///
    /// Measurements in \[m/s²\]
    ///
    /// # Examples
    ///
    /// ```no_run
    /// use navigator_rs::{Navigator};
    /// use std::thread::sleep;
    /// use std::time::Duration;
    ///
    /// let mut nav = Navigator::new();
    /// nav.init();
    ///
    /// loop {
    ///     let accel = nav.read_accel();
    ///     println!("accel values: X={}, Y={}, Z={} [m/s²]", accel.x, accel.y, accel.z);
    ///     sleep(Duration::from_millis(1000));
    /// }
    /// ```
    pub fn read_accel(&mut self) -> AxisData {
        let reading: [f32; 3] = self.imu.get_scaled_accel().unwrap();
        // Change the axes to navigator's standard. Right-handed, Z-axis down (aeronautical frame, NED).
        // Obs.: ICM20602 sensor is measuring with inverted directions, not following data-sheet.
        //       Thus, with IC's placement only Z needs to be inverted.
        AxisData {
            x: reading[0],
            y: reading[1],
            z: reading[2] * -1.0,
        }
    }

    /// Reads angular velocity based on ICM20689 of [`Navigator`].
    ///
    /// Measurements in \[rad/s\]
    ///
    /// # Examples
    ///
    /// ```no_run
    /// use navigator_rs::{Navigator};
    /// use std::thread::sleep;
    /// use std::time::Duration;
    ///
    /// let mut nav = Navigator::new();
    /// nav.init();
    ///
    /// loop {
    ///     let gyro = nav.read_gyro();
    ///     println!("accel values: X={}, Y={}, Z={} [rad/s]", gyro.x, gyro.y, gyro.z);
    ///     sleep(Duration::from_millis(1000));
    /// }
    /// ```
    pub fn read_gyro(&mut self) -> AxisData {
        let reading: [f32; 3] = self.imu.get_scaled_gyro().unwrap();
        // Change the axes to navigator's standard. Right-handed, Z-axis down (aeronautical frame, NED).
        AxisData {
            x: reading[0] * -1.0,
            y: reading[1] * -1.0,
            z: reading[2],
        }
    }

    /// Reads all sensors and stores on a single structure.
    ///
    /// # Examples
    ///
    /// ```no_run
    /// use navigator_rs::{Navigator, SensorData};
    /// use std::thread::sleep;
    /// use std::time::Duration;
    ///
    /// let mut nav = Navigator::new();
    /// nav.init();
    ///
    /// loop {
    ///     println!("---------------reading---------------");
    ///     let sensors_data = nav.read_all();
    ///     println!("{sensors_data:#?}");
    ///     sleep(Duration::from_millis(1000));
    /// }
    /// ```
    pub fn read_all(&mut self) -> SensorData {
        SensorData {
            adc: self.read_adc_all(),
            temperature: self.read_temperature(),
            pressure: self.read_pressure(),
            accelerometer: self.read_accel(),
            magnetometer: self.read_mag(),
            gyro: self.read_gyro(),
        }
    }

    pub fn fmt_debug(&mut self) -> impl fmt::Debug {
        #[allow(dead_code)]
        #[derive(Debug)]
        struct Bmp {
            temperature: f32,
            altitude: f32,
            pressure: f32,
        }

        #[allow(dead_code)]
        #[derive(Debug)]
        struct Imu {
            accelerometer: AxisData,
            gyroscope: AxisData,
        }

        #[allow(dead_code)]
        #[derive(Debug)]
        struct Navigator {
            adc: ADCData,
            bmp: Bmp,
            imu: Imu,
            mag: AxisData,
        }

        Navigator {
            adc: self.read_adc_all(),
            bmp: Bmp {
                temperature: self.read_temperature(),
                altitude: self.read_altitude(),
                pressure: self.read_pressure(),
            },
            imu: Imu {
                accelerometer: self.read_accel(),
                gyroscope: self.read_gyro(),
            },
            mag: self.read_mag(),
        }
    }
}
