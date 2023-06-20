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
use nb::block;
use pwm_pca9685::{Address as pwm_Address, Pca9685};

pub use pwm_pca9685::Channel as pwm_Channel;

pub use ads1x1x::ChannelSelection as adc_Channel;

pub struct AxisData {
    pub x: f32,
    pub y: f32,
    pub z: f32,
}

pub struct ADCData {
    pub channel: [i16; 4],
}

pub struct SensorData {
    pub adc: ADCData,
    pub temperature: f32,
    pub pressure: f32,
    pub accelerometer: AxisData,
    pub magnetometer: AxisData,
    pub gyro: AxisData,
}

pub struct Navigator {
    pwm: Pca9685<I2cdev>,
    bmp: Bmp280,
    adc: Ads1x1x<I2cInterface<I2cdev>, Ads1115, Resolution16Bit, ads1x1x::mode::OneShot>,
    imu: ICM20689<SpiInterface<Spidev, Pin>>,
    mag: Ak09915<I2cdev>,
    led: Led,
}

pub struct Led {
    first: Pin,
    second: Pin,
    third: Pin,
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

    pub fn all_on(&mut self) {
        for pin in self.as_mut_array().iter_mut() {
            pin.set_value(0).expect("Error: Set led value to 0");
        }
    }

    pub fn all_off(&mut self) {
        for pin in self.as_mut_array().iter_mut() {
            pin.set_value(1).expect("Error: Set led value to 1");
        }
    }
}

impl Default for Navigator {
    fn default() -> Self {
        Self::new()
    }
}

impl Navigator {
    pub fn new() -> Navigator {
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

        let mut spi = Spidev::open("/dev/spidev1.0").expect("Error: Failled during setting up SPI");
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

        let imu = imu_Builder::new_spi(spi, cs_2);

        let led = Led::new();

        Self {
            adc: (adc),
            bmp: (bmp),
            pwm: (pwm),
            mag: (mag),
            imu: (imu),
            led: (led),
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

        self.pwm.reset_internal_driver_state();
        self.pwm.use_external_clock().unwrap();
        self.pwm.set_prescale(100).unwrap();
        self.pwm.enable().unwrap();

        self.bmp.zero().unwrap();

        self.led.all_off()
    }

    pub fn self_test(&mut self) -> bool {
        //Check if the sensors are attached by it's IDs,
        //run self-test if they have.
        self.imu.check_identity(&mut Delay {}).unwrap();
        self.mag
            .self_test()
            .expect("Error : Error on magnetometer during self-test")
    }

    pub fn set_pwm_channel_value(&mut self, channel: pwm_Channel, value: u16) {
        self.pwm.set_channel_on(channel, 0).unwrap();
        self.pwm.set_channel_off(channel, value).unwrap();
    }

    pub fn set_pwm_channels_value<const N: usize>(
        &mut self,
        channels: &[pwm_Channel; N],
        value: u16,
    ) {
        for &channel in channels.iter().take(N) {
            self.set_pwm_channel_value(channel, value)
        }
    }

    pub fn set_pwm_channels_values<const N: usize>(
        &mut self,
        channels: &[pwm_Channel; N],
        values: &[u16; N],
    ) {
        for i in 0..N {
            self.set_pwm_channel_value(channels[i], values[i])
        }
    }

    pub fn set_pwm_freq(&mut self) {
        todo!()
    }

    pub fn set_pwm_off(&mut self) {
        todo!()
    }

    pub fn set_pwm_on(&mut self) {
        todo!()
    }

    pub fn set_led_on(&mut self) {
        self.led.all_on()
    }

    pub fn set_led_off(&mut self) {
        self.led.all_off()
    }

    pub fn read_mag(&mut self) -> AxisData {
        let (x, y, z) = self.mag.read().unwrap();
        AxisData { x, y, z }
    }

    pub fn read_temperature(&mut self) -> f32 {
        self.bmp.temperature_celsius().unwrap()
    }

    pub fn read_altitude(&mut self) -> f32 {
        self.bmp.altitude_m().unwrap()
    }

    pub fn read_pressure(&mut self) -> f32 {
        self.bmp.pressure_kpa().unwrap()
    }

    pub fn read_adc_all(&mut self) -> ADCData {
        ADCData {
            channel: [
                self.read_adc(adc_Channel::SingleA0),
                self.read_adc(adc_Channel::SingleA1),
                self.read_adc(adc_Channel::SingleA2),
                self.read_adc(adc_Channel::SingleA3),
            ],
        }
    }

    pub fn read_adc(&mut self, channel: adc_Channel) -> i16 {
        block!(self.adc.read(channel)).unwrap()
    }

    pub fn read_accel(&mut self) -> AxisData {
        let reading: [f32; 3] = self.imu.get_scaled_accel().unwrap();
        AxisData {
            x: reading[0],
            y: reading[1],
            z: reading[2],
        }
    }

    pub fn read_gyro(&mut self) -> AxisData {
        let reading: [f32; 3] = self.imu.get_scaled_gyro().unwrap();
        AxisData {
            x: reading[0],
            y: reading[1],
            z: reading[2],
        }
    }

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
}
