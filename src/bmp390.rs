use std::error::Error;
use std::path::PathBuf;

use embedded_hal::i2c::I2c;
use linux_embedded_hal::I2cdev;

use crate::peripherals::{
    AnyHardware, BarometerSensor, PeripheralClass, PeripheralInfo, Peripherals, TemperatureSensor,
};

const DEFAULT_I2C_ADDRESS: u8 = 0x76;
const DEFAULT_I2C_PATH: &str = "/dev/i2c-1";

/// Wrapper type for results
pub type Result<T> = std::result::Result<T, Box<dyn Error>>;

/// BMP390 Registers
enum Register {
    ChipId = 0x00,
    ErrReg = 0x02,
    Event = 0x10,
    /// Calibration data start
    NvmParT1_0 = 0x31,
    /// data registers
    Data0 = 0x04, // pressure XLSB
    Data3 = 0x07, // temperature XLSB

    PwrCtrl = 0x1B,
}

/// Output from the BMP390 consists of ADC outputs.
///
/// These must be compensated using formulas from the datasheet to obtain the actual temperature and pressure values,
/// using coefficients stored in non-volatile memory (NVM).
///
/// # Datasheet
/// - Section 3.11 Output compensation.
/// - Appendix A: Computation formulae reference implementation.
#[derive(Debug, Default, Clone, Copy)]
struct CalibrationCoefficients {
    par_t1: f32,
    par_t2: f32,
    par_t3: f32,
    par_p1: f32,
    par_p2: f32,
    par_p3: f32,
    par_p4: f32,
    par_p5: f32,
    par_p6: f32,
    par_p7: f32,
    par_p8: f32,
    par_p9: f32,
    par_p10: f32,
    par_p11: f32,
}

pub struct Bmp390 {
    i2c_device: I2cdev,
    address: u8,
    calibration: CalibrationCoefficients,
    ground_pressure: f32,
}

pub struct Bmp390Device {
    bmp390: Bmp390,
    info: PeripheralInfo,
}

impl Bmp390Device {
    pub fn builder() -> Bmp390Builder {
        Bmp390Builder::new()
    }

    pub fn get_peripheral_info(&self) -> &PeripheralInfo {
        &self.info
    }
}

impl AnyHardware for Bmp390Device {
    fn as_temperature_sensor(&mut self) -> Option<&mut dyn TemperatureSensor> {
        Some(self)
    }

    fn as_barometer_sensor(&mut self) -> Option<&mut dyn BarometerSensor> {
        Some(self)
    }
}

pub struct Bmp390Builder {
    i2c_address: u8,
    i2c_path: PathBuf,
    ground_pressure: f32,
}

impl Bmp390Builder {
    pub fn new() -> Self {
        Bmp390Builder {
            i2c_address: DEFAULT_I2C_ADDRESS,
            i2c_path: PathBuf::from(DEFAULT_I2C_PATH),
            ground_pressure: 0.,
        }
    }

    pub fn address(&mut self, address: u16) -> &mut Self {
        self.i2c_address = address as u8;
        self
    }

    pub fn path(&mut self, path: impl Into<PathBuf>) -> &mut Self {
        self.i2c_path = path.into();
        self
    }

    pub fn ground_pressure(&mut self, pressure: f32) -> &mut Self {
        self.ground_pressure = pressure;
        self
    }

    pub fn build(&self) -> Result<Bmp390Device> {
        let dev = I2cdev::new(&self.i2c_path)?;
        let mut sensor = Bmp390 {
            i2c_device: dev,
            address: self.i2c_address,
            calibration: CalibrationCoefficients::default(),
            ground_pressure: self.ground_pressure,
        };

        sensor.begin()?;

        if self.ground_pressure != 0. {
            sensor.zero()?;
        }

        Ok(Bmp390Device {
            bmp390: sensor,
            info: PeripheralInfo {
                peripheral: Peripherals::Bmp390,
                class: vec![PeripheralClass::Pressure, PeripheralClass::Temperature],
            },
        })
    }
}

impl Default for Bmp390Builder {
    fn default() -> Self {
        Self::new()
    }
}

impl Bmp390 {
    fn write_reg(&mut self, reg: Register, value: u8) -> Result<()> {
        self.i2c_device.write(self.address, &[reg as u8, value])?;
        Ok(())
    }

    fn read_reg(&mut self, reg: Register) -> Result<u8> {
        let mut buf = [0u8; 1];
        self.i2c_device
            .write_read(self.address, &[reg as u8], &mut buf)?;
        Ok(buf[0])
    }

    fn read_burst(&mut self, start: Register, buf: &mut [u8]) -> Result<()> {
        self.i2c_device
            .write_read(self.address, &[start as u8], buf)?;
        Ok(())
    }

    fn begin(&mut self) -> Result<()> {
        let chip_id = self.read_reg(Register::ChipId)?;
        if chip_id != 0x60 {
            return Err(format!("BMP390: Invalid chip ID: 0x{:02X}", chip_id).into());
        }

        // reset device not strictly necessary here
        // Configure sensor (similar to BMP390 defaults)
        // Enable temperature and pressure, normal mode
        // PwrCtrl: enable temp + pressure + normal mode: 0x33 (temp + press + normal)
        self.write_reg(Register::PwrCtrl, 0x33)?;

        // wait a bit for device to load NVM
        std::thread::sleep(std::time::Duration::from_millis(2));

        self.read_calibration()?;

        Ok(())
    }

    pub fn zero(&mut self) -> Result<f32> {
        self.ground_pressure = self.pressure_pa()?;
        Ok(self.ground_pressure)
    }

    fn read_calibration(&mut self) -> Result<()> {
        // Reading calibration coefficients from BMP390:
        let mut data = [0u8; 21];
        self.read_burst(Register::NvmParT1_0, &mut data)?;

        // Parsing according to BMP390 datasheet (from earlier code):
        let nvm_par_t1 = ((data[1] as u16) << 8) | data[0] as u16;
        let nvm_par_t2 = ((data[3] as u16) << 8) | data[2] as u16;
        let nvm_par_t3 = data[4] as i8;
        let nvm_par_p1 = ((data[6] as i16) << 8) | data[5] as i16;
        let nvm_par_p2 = ((data[8] as i16) << 8) | data[7] as i16;
        let nvm_par_p3 = data[9] as i8;
        let nvm_par_p4 = data[10] as i8;
        let nvm_par_p5 = ((data[12] as u16) << 8) | data[11] as u16;
        let nvm_par_p6 = ((data[14] as u16) << 8) | data[13] as u16;
        let nvm_par_p7 = data[15] as i8;
        let nvm_par_p8 = data[16] as i8;
        let nvm_par_p9 = ((data[18] as i16) << 8) | data[17] as i16;
        let nvm_par_p10 = data[19] as i8;
        let nvm_par_p11 = data[20] as i8;

        self.calibration = CalibrationCoefficients {
            par_t1: (nvm_par_t1 as f32) / 0.00390625,
            par_t2: (nvm_par_t2 as f32) / 1073741824.0,
            par_t3: (nvm_par_t3 as f32) / 281474976710656.0,
            par_p1: ((nvm_par_p1 as f32) - 16384.0) / 1048576.0,
            par_p2: ((nvm_par_p2 as f32) - 16384.0) / 536870912.0,
            par_p3: (nvm_par_p3 as f32) / 4294967296.0,
            par_p4: (nvm_par_p4 as f32) / 137438953472.0,
            par_p5: (nvm_par_p5 as f32) / 0.125,
            par_p6: (nvm_par_p6 as f32) / 64.0,
            par_p7: (nvm_par_p7 as f32) / 256.0,
            par_p8: (nvm_par_p8 as f32) / 32768.0,
            par_p9: (nvm_par_p9 as f32) / 281474976710656.0,
            par_p10: (nvm_par_p10 as f32) / 281474976710656.0,
            par_p11: (nvm_par_p11 as f32) / 36893488147419103232.0,
        };

        Ok(())
    }

    fn read_raw_measurements(&mut self) -> Result<(u32, u32)> {
        // BMP390 data: pressure in DATA_0..2, temp in DATA_3..5
        let mut buf = [0u8; 6];
        self.read_burst(Register::Data0, &mut buf)?;

        let raw_pressure = (buf[0] as u32) | ((buf[1] as u32) << 8) | ((buf[2] as u32) << 16);
        let raw_temperature = (buf[3] as u32) | ((buf[4] as u32) << 8) | ((buf[5] as u32) << 16);

        Ok((raw_temperature, raw_pressure))
    }

    fn temperature_celsius(&mut self) -> Result<f32> {
        let (raw_temp, _) = self.read_raw_measurements()?;
        let t_lin = self.compensate_temperature(raw_temp);
        Ok(t_lin)
    }

    fn pressure_pa(&mut self) -> Result<f32> {
        let (raw_temp, raw_press) = self.read_raw_measurements()?;

        let t_lin = self.compensate_temperature(raw_temp);
        let pressure = self.compensate_pressure(t_lin, raw_press);
        Ok(pressure)
    }

    /// Compensate a temperature reading according to calibration coefficients.
    ///
    /// # Datasheet
    /// Apendix A, Section 8.5
    fn compensate_temperature(&self, raw_temp: u32) -> f32 {
        let uncomp = raw_temp as f32;
        let partial_data1 = uncomp - self.calibration.par_t1;
        let partial_data2 = partial_data1 * self.calibration.par_t2;
        partial_data2 + (partial_data1 * partial_data1) * self.calibration.par_t3
    }

    /// Compensate a pressure reading according to calibration coefficients.
    ///
    /// # Datasheet
    /// Apendix A, Section 8.6
    fn compensate_pressure(&self, temperature: f32, raw_press: u32) -> f32 {
        let uncomp = raw_press as f32;

        let partial_data1 = self.calibration.par_p6 * temperature;
        let partial_data2 = self.calibration.par_p7 * temperature * temperature;
        let partial_data3 = self.calibration.par_p8 * temperature * temperature * temperature;
        let partial_out1 = self.calibration.par_p5 + partial_data1 + partial_data2 + partial_data3;

        let partial_data1 = self.calibration.par_p2 * temperature;
        let partial_data2 = self.calibration.par_p3 * temperature * temperature;
        let partial_data3 = self.calibration.par_p4 * temperature * temperature * temperature;
        let partial_out2 =
            uncomp * (self.calibration.par_p1 + partial_data1 + partial_data2 + partial_data3);

        let partial_data1 = uncomp * uncomp;
        let partial_data2 = self.calibration.par_p9 + self.calibration.par_p10 * temperature;
        let partial_data3 = partial_data1 * partial_data2;
        let partial_data4 = partial_data3 + uncomp * uncomp * uncomp * self.calibration.par_p11;

        partial_out1 + partial_out2 + partial_data4
    }
}

impl TemperatureSensor for Bmp390Device {
    fn read_temperature(&mut self) -> Result<f32> {
        let temperature = self.bmp390.temperature_celsius()?;
        Ok(temperature)
    }
}

impl BarometerSensor for Bmp390Device {
    fn read_pressure(&mut self) -> Result<f32> {
        let pressure = self.bmp390.pressure_pa()? / 1000.0;
        Ok(pressure)
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use std::{thread::sleep, time::Duration};

    #[ignore]
    #[test]
    fn test_bmp390_pi_5() {
        let mut baro = Bmp390Device::builder()
            .build()
            .expect("Failed to build BMP390");
        for _ in 0..10 {
            println!("BMP390 temperature: {:?}", baro.read_temperature());
            println!("BMP390 pressure: {:?}", baro.read_pressure());
            sleep(Duration::from_millis(100));
        }
    }
}
