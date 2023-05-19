use navigator_rs::{pwm_Channel, Navigator, SensorData};
use std::thread::sleep;
use std::time::Duration;

fn main() {
    println!("Creating your navigator module!");
    let mut nav = Navigator::new();

    println!("Setting up your navigator, ahoy!");
    nav.init();

    println!("Reading all sensors individually");
    let read = nav.read_adc();
    println!(
        "adc values per channel: 1 : {},2: {}, 3: {}, 4: {}",
        read.channel[0], read.channel[1], read.channel[2], read.channel[3]
    );

    println!("pressure values: {}", nav.read_pressure());
    println!("temperature values: {}", nav.read_temperature());
    let read = nav.read_accel();
    println!("accel values: X={},Y={},Z={}", read.x, read.y, read.z);
    let read = nav.read_gyro();
    println!("gyro values: X={},Y={},Z={}", read.x, read.y, read.z);
    let read = nav.read_mag();
    println!("mag values: X={},Y={},Z={}", read.x, read.y, read.z);
    println!("Reading all sensors with read_all");
    loop {
        nav.set_pwm_channel_value(pwm_Channel::All, 0);
        let sensor_data: SensorData = nav.read_all();
        println!(
            "ADC values: {}, {}, {}, {}",
            sensor_data.adc.channel[0],
            sensor_data.adc.channel[1],
            sensor_data.adc.channel[2],
            sensor_data.adc.channel[3],
        );
        println!("PRES   :   P: {}", sensor_data.pressure);
        println!("TEMP   :   T: {}", sensor_data.temperature);
        println!(
            "ACCE   :   X: {}, Y: {}, Z: {}",
            sensor_data.accelerometer.x, sensor_data.accelerometer.y, sensor_data.accelerometer.z
        );
        println!(
            "GYRO  :   X: {}, Y: {}, Z: {}",
            sensor_data.gyro.x, sensor_data.gyro.y, sensor_data.gyro.z
        );
        println!(
            "MAGN   :   X: {}, Y: {}, Z: {}",
            sensor_data.magnetometer.x, sensor_data.magnetometer.y, sensor_data.magnetometer.z
        );
        println!("---------------reading---------------");
        sleep(Duration::from_millis(1000))
    }
}

//Output for the example:
//---------------------------------------------------------------
// Creating your navigator module!
// Setting up your navigator, ahoy!
// Self-test passed
// Magnetometer: x=-6, y=-6, z=-394
// Reading all sensors individually
// adc values per channel: 1 : 411,2: 421, 3: 429, 4: 439
// pressure values: 101.08484
// temperature values: 38.22
// accel values: X=0.08380005,Y=-0.14605151,Z=-9.878829
// gyro values: X=0.002263687,Y=0.023702133,Z=0.0038615838
// mag values: X=55,Y=-129,Z=108
// Reading all sensors with read_all
// ADC values: 412, 419, 430, 440
// PRES   :   P: 101.083885
// TEMP   :   T: 38.22
// ACCE   :   X: 0.06464575, Y: -0.15562867, Z: -9.876434
// GYRO  :   X: -0.0031957934, Y: 0.02117213, Z: 0.0055926386
// MAGN   :   X: 56, Y: -141, Z: 101
