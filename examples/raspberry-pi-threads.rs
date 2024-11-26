use navigator_rs::{Navigator, UserLed};
use std::sync::{Arc, Mutex, RwLock};
use std::thread::{self, sleep};
use std::time::Duration;

fn main() {
    println!("Creating your SensorData cache!");
    let sensor_data = Arc::new(RwLock::new(navigator_rs::SensorData::default()));

    println!("Creating your navigator module!");
    let nav = Arc::new(Mutex::new(Navigator::new()));

    // This code block creates a thread that updates the `sensor_data` periodically.
    let sensor_data_cloned = sensor_data.clone();
    let nav_cloned = nav.clone();
    thread::Builder::new()
        .name("Sensor reader".into())
        .spawn(move || loop {
            if let Ok(mut sensor_data) = sensor_data_cloned.write() {
                *sensor_data = nav_cloned.lock().unwrap().read_all();
                println!("Updated value: {sensor_data:?}");
            }
            sleep(Duration::from_millis(10000));
        })
        .expect("Failed to spawn the sensor reader thread");

    // This code block creates a thread that updates the `UserLed` according to its position, based on gravity.
    // It will keep the LED state on, indicating that the navigator has fully flipped to axes X and Y or turned upside down (Z-axis).
    // You can use it to also monitor abrupt collisions!
    let sensor_data_cloned = sensor_data.clone();
    let nav_cloned = nav.clone();
    thread::Builder::new()
        .name("Flip monitor".into())
        .spawn(move || loop {
            if let Ok(sensor_data) = sensor_data_cloned.read() {
                if sensor_data.accelerometer.x.abs() > 8.00 {
                    nav_cloned.lock().unwrap().set_led(UserLed::Led1, true)
                };
                if sensor_data.accelerometer.y.abs() > 8.00 {
                    nav_cloned.lock().unwrap().set_led(UserLed::Led2, true)
                };
                if sensor_data.accelerometer.z < -8.00 {
                    nav_cloned.lock().unwrap().set_led(UserLed::Led3, true)
                };
            }
            sleep(Duration::from_millis(5000));
        })
        .expect("Failed to spawn the flip monitor thread");

    // This code block could also be another thread, but we are going to run in the main one.
    // This could work as a server get handler, or an instance that could be used by different services for monitoring and analysis.
    loop {
        if let Ok(sensor_data) = sensor_data.read() {
            println!("Read SensorData: {sensor_data:?}");
        }
        sleep(Duration::from_millis(1000));
    }
}
