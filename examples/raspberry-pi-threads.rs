use navigator_rs::Navigator;
use std::sync::{Arc, Mutex, RwLock};
use std::thread::{self, sleep};
use std::time::Duration;

fn main() {
    println!("Creating your SensorData cache!");
    let sensor_data = Arc::new(RwLock::new(navigator_rs::SensorData::default()));

    println!("Creating your navigator module!");
    let nav = Arc::new(Mutex::new(Navigator::new()));

    println!("Setting up your navigator, ahoy!");
    nav.lock().unwrap().init();

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

    // This code block could be also other thread, but just run on the main one.
    // This could work as a server get handler, or an instance that can be used by different services to monitor and analysis.
    loop {
        if let Ok(sensor_data) = sensor_data.read() {
            println!("Read SensorData: {sensor_data:?}");
        }
        sleep(Duration::from_millis(1000));
    }
}
