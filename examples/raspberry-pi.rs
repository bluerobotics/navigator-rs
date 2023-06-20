use navigator_rs::{pwm_Channel, Navigator, SensorData};
use std::thread::sleep;
use std::time::Duration;

fn main() {
    println!("Creating your navigator module!");
    let mut nav = Navigator::new();

    println!("Setting up your navigator, ahoy!");
    nav.init();

    loop {
        nav.set_pwm_channel_value(pwm_Channel::All, 0);
        println!("{:#?}", nav.fmt_debug());
        sleep(Duration::from_millis(1000));
    }
}
