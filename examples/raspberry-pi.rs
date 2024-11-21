use navigator_rs::Navigator;
use std::thread::sleep;
use std::time::Duration;

fn main() {
    println!("Creating your navigator module!");
    let mut nav = Navigator::new();

    println!("Setting up your navigator, ahoy!");
    nav.set_pwm_enable(true);
    nav.set_pwm_frequency(60.0);

    loop {
        nav.set_duty_cycle_all(0.25);
        sleep(Duration::from_millis(1000));
    }
}
