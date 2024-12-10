use navigator_rs::{Navigator, UserLed};
use std::thread::sleep;
use std::time::Duration;

use rand::seq::SliceRandom;

fn main() {
    let mut nav = Navigator::create().build_navigator_v2_pi5();

    let colors = [
        [40u8, 0, 0],
        [0, 40, 0],
        [0, 0, 40],
        [40, 40, 0],
        [0, 40, 40],
        [40, 0, 40],
    ];

    println!("LED show started!");
    loop {
        nav.set_neopixel(&[*colors.choose(&mut rand::thread_rng()).unwrap()]);
        for led in [UserLed::Led1, UserLed::Led2, UserLed::Led3] {
            nav.set_led_toggle(led);
            sleep(Duration::from_millis(300));
        }

        println!("{:?}", nav.read_all());
    }
}
