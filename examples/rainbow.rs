use navigator_rs::Navigator;
use std::thread::sleep;
use std::time::Duration;

fn color_from_sine(percentage: f32) -> [u8; 3] {
    let pi = std::f32::consts::PI;
    let red = (percentage * 2.0 * pi).sin() * 0.5 + 0.5;
    let green = ((percentage + 0.33) * 2.0 * pi).sin() * 0.5 + 0.5;
    let blue = ((percentage + 0.67) * 2.0 * pi).sin() * 0.5 + 0.5;
    [
        (red * 255.0) as u8,
        (green * 255.0) as u8,
        (blue * 255.0) as u8,
    ]
}

fn main() {
    let mut nav = Navigator::new();

    println!("Creating rainbow effect!");
    loop {
        let steps = 1000;
        for i in 0..=steps {
            let ratio = i as f32 / steps as f32;
            nav.set_neopixel(&[color_from_sine(ratio)]);
            sleep(Duration::from_millis(10));
        }
    }
}
