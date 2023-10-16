#![feature(test)]

extern crate test;


use test::Bencher;


use navigator_rs::Navigator;
// use std::thread::sleep;
// use std::time::Duration;

// fn main() {
//     println!("Creating your navigator module!");
//     let mut nav = Navigator::new();

//     println!("Setting up your navigator, ahoy!");
//     nav.init();

//     loop {
//         nav.set_pwm_channel_value(PwmChannel::All, 0);
//         println!("{:#?}", nav.fmt_debug());
//         sleep(Duration::from_millis(1000));
//     }
// }


#[bench]
fn init(b: &mut Bencher) {
    b.iter(|| {
        let mut nav = Navigator::new();
        let _ = nav.init();
    });
}

// #[bench]
// fn bench_fib_20(b: &mut Bencher) {
//     b.iter(|| {
//         let _ = fib(20);
//     });
// }
