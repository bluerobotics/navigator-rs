use criterion::{criterion_group, criterion_main, Criterion};
use navigator_rs::{AdcChannel, Navigator, PwmChannel, UserLed};

fn navigator_benchmark(c: &mut Criterion) {
    #[macro_export]
    macro_rules! bench {
    ($bench_fn:ident($($arg:tt)*)) => {
        let mut nav = Navigator::new();
        nav.init();
        c.bench_function(stringify!($bench_fn), |b| b.iter(|| nav.$bench_fn($($arg)*)));
    }}

    // Navigator creation benchmark
    c.bench_function("new", |b| b.iter(|| Navigator::new()));

    // Benchmark Inputs
    bench!(init());

    bench!(read_adc(AdcChannel::Ch0));
    bench!(read_adc_all());
    bench!(read_all());

    bench!(read_accel());
    bench!(read_gyro());
    bench!(read_mag());

    bench!(read_pressure());
    bench!(read_temperature());

    bench!(read_all());

    // Benchmark Outputs
    bench!(set_pwm_enable(false));
    bench!(get_pwm_enable());
    bench!(set_pwm_channel_value(PwmChannel::Ch1, 100));
    bench!(set_pwm_freq_hz(60.0));
    bench!(set_pwm_freq_prescale(100));

    bench!(set_neopixel(&[[0, 0, 0]]));

    bench!(set_led(UserLed::Led1, false));
    bench!(set_led_toggle(UserLed::Led1));
    bench!(get_led(UserLed::Led1));
    bench!(read_leak());
}

criterion_group!(benches, navigator_benchmark);
criterion_main!(benches);
