use criterion::{criterion_group, criterion_main, Criterion};
use navigator_rs::{AdcChannel, Navigator, UserLed};

fn navigator_benchmark(c: &mut Criterion) {
    #[macro_export]
    macro_rules! bench {
    ($bench_fn:ident($($arg:tt)*)) => {
        let mut nav = Navigator::new();
        c.bench_function(stringify!($bench_fn), |b| b.iter(|| nav.$bench_fn($($arg)*)));
    }}

    bench!(read_adc(AdcChannel::Ch0));
    bench!(read_all());

    bench!(read_accel());
    bench!(read_gyro());
    bench!(read_mag());

    bench!(read_pressure());
    bench!(read_temperature());

    bench!(read_adc_all());

    // Benchmark Outputs
    bench!(set_pwm_enable(false));
    bench!(set_pwm_duty_cycle(0, 0.1));
    bench!(set_pwm_frequency(60.0));
    bench!(set_neopixel(&[[0, 0, 0]]));
    bench!(set_led(UserLed::Led1, false));
    bench!(set_led_toggle(UserLed::Led1));
    bench!(get_led(UserLed::Led1));
    bench!(read_leak());
}

criterion_group!(benches, navigator_benchmark);
criterion_main!(benches);
