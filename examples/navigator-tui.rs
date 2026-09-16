//! Terminal dashboard for the Navigator board.
//!
//! Shows every sensor and drives the user LEDs, the PWM outputs and the RGB
//! LED from a single screen. The Raspberry Pi model comes from the device
//! tree, `--pi` overrides it.
//!
//! ```shell
//! cargo run --example navigator-tui -- --navigator v3
//! ```

use std::io;
use std::time::{Duration, Instant};

use clap::{Parser, ValueEnum};
use navigator_rs::{
    AxisData, Navigator, NavigatorVersion, Peripherals, PiVersion, SensorData, UserLed,
};
use ratatui::crossterm::event::{self, Event, KeyCode, KeyEventKind};
use ratatui::layout::{Constraint, Layout, Rect};
use ratatui::style::{Color, Style, Stylize};
use ratatui::text::{Line, Span};
use ratatui::widgets::{Block, Gauge, Paragraph};
use ratatui::{DefaultTerminal, Frame};

/// How often the sensors are read. A full set costs a few tens of
/// milliseconds over I2C and SPI, so this is not free.
const REFRESH: Duration = Duration::from_millis(200);
/// Full scale of the ADS1115 as this crate configures it.
const ADC_FULL_SCALE: f32 = 4.096;
/// Range the PCA9685 can be asked for.
const PWM_FREQUENCY_LIMITS: (f32, f32) = (24.0, 1526.0);

/// Dim on purpose, the RGB LED is bright enough to hurt at full scale.
const COLORS: [(&str, [u8; 3]); 7] = [
    ("off", [0, 0, 0]),
    ("red", [40, 0, 0]),
    ("green", [0, 40, 0]),
    ("blue", [0, 0, 40]),
    ("yellow", [40, 40, 0]),
    ("cyan", [0, 40, 40]),
    ("magenta", [40, 0, 40]),
];

#[derive(Parser)]
#[command(about = "Control and visualize a Navigator board from the terminal")]
struct Cli {
    /// Navigator board revision, probed over I2C when not given
    #[arg(short, long, value_enum)]
    navigator: Option<Board>,

    /// Raspberry Pi model, taken from the device tree when not given
    #[arg(short, long, value_enum)]
    pi: Option<Pi>,

    /// Number of LEDs on the RGB strip
    #[arg(long, default_value_t = 1)]
    rgb_led_strip_size: usize,
}

#[derive(Debug, Clone, Copy, PartialEq, Eq, ValueEnum)]
enum Board {
    V1,
    V2,
    V3,
}

#[derive(Debug, Clone, Copy, ValueEnum)]
enum Pi {
    Pi4,
    Pi5,
}

impl From<Board> for NavigatorVersion {
    fn from(board: Board) -> Self {
        match board {
            Board::V1 => NavigatorVersion::V1,
            Board::V2 => NavigatorVersion::V2,
            Board::V3 => NavigatorVersion::V3,
        }
    }
}

impl From<Pi> for PiVersion {
    fn from(pi: Pi) -> Self {
        match pi {
            Pi::Pi4 => PiVersion::Pi4,
            Pi::Pi5 => PiVersion::Pi5,
        }
    }
}

/// Resolves one setting and says where the answer came from, which is worth
/// showing: a wrong board or Pi silently picks the wrong buses.
fn resolve<T, U>(requested: Option<T>, detected: Option<U>) -> (U, &'static str)
where
    T: Into<U>,
    U: Default,
{
    match (requested, detected) {
        (Some(asked), _) => (asked.into(), "asked for"),
        (None, Some(found)) => (found, "detected"),
        (None, None) => (U::default(), "not found, assumed"),
    }
}

/// Everything the screen shows, kept apart from the hardware so it can be
/// rendered without a board.
#[derive(Default)]
struct View {
    title: String,
    sensors: SensorData,
    /// The IIS2MDC, only wired on V3. The primary answer already comes from
    /// the MMC5983MA through `read_mag`.
    second_magnetometer: Option<AxisData>,
    leds: [bool; 3],
    pwm_enabled: bool,
    pwm_frequency: f32,
    duty_cycle: f32,
    color: usize,
}

struct App {
    navigator: Navigator,
    board: NavigatorVersion,
    view: View,
    last_read: Instant,
}

impl App {
    fn new(cli: &Cli) -> Self {
        let (board, board_source) = resolve(cli.navigator, NavigatorVersion::detect());
        let (pi, pi_source) = resolve(cli.pi, PiVersion::detect());
        let navigator = Navigator::create()
            .with_navigator(board)
            .with_pi(pi)
            .with_rgb_led_strip_size(cli.rgb_led_strip_size)
            .build();

        Self {
            navigator,
            board,
            view: View {
                title: format!(" Navigator {board:?} ({board_source}) on {pi:?} ({pi_source}) "),
                pwm_frequency: 50.0,
                ..Default::default()
            },
            last_read: Instant::now() - REFRESH,
        }
    }

    fn read_sensors(&mut self) {
        self.view.sensors = self.navigator.read_all();
        if matches!(self.board, NavigatorVersion::V3) {
            self.view.second_magnetometer =
                Some(self.navigator.read_mag_from(Peripherals::Iis2mdc));
        }

        self.last_read = Instant::now();
    }

    fn toggle_led(&mut self, index: usize) {
        let led = [UserLed::Led1, UserLed::Led2, UserLed::Led3][index];
        self.view.leds[index] = !self.view.leds[index];
        self.navigator.set_led(led, self.view.leds[index]);
    }

    fn set_pwm_enabled(&mut self, enabled: bool) {
        self.view.pwm_enabled = enabled;
        self.navigator.set_pwm_enable(enabled);
    }

    fn set_pwm_frequency(&mut self, frequency: f32) {
        self.view.pwm_frequency = frequency.clamp(PWM_FREQUENCY_LIMITS.0, PWM_FREQUENCY_LIMITS.1);
        self.navigator.set_pwm_frequency(self.view.pwm_frequency);
    }

    fn set_duty_cycle(&mut self, duty_cycle: f32) {
        self.view.duty_cycle = duty_cycle.clamp(0.0, 1.0);
        self.navigator.set_duty_cycle_all(self.view.duty_cycle);
    }

    fn next_color(&mut self) {
        self.view.color = (self.view.color + 1) % COLORS.len();
        self.navigator.set_neopixel(&[COLORS[self.view.color].1]);
    }
}

fn main() -> io::Result<()> {
    let cli = Cli::parse();
    let mut app = App::new(&cli);
    ratatui::run(|terminal| run(terminal, &mut app))
}

fn run(terminal: &mut DefaultTerminal, app: &mut App) -> io::Result<()> {
    loop {
        if app.last_read.elapsed() >= REFRESH {
            app.read_sensors();
        }

        terminal.draw(|frame| draw(frame, &app.view))?;

        // Whatever is left of the refresh period goes to the keyboard, so
        // input never waits on a full sensor round
        let timeout = REFRESH.saturating_sub(app.last_read.elapsed());
        if !event::poll(timeout)? {
            continue;
        }
        if let Event::Key(key) = event::read()? {
            if key.kind != KeyEventKind::Press {
                continue;
            }
            match key.code {
                KeyCode::Char('q') | KeyCode::Esc => return Ok(()),
                KeyCode::Char('1') => app.toggle_led(0),
                KeyCode::Char('2') => app.toggle_led(1),
                KeyCode::Char('3') => app.toggle_led(2),
                KeyCode::Char('p') => app.set_pwm_enabled(!app.view.pwm_enabled),
                KeyCode::Char('c') => app.next_color(),
                KeyCode::Char('-') => app.set_pwm_frequency(app.view.pwm_frequency - 10.0),
                KeyCode::Char('+') | KeyCode::Char('=') => {
                    app.set_pwm_frequency(app.view.pwm_frequency + 10.0)
                }
                KeyCode::Left => app.set_duty_cycle(app.view.duty_cycle - 0.05),
                KeyCode::Right => app.set_duty_cycle(app.view.duty_cycle + 0.05),
                _ => {}
            }
        }
    }
}

fn draw(frame: &mut Frame, view: &View) {
    let window = Block::bordered().title(view.title.clone().bold());
    let inner = window.inner(frame.area());
    frame.render_widget(window, frame.area());

    let [left, right] =
        Layout::horizontal([Constraint::Fill(1), Constraint::Length(34)]).areas(inner);

    // Motion takes the slack, the two above it need a known number of lines
    let [status, adc, motion] = Layout::vertical([
        Constraint::Length(5),
        Constraint::Length(6),
        Constraint::Min(7),
    ])
    .areas(left);

    draw_status(frame, status, view);
    draw_adc(frame, adc, view);
    draw_motion(frame, motion, view);

    let [controls, help] =
        Layout::vertical([Constraint::Length(12), Constraint::Fill(1)]).areas(right);

    draw_controls(frame, controls, view);
    draw_help(frame, help);
}

fn draw_status(frame: &mut Frame, area: Rect, view: &View) {
    let (leak, leak_style) = if view.sensors.leak {
        ("LEAK DETECTED", Style::new().fg(Color::Red).bold())
    } else {
        ("dry", Style::new().fg(Color::Green))
    };

    let text = vec![
        Line::from(format!("Temperature  {:>8.2} °C", view.sensors.temperature)),
        Line::from(format!("Pressure     {:>8.2} kPa", view.sensors.pressure)),
        Line::from(vec!["Leak         ".into(), Span::styled(leak, leak_style)]),
    ];
    frame.render_widget(
        Paragraph::new(text).block(Block::bordered().title(" Status ")),
        area,
    );
}

fn draw_adc(frame: &mut Frame, area: Rect, view: &View) {
    let block = Block::bordered().title(" ADC ");
    let inner = block.inner(area);
    frame.render_widget(block, area);

    let rows = Layout::vertical([Constraint::Length(1); 4]).split(inner);
    for (channel, row) in rows.iter().enumerate() {
        let voltage = view.sensors.adc.get(channel).copied().unwrap_or(0.0);
        frame.render_widget(
            Gauge::default()
                .ratio(f64::from(voltage / ADC_FULL_SCALE).clamp(0.0, 1.0))
                .gauge_style(Style::new().fg(Color::Green))
                .label(format!("ch{channel}  {voltage:.3} V")),
            *row,
        );
    }
}

fn draw_motion(frame: &mut Frame, area: Rect, view: &View) {
    let mut text = vec![
        Line::from(format!("{:>19}{:>8}{:>8}", "x", "y", "z").italic()),
        axis_line("Accel", &view.sensors.accelerometer, "m/s²"),
        axis_line("Gyro", &view.sensors.gyro, "rad/s"),
        axis_line("Mag", &view.sensors.magnetometer, "µT"),
    ];
    // V3 carries two of them, the primary already answered above
    if let Some(second) = &view.second_magnetometer {
        text[3] = axis_line("Mag MMC5983", &view.sensors.magnetometer, "µT");
        text.push(axis_line("Mag IIS2MDC", second, "µT"));
    }

    frame.render_widget(
        Paragraph::new(text).block(Block::bordered().title(" Motion ")),
        area,
    );
}

fn axis_line(name: &str, axis: &AxisData, unit: &str) -> Line<'static> {
    Line::from(format!(
        "{name:<11}{:>8.3}{:>8.3}{:>8.3} {unit}",
        axis.x, axis.y, axis.z
    ))
}

fn draw_controls(frame: &mut Frame, area: Rect, view: &View) {
    let block = Block::bordered().title(" Controls ");
    let inner = block.inner(area);
    frame.render_widget(block, area);

    let [leds, pwm, duty, color] = Layout::vertical([
        Constraint::Length(2),
        Constraint::Length(3),
        Constraint::Length(2),
        Constraint::Fill(1),
    ])
    .areas(inner);

    let states: Vec<String> = view
        .leds
        .iter()
        .enumerate()
        .map(|(index, on)| format!("[{}] {}", index + 1, if *on { "on " } else { "off" }))
        .collect();
    frame.render_widget(
        Paragraph::new(vec![
            Line::from("User LEDs".bold()),
            Line::from(states.join("  ")),
        ]),
        leds,
    );

    let pwm_state = if view.pwm_enabled {
        "enabled"
    } else {
        "disabled"
    };
    frame.render_widget(
        Paragraph::new(vec![
            Line::from("PWM".bold()),
            Line::from(format!("output     {pwm_state}")),
            Line::from(format!("frequency  {:.0} Hz", view.pwm_frequency)),
        ]),
        pwm,
    );

    let duty_rows = Layout::vertical([Constraint::Length(1); 2]).split(duty);
    frame.render_widget(
        Paragraph::new(Line::from("Duty cycle".bold())),
        duty_rows[0],
    );
    frame.render_widget(
        Gauge::default()
            .ratio(f64::from(view.duty_cycle))
            .gauge_style(Style::new().fg(Color::Magenta)),
        duty_rows[1],
    );

    let (name, rgb) = COLORS[view.color];
    frame.render_widget(
        Paragraph::new(vec![
            Line::from("RGB LED".bold()),
            Line::from(format!("{name}  {rgb:?}")),
        ]),
        color,
    );
}

fn draw_help(frame: &mut Frame, area: Rect) {
    let keys = [
        ("1 2 3", "toggle user LEDs"),
        ("p", "toggle PWM output"),
        ("+ -", "PWM frequency"),
        ("← →", "duty cycle, all channels"),
        ("c", "next RGB colour"),
        ("q", "quit"),
    ];
    let text: Vec<Line> = keys
        .iter()
        .map(|(key, action)| Line::from(format!("{key:<7} {action}")))
        .collect();

    frame.render_widget(
        Paragraph::new(text).block(Block::bordered().title(" Keys ")),
        area,
    );
}

#[cfg(test)]
mod tests {
    use super::*;
    use ratatui::backend::TestBackend;
    use ratatui::Terminal;

    /// Run with `cargo test --example navigator-tui`.
    ///
    /// The layouts destructure a fixed number of areas, which panics as soon
    /// as a constraint list and its pattern stop agreeing.
    #[test]
    fn every_panel_fits_the_terminal() {
        let mut view = View {
            title: " Navigator V3 on Pi5 (detected) ".to_string(),
            pwm_frequency: 50.0,
            ..Default::default()
        };
        view.sensors.adc = vec![0.0, 1.5, 3.3, 4.2];

        // Both the single magnetometer boards and V3, at a cramped terminal,
        // the usual one and a wide one
        for second_magnetometer in [None, Some(AxisData::default())] {
            view.second_magnetometer = second_magnetometer;
            for (width, height) in [(40, 16), (80, 24), (200, 60)] {
                let mut terminal = Terminal::new(TestBackend::new(width, height)).unwrap();
                terminal.draw(|frame| draw(frame, &view)).unwrap();
            }
        }
    }
}
