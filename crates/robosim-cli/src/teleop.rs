//! `robosim teleop`: crossterm raw-mode TUI with an ASCII map, a lidar strip,
//! and WASD/arrow keys driving the robot at ~30 Hz draw / 100 Hz physics.

use anyhow::Result;
use clap::Args;
use crossterm::event::{self, Event, KeyCode};
use crossterm::terminal::{disable_raw_mode, enable_raw_mode};
use crossterm::{cursor, execute, terminal};
use robosim_core::{Simulator, World};
use robosim_render::{AsciiBackend, RenderBackend, RenderOutput};
use robosim_ros2::{connect_or_fallback, Bridge};
use std::io::Write;
use std::time::{Duration, Instant};

/// `robosim teleop` arguments.
#[derive(Args)]
pub struct TeleopArgs {
    /// rosbridge websocket URL.
    #[arg(long)]
    pub rosbridge: Option<String>,
    /// Map width in columns.
    #[arg(long, default_value_t = 100)]
    pub cols: u32,
    /// Map height in rows.
    #[arg(long, default_value_t = 30)]
    pub rows: u32,
}

const V_STEP: f32 = 0.1;
const W_STEP: f32 = 0.2;
const PHYS_DT: f64 = 0.01; // 100 Hz physics
const DRAW_PERIOD: Duration = Duration::from_millis(33); // ~30 Hz

struct RawGuard;
impl Drop for RawGuard {
    fn drop(&mut self) {
        let _ = disable_raw_mode();
        let _ = execute!(std::io::stdout(), cursor::Show);
    }
}

pub fn teleop(args: TeleopArgs) -> Result<()> {
    let transport = connect_or_fallback(args.rosbridge.as_deref());
    let mut bridge = Bridge::new(transport);
    let mut sim = Simulator::new(World::demo_scene());
    sim.clock.dt = PHYS_DT;

    let mut v = 0.0f32;
    let mut omega = 0.0f32;

    enable_raw_mode()?;
    let _guard = RawGuard;
    let mut out = std::io::stdout();
    execute!(out, terminal::EnterAlternateScreen, cursor::Hide)?;

    let mut ascii = AsciiBackend::new(args.cols as usize, args.rows as usize);
    let mut last_draw = Instant::now() - DRAW_PERIOD;
    let mut physics_acc = Duration::ZERO;
    let mut last = Instant::now();
    let mut status = String::new();

    'outer: loop {
        // Input (non-blocking).
        while event::poll(Duration::ZERO)? {
            if let Event::Key(k) = event::read()? {
                match k.code {
                    KeyCode::Char('q') | KeyCode::Esc => break 'outer,
                    KeyCode::Char('w') | KeyCode::Up => v += V_STEP,
                    KeyCode::Char('s') | KeyCode::Down => v -= V_STEP,
                    KeyCode::Char('a') | KeyCode::Left => omega += W_STEP,
                    KeyCode::Char('d') | KeyCode::Right => omega -= W_STEP,
                    KeyCode::Char(' ') => {
                        v = 0.0;
                        omega = 0.0;
                    }
                    _ => {}
                }
            }
        }
        v = v.clamp(-1.0, 1.0);
        omega = omega.clamp(-3.0, 3.0);
        sim.set_twist(0, v, omega);

        // Physics at fixed dt, catching up to wall time.
        let now = Instant::now();
        physics_acc += now - last;
        last = now;
        while physics_acc >= Duration::from_secs_f64(PHYS_DT) {
            sim.step();
            physics_acc -= Duration::from_secs_f64(PHYS_DT);
        }

        if now - last_draw >= DRAW_PERIOD {
            last_draw = now;
            let frame = sim.snapshot();
            bridge.publish_frame(&frame);

            ascii.begin_frame(args.cols, args.rows);
            ascii.draw_scene(&frame, &sim.world);
            let map = match ascii.end_frame() {
                RenderOutput::Text(s) => s,
                _ => String::new(),
            };
            // One-character-tall lidar strip: nearest hit per ~fov bucket.
            let strip = lidar_strip(&frame.scan, args.cols as usize);
            let r = &frame.robots[0];
            status = format!(
                "t={:6.2} pose=({:+.2},{:+.2},{:+.2}) v={:+.2} omega={:+.2} | WASD/arrows drive, space stop, q quit",
                frame.time,
                r.pose.position.x,
                r.pose.position.y,
                r.pose.heading,
                r.v,
                r.omega
            );
            execute!(out, cursor::MoveTo(0, 0))?;
            write!(out, "{map}{strip}\r\n{status}\r\n")?;
            out.flush()?;
        }
        std::thread::sleep(Duration::from_millis(2));
    }
    execute!(out, terminal::LeaveAlternateScreen)?;
    println!("teleop ended: {status}");
    Ok(())
}

fn lidar_strip(scan: &robosim_core::LaserScan, cols: usize) -> String {
    let mut s = String::with_capacity(cols);
    for c in 0..cols {
        let idx = c * scan.ranges.len().max(1) / cols;
        let ch = match scan.ranges.get(idx).copied().unwrap_or(f32::INFINITY) {
            r if !r.is_finite() => ' ',
            r if r < 0.5 => '#',
            r if r < 1.5 => '*',
            r if r < 3.0 => '+',
            _ => '.',
        };
        s.push(ch);
    }
    s
}
