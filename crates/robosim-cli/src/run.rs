//! `robosim run`: headless simulation loop publishing frames.

use anyhow::Result;
use clap::Args;
use robosim_core::{Simulator, World};
use robosim_ros2::{connect_or_fallback, Bridge, JsonlTransport, Transport};
use std::io::{BufRead, BufReader, Write};
use std::sync::mpsc::{channel, Receiver};
use std::time::{Duration, Instant};

/// `robosim run` arguments.
#[derive(Args)]
pub struct RunArgs {
    /// Number of physics steps (0 = run until interrupted).
    #[arg(long, default_value_t = 1000)]
    pub steps: u64,
    /// Pace the loop to wall-clock time (one step per `dt` seconds).
    #[arg(long)]
    pub realtime: bool,
    /// Timestep in seconds.
    #[arg(long, default_value_t = 0.01)]
    pub dt: f64,
    /// rosbridge websocket URL, e.g. ws://localhost:9090.
    #[arg(long)]
    pub rosbridge: Option<String>,
    /// Write rosbridge-style JSON lines to PATH, or `-` for stdout.
    #[arg(long)]
    pub jsonl: Option<String>,
    /// Initial twist command as "v,omega" (m/s, rad/s).
    #[arg(long)]
    pub twist: Option<String>,
    /// Emit a camera frame every N steps (0 = off).
    #[arg(long, default_value_t = 10)]
    pub camera_every: u64,
    /// Read twist commands `{"v":..,"omega":..}` as JSON lines from stdin.
    #[arg(long)]
    pub cmd_stdin: bool,
}

fn parse_twist(s: &str) -> Result<(f32, f32)> {
    let mut it = s.split(',');
    let v: f32 = it.next().unwrap_or("").trim().parse()?;
    let omega: f32 = it.next().unwrap_or("0").trim().parse()?;
    Ok((v, omega))
}

/// Spawn a thread reading `{"v":..,"omega":..}` lines from stdin.
fn cmd_stdin_channel() -> Receiver<(f32, f32)> {
    let (tx, rx) = channel();
    std::thread::spawn(move || {
        for line in BufReader::new(std::io::stdin()).lines() {
            let Ok(line) = line else { break };
            let Ok(v) = serde_json::from_str::<serde_json::Value>(&line) else {
                continue;
            };
            let (Some(v_), Some(o_)) = (
                v.get("v").and_then(|x| x.as_f64()),
                v.get("omega").and_then(|x| x.as_f64()),
            ) else {
                continue;
            };
            if tx.send((v_ as f32, o_ as f32)).is_err() {
                break;
            }
        }
    });
    rx
}

/// Pick the transport: rosbridge > jsonl > null.
fn build_transport(args: &RunArgs) -> Result<Box<dyn Transport>> {
    if args.rosbridge.is_some() {
        return Ok(connect_or_fallback(args.rosbridge.as_deref()));
    }
    if let Some(path) = &args.jsonl {
        if path == "-" {
            return Ok(Box::new(JsonlTransport::new(std::io::stdout())));
        }
        let f = std::fs::File::create(path)?;
        return Ok(Box::new(JsonlTransport::new(std::io::BufWriter::new(f))));
    }
    Ok(Box::new(robosim_ros2::NullTransport::default()))
}

pub fn run(args: RunArgs) -> Result<()> {
    // --jsonl - writes protocol lines to stdout; keep the summary on stderr there.
    let summary_to_stderr = args.jsonl.as_deref() == Some("-");

    let transport = build_transport(&args)?;
    let mut bridge = Bridge::new(transport);

    let mut sim = Simulator::new(World::demo_scene());
    sim.clock.dt = args.dt;
    sim.camera_every_n_steps = args.camera_every;

    if let Some(t) = &args.twist {
        let (v, o) = parse_twist(t)?;
        sim.set_twist(0, v, o);
    }
    let cmd_rx = args.cmd_stdin.then(cmd_stdin_channel);

    let start = Instant::now();
    while args.steps == 0 || sim.clock.step < args.steps {
        if args.realtime {
            let target = start + Duration::from_secs_f64(sim.clock.time_s);
            if let Some(wait) = target.checked_duration_since(Instant::now()) {
                std::thread::sleep(wait);
            }
        }
        if let Some(rx) = &cmd_rx {
            while let Ok((v, o)) = rx.try_recv() {
                sim.set_twist(0, v, o);
            }
        }
        if let Some(t) = bridge.transport.poll_twist() {
            sim.set_twist(0, t.linear.x as f32, t.angular.z as f32);
        }
        sim.step();
        bridge.publish_frame(&sim.snapshot());
    }
    std::io::stdout().flush().ok();

    let r = &sim.world.robots[0];
    let summary = format!(
        "done: steps={} t={:.2}s pose=({:.3},{:.3},{:.3}) v={:.3} omega={:.3} publications={}",
        sim.clock.step,
        sim.clock.time_s,
        r.state.pose.position.x,
        r.state.pose.position.y,
        r.state.pose.heading,
        r.state.v,
        r.state.omega,
        bridge.transport.published_count(),
    );
    if summary_to_stderr {
        eprintln!("{summary}");
    } else {
        println!("{summary}");
    }
    Ok(())
}
