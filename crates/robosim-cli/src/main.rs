//! `robosim` CLI: `run` (headless sim + ROS 2 bridge), `teleop` (ASCII TUI),
//! `render` (headless PPM snapshot).

mod run;
mod teleop;

use anyhow::Result;
use clap::{Parser, Subcommand};

/// robo-sim: lightweight cross-platform robotics simulation POC.
#[derive(Parser)]
#[command(name = "robosim", version, about)]
struct Cli {
    #[command(subcommand)]
    cmd: Cmd,
}

#[derive(Subcommand)]
enum Cmd {
    /// Run the simulation headlessly, publishing frames via a transport.
    Run(run::RunArgs),
    /// Interactive keyboard teleop TUI (ASCII map + lidar strip).
    Teleop(teleop::TeleopArgs),
    /// Run the demo and write a top-down PPM frame.
    Render(RenderArgs),
}

/// `robosim render` arguments.
#[derive(clap::Args)]
struct RenderArgs {
    /// Output PPM path.
    #[arg(long, default_value = "out/frame.ppm")]
    out: std::path::PathBuf,
    /// Simulation steps before rendering.
    #[arg(long, default_value_t = 300)]
    steps: u64,
    /// Image width.
    #[arg(long, default_value_t = 480)]
    width: u32,
    /// Image height.
    #[arg(long, default_value_t = 480)]
    height: u32,
}

fn main() -> Result<()> {
    let cli = Cli::parse();
    match cli.cmd {
        Cmd::Run(args) => run::run(args),
        Cmd::Teleop(args) => teleop::teleop(args),
        Cmd::Render(args) => render(args),
    }
}

fn render(args: RenderArgs) -> Result<()> {
    use robosim_render::{HeadlessBackend, RenderBackend};
    let mut sim = robosim_core::Simulator::new(robosim_core::World::demo_scene());
    sim.set_twist(0, 0.4, 0.3);
    sim.step_n(args.steps);
    let frame = sim.snapshot();

    let mut backend = HeadlessBackend::default();
    backend.begin_frame(args.width, args.height);
    backend.draw_scene(&frame, &sim.world);
    backend.end_frame();

    if let Some(dir) = args.out.parent() {
        std::fs::create_dir_all(dir)?;
    }
    backend.save_ppm(&args.out)?;
    println!(
        "wrote {} ({}x{})",
        args.out.display(),
        args.width,
        args.height
    );
    Ok(())
}
