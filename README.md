# robo-sim

[![CI](https://github.com/CommunityPokeOrg/robo-sim/actions/workflows/ci.yml/badge.svg)](https://github.com/CommunityPokeOrg/robo-sim/actions/workflows/ci.yml)
[![License: MIT](https://img.shields.io/badge/license-MIT-blue.svg)](LICENSE)

A lightweight, cross-platform robotics simulation framework **proof of concept**,
written in pure Rust.

robo-sim simulates differential-drive robots in a 2D arena with deterministic
fixed-step physics, an analytic lidar and a mock camera, and bridges the
simulated data to ROS 2 — over rosbridge or an `rclpy` relay — while running
happily when ROS 2 is not installed. It builds and tests on macOS, Linux and
Windows with a single `cargo build`: no C libraries, no GPU, no ROS toolchain
required.

This is *not* a Gazebo replacement. See [docs/roadmap.md](docs/roadmap.md) for
an honest comparison and the list of limitations.

## Quickstart

Requires a stable Rust toolchain (<https://rustup.rs>).

```sh
git clone https://github.com/CommunityPokeOrg/robo-sim
cd robo-sim
cargo build --workspace
cargo test --workspace

# Headless run, 1000 steps at 100 Hz, printing ROS-style JSON lines to stdout
cargo run -p robosim-cli -- run --steps 1000 --twist 0.5,0.3 --jsonl -

# Drive the robot in your terminal (WASD / arrows, space to stop, q to quit)
cargo run -p robosim-cli -- teleop

# Render a top-down frame of the demo scene to a PPM image
cargo run -p robosim-cli -- render --out frame.ppm --steps 300
```

`robosim run --help` lists every flag (timestep, camera rate, rosbridge URL,
`/cmd_vel` over stdin, …).

## What's inside

| Crate | Purpose |
| --- | --- |
| `robosim-core` | Clock, 2D geometry, world, diff-drive robot, pluggable physics, lidar + camera sensors, `Simulator` → `Frame` |
| `robosim-render` | `RenderBackend` trait; headless RGB rasteriser and ASCII backend; GPU backend stub |
| `robosim-ros2` | ROS 2 message structs, `Frame` → topic conversion, `Null` / `JSONL` / rosbridge transports |
| `robosim-cli` | `robosim` binary: `run`, `teleop`, `render` |
| `ros2/` | `robosim_relay.py` (rclpy relay) and ROS 2 setup notes |

Published topics: `/clock`, `/tf`, `/joint_states`, `/scan`, `/camera/image_raw`,
`/odom`. Subscribed: `/cmd_vel`.

## ROS 2

```sh
# Option A: rosbridge_suite
ros2 launch rosbridge_server rosbridge_websocket_launch.xml
cargo run -p robosim-cli -- run --rosbridge ws://localhost:9090 --steps 0 --realtime

# Option B: rclpy relay (no rosbridge needed)
source /opt/ros/<distro>/setup.bash
python3 ros2/robosim_relay.py --twist 0.4,0.2
```

If rosbridge is unreachable the sim prints one warning and continues without
ROS 2. Details, including the planned Zenoh/DDS path, in
[ros2/README.md](ros2/README.md).

## Examples

```sh
cargo run -p robosim-core --example headless_drive
cargo run -p robosim-core --example lidar_scan
cargo run -p robosim-ros2 --example rosbridge_publish
```

## Documentation

- [Architecture](docs/architecture.md) — module boundaries, timing, physics,
  sensors, the cross-platform rendering bridge (Metal / Vulkan / DirectX) and
  ROS 2 integration.
- [Limitations & roadmap](docs/roadmap.md)
- [ROS 2 integration](ros2/README.md)

## Development

```sh
cargo fmt --all -- --check
cargo clippy --workspace --all-targets -- -D warnings
cargo test --workspace
```

CI runs the same on `ubuntu-latest`, `macos-latest` and `windows-latest`.

## License

MIT — see [LICENSE](LICENSE).
