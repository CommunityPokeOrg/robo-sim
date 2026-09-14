# robo-sim

A lightweight, cross-platform robotics simulation framework proof-of-concept written
in pure Rust. robo-sim simulates differential-drive robots in a 2D world with analytic
physics, a lidar and a software camera, and bridges the simulated data to ROS 2 over
rosbridge or a JSONL relay — no C libraries, no GPU required.

## Quickstart

```sh
cargo build --workspace
cargo run -p robosim-cli -- run --steps 200 --jsonl -
cargo run -p robosim-cli -- teleop
cargo run -p robosim-cli -- render --out frame.ppm
```

## Documentation

- [Architecture](docs/architecture.md)
- [Roadmap](docs/roadmap.md)
- [ROS 2 integration](ros2/README.md)

## License

MIT — see [LICENSE](LICENSE).
