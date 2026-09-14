# robo-sim architecture

robo-sim is a proof-of-concept for a lightweight, cross-platform robotics
simulator. The goal of this document is to fix the module boundaries and the
contracts between them so the POC can grow without a rewrite. Everything
described as *implemented* exists in this repository today; everything marked
*planned* is a design commitment with no code behind it yet.

## 1. Design goals

| Goal | Consequence |
| --- | --- |
| Runs on macOS, Linux and Windows from one `cargo build` | Pure-Rust dependencies only; no system libraries in the core. |
| Deterministic and testable | Fixed-timestep simulation, no wall-clock in the core, seeded noise. |
| Headless first | Rendering and ROS 2 are optional adapters, never dependencies of the core. |
| Small enough to read in an afternoon | ~3k lines across four crates; one responsibility per crate. |

## 2. Crate / module map

```
┌─────────────────────────────────────────────────────────────────┐
│ robosim-cli   (binary `robosim`: run / teleop / render)         │
└──────┬───────────────────────┬─────────────────────────┬────────┘
       │                       │                         │
┌──────▼──────────┐  ┌─────────▼─────────────┐  ┌────────▼─────────┐
│ robosim-render  │  │ robosim-ros2          │  │ (future plugins) │
│ RenderBackend   │  │ Frame → ROS msgs      │  │ controllers,     │
│  ├ Headless     │  │ Transport             │  │ scripting        │
│  ├ Ascii        │  │  ├ Null               │  │                  │
│  └ Gpu (planned)│  │  ├ Jsonl              │  └──────────────────┘
└──────┬──────────┘  │  └ Rosbridge (ws)     │
       │             └─────────┬─────────────┘
       │                       │
┌──────▼───────────────────────▼──────────────────────────────────┐
│ robosim-core                                                    │
│  time · geometry · world · robot · physics · sensors · sim      │
│  output type: `Frame` (serde)                                   │
└─────────────────────────────────────────────────────────────────┘
```

Dependency rule: arrows only point down. `robosim-core` depends on nothing
else in the workspace; adapters depend on core; the binary depends on all.

### 2.1 `robosim-core`

| Module | Responsibility | Key types |
| --- | --- | --- |
| `time` | Fixed-step simulation clock | `SimClock { time_s, step, dt }` |
| `geometry` | 2D primitives and ray tests | `Pose2`, `Aabb`, `ray_aabb`, `ray_circle` |
| `world` | Static scene + robots | `World`, `Obstacle::{Box, Circle}`, `World::demo_scene()` |
| `robot` | Differential-drive model | `DiffDriveParams`, `DiffDriveState`, `WheelCommand`, `DiffDriveRobot` |
| `physics` | Pluggable integrator + collision | `PhysicsBackend` trait, `SimplePhysics` |
| `sensors` | Sensor trait and mock sensors | `Sensor`, `Lidar2D → LaserScan`, `MockCamera → Image` |
| `sim` | Driver that ties it together | `Simulator`, `Frame`, `RobotSnapshot` |

`Frame` is the single hand-off type out of the core: a serde-serialisable
snapshot of sim time, every robot's pose/velocities/joints, one lidar scan and
an optional camera image. Renderers and bridges consume `Frame`s and never
reach into the simulator.

### 2.2 `robosim-render`

`RenderBackend` is a tiny immediate-mode contract:

```rust
trait RenderBackend {
    fn name(&self) -> &str;
    fn begin_frame(&mut self, width: u32, height: u32);
    fn draw_scene(&mut self, frame: &Frame, world: &World);
    fn end_frame(&mut self) -> RenderOutput; // Pixels | Text | None
}
```

Implemented: `HeadlessBackend` (software top-down rasteriser → RGB8 / PPM) and
`AsciiBackend` (character grid for the terminal teleop). `GpuBackendStub` only
reports which GPU API would be selected on the current OS.

### 2.3 `robosim-ros2`

Three layers, each independently testable without ROS 2 installed:

1. `msgs` — serde structs whose field names match the ROS 2 JSON wire format
   used by rosbridge (`std_msgs/Header`, `sensor_msgs/LaserScan`, …).
2. `convert` — `frame_to_messages(&Frame, &BridgeConfig) -> Vec<Publication>`.
3. `transport` — `Transport` trait with `advertise`, `publish`, `poll_twist`.
   Implementations: `NullTransport`, `JsonlTransport<W>`, `RosbridgeTransport`.

`Bridge` owns a `Box<dyn Transport>` and lazily advertises on first publish.
`connect_or_fallback(url)` is the graceful-degradation entry point used by the
CLI: if rosbridge is unreachable it warns once and returns `NullTransport`.

### 2.4 `robosim-cli`

Thin `clap` front-end. `run` is the headless batch mode, `teleop` a `crossterm`
raw-mode TUI, `render` writes a PPM. It is the only crate allowed to touch the
wall clock and the terminal.

## 3. Timing model

* The core is **fixed-timestep**: `Simulator::step()` advances exactly
  `clock.dt` (default 10 ms / 100 Hz). No wall-clock anywhere in the core.
* Sim time is `f64` seconds plus a `u64` step counter, so it never drifts and
  `to_sec_nanosec()` maps losslessly onto `builtin_interfaces/Time`.
* Real-time pacing is the caller's job. `teleop` uses an accumulator loop
  (catch up to wall time in whole `dt` steps, redraw at ~30 Hz); `run` steps as
  fast as possible.
* Sensors sample on demand (`snapshot()`), so sensor rate is decoupled from
  physics rate. The camera is additionally throttled by
  `camera_every_n_steps`.
* Planned: per-sensor rates and a `/clock`-driven `use_sim_time` contract for
  ROS 2 nodes are already representable; only scheduling code is missing.

## 4. Physics

`SimplePhysics` is a kinematic 2D backend, chosen because a POC needs
credible, debuggable motion far more than rigid-body fidelity:

* Differential drive: `v = r/2 (ωL + ωR)`, `ω = r/L (ωR − ωL)`; wheel speeds
  are clamped to `max_wheel_speed`; wheel angles integrate for `/joint_states`.
* Pose integration uses the exact circular-arc solution when `|ω| > ε`,
  straight-line otherwise (no Euler drift on turns).
* Collision: robot body is a circle. Circle–AABB (closest-point) and
  circle–circle overlaps are resolved by push-out along the contact normal, and
  the velocity component into the contact is removed. Arena bounds are
  enforced the same way.
* Deterministic: fixed iteration order, no RNG.

The `PhysicsBackend` trait is the seam for a real rigid-body engine (e.g. a
pure-Rust 2D/3D engine) without touching sensors, rendering or the bridge.

## 5. Sensors

`Sensor` is a pure function of `(World, robot, clock) → Output`.

* **Lidar2D** — analytic ray casting (slab test against AABBs, quadratic
  against circles). Configurable beam count, FOV, min/max range, mount offset
  and optional uniform range noise (seeded from the step index so runs stay
  deterministic). Misses are `f32::INFINITY`, matching ROS conventions.
* **MockCamera** — a CPU column ray-caster: one ray per image column, wall
  height shaded by distance, obstacle colour by type, flat floor/sky. Emits
  `rgb8`. It is intentionally *mock*: good enough to exercise the image
  pipeline (`sensor_msgs/Image`, relay, bandwidth), not a photoreal renderer.
* **Odometry** — derived from robot state (`OdomSample`), published as
  `/odom` with zeroed covariance placeholders.

Adding a sensor means implementing `Sensor`, adding a field to `Frame`, and
one `match` arm in `convert`.

## 6. Cross-platform rendering bridge

The rendering layer is deliberately split into a **backend-agnostic scene
description** (`Frame` + `World`) and **backends**. The current software
backends run everywhere with zero platform code. For a GPU path the plan is:

| Platform | Preferred API | Fallback | Surface |
| --- | --- | --- | --- |
| macOS | Metal | OpenGL 4.1 (deprecated by Apple, still ships) | `CAMetalLayer` via `winit` |
| Linux | Vulkan | OpenGL 3.3 / GLES 3 | X11 / Wayland via `winit` |
| Windows | DirectX 12 | Vulkan, DirectX 11 | HWND via `winit` |

Implementation strategy:

1. Use **wgpu** as the single portable GPU abstraction. It selects Metal on
   macOS, Vulkan (or GL) on Linux and DX12 (or Vulkan/DX11) on Windows at
   runtime, which matches the table above with one code path.
2. Put it behind a Cargo feature (`gpu`) in `robosim-render` so the default
   build — and CI — stays headless and dependency-light.
3. Windowing via `winit`; the same event loop hosts the teleop input so the
   TUI and GUI share a controller.
4. The GPU backend implements the same `RenderBackend` trait; the camera
   sensor can later be re-pointed at an off-screen GPU render target to replace
   the CPU column caster without changing `Frame`.

`GpuBackendStub::describe()` already reports the per-OS choice so downstream
code can be written against it today.

## 7. ROS 2 integration

Published topics (all from `frame_to_messages`):

| Topic | Type | Source |
| --- | --- | --- |
| `/clock` | `rosgraph_msgs/msg/Clock` | `SimClock` |
| `/tf` | `tf2_msgs/msg/TFMessage` | `odom→base_link`, `base_link→laser_frame`, `base_link→camera_link` |
| `/joint_states` | `sensor_msgs/msg/JointState` | wheel angles/velocities |
| `/scan` | `sensor_msgs/msg/LaserScan` | `Lidar2D` |
| `/camera/image_raw` | `sensor_msgs/msg/Image` | `MockCamera` |
| `/odom` | `nav_msgs/msg/Odometry` | robot state |

Subscribed: `/cmd_vel` (`geometry_msgs/msg/Twist`).

Transport options and their trade-offs:

| Path | Status | Needs ROS 2 on the sim host? | Notes |
| --- | --- | --- | --- |
| rosbridge websocket | implemented | no | JSON over ws; simplest cross-machine setup, highest overhead for images. |
| JSONL → `rclpy` relay | implemented | relay host only | `ros2/robosim_relay.py`; works over a pipe or SSH. |
| Zenoh (`zenoh-bridge-ros2dds`) | planned | no | Native Rust client possible; best fit for a Rust sim. |
| Native DDS / rmw | planned | yes | Highest fidelity, heaviest toolchain; not appropriate for the POC. |

Graceful absence: nothing in the workspace links against ROS. If no transport
is configured, or rosbridge is unreachable, the sim logs one warning and runs
with `NullTransport`. The Python relay exits with a clear message if `rclpy`
cannot be imported.

## 8. Testing strategy

* Unit tests per module (kinematics, ray casts, clamping).
* Integration tests per crate: lidar hit distances against a known box,
  wall-stop invariants, determinism (two sims → byte-identical `Frame`s),
  render output sizes, bridge topic coverage, JSONL well-formedness,
  fallback on an unreachable rosbridge.
* CI runs fmt, clippy (`-D warnings`), build, tests and two CLI smoke runs on
  macOS, Ubuntu and Windows.

## 9. Known limitations

See [roadmap.md](roadmap.md) for the full list and the comparison with Gazebo.
