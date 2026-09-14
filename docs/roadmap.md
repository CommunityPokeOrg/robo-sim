# Limitations and roadmap

robo-sim is a **proof of concept**. This page is honest about what it is not,
and about what would have to happen for it to become a usable simulator.

## What this POC is

* A pure-Rust, dependency-light core that builds and tests identically on
  macOS, Linux and Windows.
* A deterministic fixed-step differential-drive simulation with collision
  push-out, an analytic 2D lidar and a mock camera.
* A headless and a terminal renderer, and a `RenderBackend` seam for a GPU
  backend.
* A ROS 2 bridge that publishes `/clock`, `/tf`, `/joint_states`, `/scan`,
  `/camera/image_raw`, `/odom` and consumes `/cmd_vel` via rosbridge or an
  `rclpy` relay — and keeps working when ROS 2 is absent.

## What it is not (vs. Gazebo)

| Area | Gazebo (Harmonic) | robo-sim POC |
| --- | --- | --- |
| Physics | 3D rigid-body (DART/Bullet/ODE), joints, friction, contacts | 2D kinematic diff-drive, circle-vs-shape push-out |
| World format | SDF/URDF, meshes, plugins | Hard-coded `World::demo_scene()`, boxes and circles |
| Sensors | Cameras, depth, GPU lidar, IMU, GPS, contact, force-torque | 2D lidar, mock RGB camera, odometry |
| Rendering | OGRE2 PBR, GUI, plugins | Software top-down / ASCII; GPU path designed, not built |
| ROS 2 | Native `ros_gz` bridge, many message types | 6 topics + `/cmd_vel` via rosbridge / relay |
| Multi-robot | Yes | Data model supports many robots; sensors/CLI target robot 0 |
| Scripting / plugins | C++ plugin API | None (Rust API only) |
| Maturity | Production | Weeks-old POC |

Use Gazebo (or Isaac Sim, Webots, CoppeliaSim) when you need any of the left
column today. robo-sim exists to explore whether a small, fast, hackable
cross-platform sim is viable — not to replace them.

## Known limitations

* 2D only; poses are `(x, y, θ)`. `/tf` and `/odom` fill `z`, roll and pitch
  with zeros.
* No dynamics: no mass, inertia, friction or wheel slip. Commands take effect
  instantly (clamped only by max wheel speed).
* One lidar and one camera, both attached to robot 0.
* Camera images are a stylised column-raycast, not a rendering of the scene.
* No scene file format; scenes are constructed in Rust.
* rosbridge transport is JSON text; images are inefficient over it.
* Zenoh / native DDS transports are designed but not implemented.
* `teleop` is verified to build on all three OSes but exercised manually only
  on Linux terminals.

## Roadmap

Rough order; each item is intended to be a self-contained PR.

1. **Scene files** — load worlds and robots from a small TOML/JSON schema;
   optional URDF subset for wheel geometry and sensor mounts.
2. **GPU renderer** — `wgpu` + `winit` backend behind a `gpu` feature
   (Metal / Vulkan / DX12 per [architecture.md](architecture.md#6-cross-platform-rendering-bridge)).
3. **Real camera** — render the camera sensor from the GPU backend
   (off-screen target), keeping the CPU mock as fallback.
4. **Physics upgrade** — swap `SimplePhysics` for a pure-Rust rigid-body
   engine behind `PhysicsBackend`; add friction and wheel slip.
5. **Zenoh transport** — native Rust client speaking to
   `zenoh-bridge-ros2dds`; CDR encoding for images.
6. **More sensors** — IMU, depth/point cloud, bumper/contact.
7. **Multi-robot** — per-robot sensor sets and namespaced topics.
8. **Python bindings** — thin PyO3 wrapper over `Simulator` for RL loops.
9. **Recording / playback** — dump and replay `Frame` streams (already serde).
10. **3D** — generalise `Pose2` to SE(3) once physics and rendering are there.

## Contributing

Small, focused PRs against `main`. CI must stay green on all three OSes, and
the default build must remain free of native dependencies.
