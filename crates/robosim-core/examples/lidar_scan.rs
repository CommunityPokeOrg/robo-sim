//! Print a small polar histogram of the lidar scan in the demo scene.

use robosim_core::{Simulator, World};

fn main() {
    let mut sim = Simulator::new(World::demo_scene());
    sim.lidar.num_beams = 36;
    sim.step_n(1);
    let f = sim.snapshot();
    for (i, r) in f.scan.ranges.iter().enumerate() {
        let angle = f.scan.angle_min + i as f32 * f.scan.angle_increment;
        let bars = if r.is_finite() {
            "#".repeat((r * 6.0) as usize)
        } else {
            "-- miss --".to_string()
        };
        println!("{:>+6.1} deg : {:>6.2}  {}", angle.to_degrees(), r, bars);
    }
}
