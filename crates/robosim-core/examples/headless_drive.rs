//! Drive the demo robot forward a few seconds headlessly and print the result.

use robosim_core::{Simulator, World};

fn main() {
    let mut sim = Simulator::new(World::demo_scene());
    sim.set_twist(0, 0.4, 0.3);
    sim.step_n(300); // 3 s at 100 Hz
    let f = sim.snapshot();
    let r = &f.robots[0];
    println!(
        "t={:.2}s pose=({:.3}, {:.3}, {:.3}) v={:.3} omega={:.3}",
        f.time, r.pose.position.x, r.pose.position.y, r.pose.heading, r.v, r.omega
    );
    let finite = f.scan.ranges.iter().filter(|r| r.is_finite()).count();
    println!("lidar: {} beams, {} hits", f.scan.ranges.len(), finite);
}
