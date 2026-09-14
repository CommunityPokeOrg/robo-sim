//! Publish a few frames to rosbridge (or fall back to counting publications).
//! Usage: `cargo run -p robosim-ros2 --example rosbridge_publish -- [ws://host:9090]`

use robosim_core::{Simulator, World};
use robosim_ros2::{connect_or_fallback, Bridge};

fn main() {
    let url = std::env::args().nth(1);
    let transport = connect_or_fallback(url.as_deref());
    let mut bridge = Bridge::new(transport);

    let mut sim = Simulator::new(World::demo_scene());
    sim.set_twist(0, 0.4, 0.2);
    for _ in 0..100 {
        sim.step();
        bridge.publish_frame(&sim.snapshot());
    }
    println!(
        "published {} messages{}",
        bridge.transport.published_count(),
        if url.is_some() {
            " via rosbridge"
        } else {
            " (null transport)"
        }
    );
}
