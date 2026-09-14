use robosim_core::{Simulator, World};
use robosim_render::{AsciiBackend, HeadlessBackend, RenderBackend, RenderOutput};

#[test]
fn headless_renders_pixels() {
    let mut sim = Simulator::new(World::demo_scene());
    sim.step_n(10);
    let frame = sim.snapshot();
    let mut b = HeadlessBackend::default();
    b.begin_frame(200, 200);
    b.draw_scene(&frame, &sim.world);
    let out = b.end_frame();
    match out {
        RenderOutput::Pixels { width, height, rgb } => {
            assert_eq!((width, height), (200, 200));
            assert_eq!(rgb.len(), 200 * 200 * 3);
            assert!(rgb.iter().any(|&v| v != 0));
        }
        _ => panic!("expected pixels"),
    }
}

#[test]
fn headless_draws_robot() {
    let sim = Simulator::new(World::demo_scene());
    let frame = sim.snapshot();
    let mut b = HeadlessBackend::default();
    let (w, h) = (200u32, 200u32);
    b.begin_frame(w, h);
    b.draw_scene(&frame, &sim.world);
    // Robot at origin → view center. Probe a small area around center for the
    // robot color (240,220,60).
    let cx = w as usize / 2;
    let cy = h as usize / 2;
    let out = b.end_frame();
    if let RenderOutput::Pixels { rgb, .. } = out {
        let mut found = false;
        for dy in -5..=5 {
            for dx in -5..=5 {
                let i = ((cy as i32 + dy) as usize * w as usize + (cx as i32 + dx) as usize) * 3;
                if rgb[i] == 240 && rgb[i + 1] == 220 && rgb[i + 2] == 60 {
                    found = true;
                }
            }
        }
        assert!(found, "robot pixel not drawn");
    } else {
        panic!("expected pixels");
    }
}

#[test]
fn ascii_grid_dims() {
    let sim = Simulator::new(World::demo_scene());
    let frame = sim.snapshot();
    let mut a = AsciiBackend::new(80, 24);
    a.begin_frame(80, 24);
    a.draw_scene(&frame, &sim.world);
    if let RenderOutput::Text(s) = a.end_frame() {
        let lines: Vec<&str> = s.lines().collect();
        assert_eq!(lines.len(), 24);
        assert!(lines.iter().all(|l| l.chars().count() == 80));
        assert!(s.contains('@'));
    } else {
        panic!("expected text");
    }
}
