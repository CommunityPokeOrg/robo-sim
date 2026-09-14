//! Rendering backends: a pluggable trait plus a headless software rasterizer,
//! an ASCII map renderer for the TUI, and a stub describing future GPU backends.
#![forbid(unsafe_code)]

mod ascii;
mod gpu;
mod headless;

use robosim_core::{Frame, World};
use thiserror::Error;

/// Errors produced by rendering backends.
#[derive(Debug, Error)]
pub enum RenderError {
    /// I/O failure (e.g. writing a PPM).
    #[error("i/o error: {0}")]
    Io(#[from] std::io::Error),
}

/// Pixel/character data produced by a backend at end of frame.
#[derive(Debug)]
pub enum RenderOutput {
    /// RGB pixel buffer.
    Pixels {
        /// Width.
        width: u32,
        /// Height.
        height: u32,
        /// Row-major RGB bytes, len == w*h*3.
        rgb: Vec<u8>,
    },
    /// Rendered text (ASCII backend).
    Text(String),
    /// Backend produced nothing this frame.
    None,
}

/// Pluggable rendering interface.
pub trait RenderBackend {
    /// Human/backend name.
    fn name(&self) -> &str;
    /// Begin a frame at the given output size.
    fn begin_frame(&mut self, w: u32, h: u32);
    /// Draw the world + latest frame.
    fn draw_scene(&mut self, frame: &Frame, world: &World);
    /// Finish and return the frame output.
    fn end_frame(&mut self) -> RenderOutput;
}

pub use ascii::AsciiBackend;
pub use gpu::GpuBackendStub;
pub use headless::HeadlessBackend;
