//! Marker for a future GPU backend. Not implemented; the headless and ASCII
//! backends are the only working renderers in this POC.

/// Describes which native GPU API a future backend would target.
pub struct GpuBackendStub;

impl GpuBackendStub {
    /// Planned GPU backend for the current platform. Not implemented.
    pub fn describe() -> &'static str {
        if cfg!(target_os = "macos") {
            "metal (planned, not implemented)"
        } else if cfg!(target_os = "windows") {
            "dx12 (planned, not implemented)"
        } else {
            "vulkan (planned, not implemented)"
        }
    }
}
