//! ROS 2 bridge: rosbridge-compatible message structs, Frame → message
//! conversion, and pluggable transports (null, JSONL, rosbridge websocket).
#![forbid(unsafe_code)]

pub mod bridge;
pub mod convert;
pub mod msgs;
pub mod transport;

pub use bridge::{connect_or_fallback, Bridge, Publication};
pub use convert::{frame_to_messages, BridgeConfig};
pub use transport::{JsonlTransport, NullTransport, RosbridgeTransport, Transport};
