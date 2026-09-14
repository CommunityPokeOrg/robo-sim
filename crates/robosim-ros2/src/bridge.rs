//! The `Bridge`: lazily advertises topics once, then publishes each `Frame`.

use crate::convert::{frame_to_messages, BridgeConfig, TOPICS};
use crate::transport::{NullTransport, RosbridgeTransport, Transport};
use robosim_core::Frame;

/// One rosbridge publication.
#[derive(Debug, Clone, serde::Serialize)]
pub struct Publication {
    /// Topic name, e.g. "/scan".
    pub topic: String,
    /// ROS message type, e.g. "sensor_msgs/msg/LaserScan".
    pub msg_type: String,
    /// Message payload as JSON.
    pub payload: serde_json::Value,
}

/// Frame → transport bridge.
pub struct Bridge {
    /// Sink for publications.
    pub transport: Box<dyn Transport>,
    /// Frame/topic naming.
    pub cfg: BridgeConfig,
    advertised: bool,
}

impl Bridge {
    /// Create a bridge over a transport.
    pub fn new(transport: Box<dyn Transport>) -> Self {
        Self {
            transport,
            cfg: BridgeConfig::default(),
            advertised: false,
        }
    }

    /// Advertise all topics once (no-op after the first call).
    pub fn advertise_once(&mut self) {
        if self.advertised {
            return;
        }
        for (topic, ty) in TOPICS {
            // Best-effort; transports that fail keep failing per publish.
            let _ = self.transport.advertise(topic, ty);
        }
        self.advertised = true;
    }

    /// Convert and publish a frame.
    pub fn publish_frame(&mut self, frame: &Frame) {
        self.advertise_once();
        for p in frame_to_messages(frame, &self.cfg) {
            let _ = self.transport.publish(&p);
        }
    }
}

/// Connect to rosbridge if `url` is provided, else return a `NullTransport`.
/// On connection failure logs a warning to stderr and falls back gracefully.
pub fn connect_or_fallback(url: Option<&str>) -> Box<dyn Transport> {
    match url {
        None => Box::new(NullTransport::default()),
        Some(u) => match RosbridgeTransport::connect(u) {
            Ok(t) => Box::new(t),
            Err(e) => {
                eprintln!("ROS 2 bridge unavailable ({e}): continuing without ROS 2");
                Box::new(NullTransport::default())
            }
        },
    }
}
