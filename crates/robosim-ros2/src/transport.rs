//! Transports that carry `Publication`s to ROS 2 — or nowhere.

use crate::msgs::Twist;
use crate::Publication;
use serde_json::json;
use std::io::Write;
use std::net::{SocketAddr, TcpStream, ToSocketAddrs};
use std::time::Duration;
use thiserror::Error;

/// Transport errors.
#[derive(Debug, Error)]
pub enum TransportError {
    /// I/O failure.
    #[error("i/o error: {0}")]
    Io(#[from] std::io::Error),
    /// Websocket failure.
    #[error("websocket error: {0}")]
    Ws(String),
    /// JSON (de)serialization failure.
    #[error("json error: {0}")]
    Json(#[from] serde_json::Error),
}

/// Result alias.
pub type Result<T> = std::result::Result<T, TransportError>;

/// A sink for publications, plus an optional /cmd_vel source.
pub trait Transport {
    /// Advertise a topic.
    fn advertise(&mut self, topic: &str, msg_type: &str) -> Result<()>;
    /// Publish one message.
    fn publish(&mut self, p: &Publication) -> Result<()>;
    /// Non-blocking poll for an incoming twist command (/cmd_vel).
    fn poll_twist(&mut self) -> Option<Twist> {
        None
    }
    /// Number of messages published so far.
    fn published_count(&self) -> usize {
        0
    }
}

/// Drops everything; used when ROS 2 is unavailable.
#[derive(Debug, Default)]
pub struct NullTransport {
    count: usize,
}

impl NullTransport {
    /// Messages seen so far.
    pub fn count(&self) -> usize {
        self.count
    }
}

impl Transport for NullTransport {
    fn advertise(&mut self, _topic: &str, _msg_type: &str) -> Result<()> {
        Ok(())
    }
    fn publish(&mut self, _p: &Publication) -> Result<()> {
        self.count += 1;
        Ok(())
    }
    fn published_count(&self) -> usize {
        self.count
    }
}

/// Writes rosbridge-style `{"op":"publish",...}` JSON lines to any `Write`.
/// Consumed by `ros2/robosim_relay.py` and handy for tests/logs.
pub struct JsonlTransport<W: Write> {
    w: W,
    count: usize,
}

impl<W: Write> JsonlTransport<W> {
    /// Wrap a writer.
    pub fn new(w: W) -> Self {
        Self { w, count: 0 }
    }
}

impl<W: Write> Transport for JsonlTransport<W> {
    fn advertise(&mut self, topic: &str, msg_type: &str) -> Result<()> {
        let line = json!({"op": "advertise", "topic": topic, "type": msg_type});
        writeln!(self.w, "{line}")?;
        self.w.flush()?;
        Ok(())
    }
    fn publish(&mut self, p: &Publication) -> Result<()> {
        let line = json!({
            "op": "publish",
            "topic": p.topic,
            "type": p.msg_type,
            "msg": p.payload,
        });
        writeln!(self.w, "{line}")?;
        self.w.flush()?;
        self.count += 1;
        Ok(())
    }
    fn published_count(&self) -> usize {
        self.count
    }
}

/// rosbridge websocket client (ws://, no TLS).
pub struct RosbridgeTransport {
    sock: tungstenite::WebSocket<TcpStream>,
    count: usize,
}

impl RosbridgeTransport {
    /// Connect to e.g. `ws://localhost:9090` with a short TCP timeout.
    /// Subscribes to /cmd_vel and sets the socket non-blocking for polling.
    pub fn connect(url: &str) -> Result<Self> {
        // Extract host:port for a connect_timeout'd TcpStream.
        let stripped = url
            .strip_prefix("ws://")
            .ok_or_else(|| TransportError::Ws(format!("unsupported url scheme: {url}")))?;
        let host_port = stripped.split('/').next().unwrap_or(stripped).to_string();
        let addr: SocketAddr = host_port
            .to_socket_addrs()?
            .next()
            .ok_or_else(|| TransportError::Ws(format!("cannot resolve {host_port}")))?;
        let stream = TcpStream::connect_timeout(&addr, Duration::from_secs(2))?;
        stream.set_nonblocking(false)?;
        let (mut sock, _resp) =
            tungstenite::client(url, stream).map_err(|e| TransportError::Ws(e.to_string()))?;
        // Subscribe to /cmd_vel.
        let sub = json!({
            "op": "subscribe",
            "topic": "/cmd_vel",
            "type": "geometry_msgs/msg/Twist",
        });
        sock.send(tungstenite::Message::Text(sub.to_string().into()))
            .map_err(|e| TransportError::Ws(e.to_string()))?;
        // Switch to nonblocking so poll_twist never stalls.
        sock.get_ref().set_nonblocking(true)?;
        Ok(Self { sock, count: 0 })
    }
}

impl Transport for RosbridgeTransport {
    fn advertise(&mut self, topic: &str, msg_type: &str) -> Result<()> {
        // Temporarily restore blocking to keep writes reliable.
        self.sock.get_ref().set_nonblocking(false)?;
        let msg = json!({"op": "advertise", "topic": topic, "type": msg_type});
        let r = self
            .sock
            .send(tungstenite::Message::Text(msg.to_string().into()))
            .map_err(|e| TransportError::Ws(e.to_string()));
        self.sock.get_ref().set_nonblocking(true)?;
        r
    }

    fn publish(&mut self, p: &Publication) -> Result<()> {
        self.sock.get_ref().set_nonblocking(false)?;
        let msg = json!({
            "op": "publish",
            "topic": p.topic,
            "type": p.msg_type,
            "msg": p.payload,
        });
        let r = self
            .sock
            .send(tungstenite::Message::Text(msg.to_string().into()))
            .map_err(|e| TransportError::Ws(e.to_string()));
        self.sock.get_ref().set_nonblocking(true)?;
        if r.is_ok() {
            self.count += 1;
        }
        r
    }

    fn poll_twist(&mut self) -> Option<Twist> {
        // Drain all pending frames; return the last Twist seen.
        let mut found = None;
        loop {
            match self.sock.read() {
                Ok(tungstenite::Message::Text(txt)) => {
                    if let Ok(v) = serde_json::from_str::<serde_json::Value>(&txt) {
                        if v.get("op").and_then(|o| o.as_str()) == Some("publish")
                            && v.get("topic").and_then(|t| t.as_str()) == Some("/cmd_vel")
                        {
                            if let Ok(t) = serde_json::from_value::<Twist>(v["msg"].clone()) {
                                found = Some(t);
                            }
                        }
                    }
                }
                Ok(_) => {}
                Err(tungstenite::Error::Io(e)) if e.kind() == std::io::ErrorKind::WouldBlock => {
                    break;
                }
                Err(_) => break,
            }
        }
        found
    }

    fn published_count(&self) -> usize {
        self.count
    }
}
