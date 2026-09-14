//! Simulation clock.

/// Monotonic simulation clock advancing in fixed steps.
#[derive(Debug, Clone, serde::Serialize, serde::Deserialize)]
pub struct SimClock {
    /// Elapsed simulated time in seconds.
    pub time_s: f64,
    /// Number of steps taken.
    pub step: u64,
    /// Fixed timestep in seconds.
    pub dt: f64,
}

impl SimClock {
    /// Default physics rate: 100 Hz.
    pub const DEFAULT_DT: f64 = 1.0 / 100.0;

    /// Create a clock with the default 1/100 s fixed timestep.
    pub fn fixed_timestep() -> Self {
        Self::with_dt(Self::DEFAULT_DT)
    }

    /// Create a clock with a custom timestep.
    pub fn with_dt(dt: f64) -> Self {
        Self {
            time_s: 0.0,
            step: 0,
            dt,
        }
    }

    /// Advance the clock by one step.
    pub fn advance(&mut self) {
        self.step += 1;
        self.time_s += self.dt;
    }

    /// Split seconds into (sec, nanosec) for ROS-style stamps.
    pub fn to_sec_nanosec(&self) -> (i32, u32) {
        let sec = self.time_s.floor() as i32;
        let nanosec = ((self.time_s - sec as f64) * 1e9).round() as u32;
        (sec, nanosec)
    }
}

impl Default for SimClock {
    fn default() -> Self {
        Self::fixed_timestep()
    }
}
