//! Native Jetlink contract, source policy and deadline-bounded local transport.
pub mod adapter;
pub mod app_pick;
pub mod aux;
pub mod client;
pub mod contract;
pub mod fault;
pub mod ffs;
pub mod gadget;
pub mod mobile;
pub mod model_spec;
pub mod owner;
pub mod peer;
pub mod phase;
pub mod physical;
mod plan_hypothesis;
pub mod platform;
pub mod protocol_choice;
pub mod rpc;
pub mod runtime;
pub mod selection;
pub mod session_contract;
pub mod shared_input;
pub mod status;
pub mod tcp;
pub mod transition;
pub mod wifi;
pub mod wire;

use std::time::Duration;

#[derive(Debug, thiserror::Error)]
pub enum Error {
    #[error("Jetlink native adapter: {0}")]
    Native(String),
    #[error("Jetlink contract: {0}")]
    Contract(&'static str),
    /// What the phone or Jetson answered, with the values needed to act on it.
    #[error("Jetlink peer: {0}")]
    Peer(String),
    #[error("Jetlink deadline exceeded")]
    Deadline,
    #[error("Jetlink session closed")]
    Closed,
    #[error(transparent)]
    Io(#[from] std::io::Error),
    #[error(transparent)]
    Json(#[from] serde_json::Error),
    #[error(transparent)]
    Parse(#[from] openpilot_modeld::parse::ParseError),
}

#[derive(Clone, Copy, Debug)]
pub struct Deadline(pub u64);
impl Deadline {
    pub fn after(duration: Duration) -> Result<Self, Error> {
        let nanos = u64::try_from(duration.as_nanos()).map_err(|_| Error::Deadline)?;
        Ok(Self(now_ns().checked_add(nanos).ok_or(Error::Deadline)?))
    }
    pub fn remaining(self) -> Result<Duration, Error> {
        self.0
            .checked_sub(now_ns())
            .filter(|n| *n > 0)
            .map(Duration::from_nanos)
            .ok_or(Error::Deadline)
    }
}
pub fn now_ns() -> u64 {
    let ts = rustix::time::clock_gettime(rustix::time::ClockId::Monotonic);
    Duration::new(
        ts.tv_sec.unsigned_abs(),
        u32::try_from(ts.tv_nsec).unwrap_or(0),
    )
    .as_nanos()
    .try_into()
    .unwrap_or(u64::MAX)
}
