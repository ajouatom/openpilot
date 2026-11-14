//! Native u-blox protocol and receiver policy, ported from system/ubloxd.
pub mod binary;
#[cfg(feature = "native")]
#[allow(unsafe_code)]
mod bridge;
pub mod commands;
pub mod framing;
mod glonass;
mod gps;
pub mod layouts;
#[cfg(feature = "native")]
pub mod native;
pub mod parser;
pub mod pigeon;
mod reports;
#[cfg(feature = "native")]
pub mod runtime;
#[cfg(feature = "native")]
pub mod serial;

#[derive(Debug, thiserror::Error)]
pub enum Error {
    #[error("No response from ublox")]
    Timeout,
    #[error("interrupted")]
    Interrupted,
    #[error("{0}")]
    Boundary(String),
    #[error("malformed u-blox data: {0}")]
    Malformed(&'static str),
    #[error(transparent)]
    Io(#[from] std::io::Error),
    #[error(transparent)]
    Capnp(#[from] capnp::Error),
    #[error(transparent)]
    Json(#[from] serde_json::Error),
    #[cfg(feature = "native")]
    #[error(transparent)]
    Messaging(#[from] openpilot_messaging::runtime::Error),
    #[cfg(feature = "native")]
    #[error(transparent)]
    Transport(#[from] openpilot_msgq::Error),
    #[cfg(feature = "native")]
    #[error(transparent)]
    Logging(#[from] openpilot_logging::Error),
    #[cfg(feature = "native")]
    #[error(transparent)]
    Hardware(#[from] openpilot_hardware_control::Error),
    #[cfg(feature = "native")]
    #[error(transparent)]
    Params(#[from] openpilot_params::Error),
    #[cfg(feature = "native")]
    #[error(transparent)]
    ParamsTyped(#[from] openpilot_params_typed::Error),
    #[cfg(feature = "native")]
    #[error(transparent)]
    Time(#[from] openpilot_timed::Error),
    #[cfg(feature = "native")]
    #[error(transparent)]
    Http(#[from] ureq::Error),
}
