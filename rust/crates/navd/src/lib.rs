#![forbid(unsafe_code)]

pub mod destination;
pub mod external;
pub mod geometry;
pub mod instructions;
mod json;
#[cfg(feature = "native")]
pub mod native;
pub mod position;
pub mod route;
pub mod wire;

#[derive(Debug, thiserror::Error)]
pub enum Error {
    #[error(transparent)]
    Json(#[from] serde_json::Error),
    #[error(transparent)]
    Capnp(#[from] capnp::Error),
    #[error(transparent)]
    Io(#[from] std::io::Error),
    #[error(transparent)]
    Url(#[from] url::ParseError),
    #[error("navigation number formatting failed")]
    Format(#[from] std::fmt::Error),
    #[error("navigation JSON field has an invalid type or is missing: {0}")]
    Field(&'static str),
    #[error("navigation geometry is empty")]
    EmptyGeometry,
    #[error("navigation distance is outside the source trigonometric domain")]
    DistanceDomain,
    #[error("navigation geometry exceeds addressable coordinates")]
    GeometrySize,
    #[error("navigation runtime: {0}")]
    Runtime(&'static str),
    #[error("navigation interrupted")]
    Interrupted,
    #[cfg(feature = "native")]
    #[error(transparent)]
    Params(#[from] openpilot_params::Error),
    #[cfg(feature = "native")]
    #[error(transparent)]
    TypedParams(#[from] openpilot_params_typed::Error),
    #[cfg(feature = "native")]
    #[error(transparent)]
    Logging(#[from] openpilot_logging::Error),
    #[cfg(feature = "native")]
    #[error(transparent)]
    Messaging(#[from] openpilot_messaging::runtime::Error),
    #[cfg(feature = "native")]
    #[error(transparent)]
    MessageState(#[from] openpilot_messaging::state::Error),
    #[cfg(feature = "native")]
    #[error(transparent)]
    Hardware(#[from] openpilot_hardware_info::Error),
    #[cfg(feature = "native")]
    #[error(transparent)]
    Integer(#[from] openpilot_beepd::IntegerError),
    #[cfg(feature = "native")]
    #[error(transparent)]
    Clock(#[from] openpilot_beepd::Error),
    #[cfg(feature = "native")]
    #[error(transparent)]
    Signing(#[from] openpilot_uploader::TransferError),
}
