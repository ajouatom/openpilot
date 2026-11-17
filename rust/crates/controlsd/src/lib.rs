pub mod centering;
pub mod config;
pub mod controller;
mod feedback;
pub mod fingerprints;
mod hud;
pub mod input_decode;
pub mod inputs;
pub mod interface;
pub mod lateral;
mod lateral_torque;
pub mod longitudinal;
pub mod parameters;
pub mod physical_hold;
pub mod publication;
mod readiness;
pub mod sanitize;
pub mod suspend;
pub mod torque_neural;

#[derive(Debug, thiserror::Error)]
pub enum Error {
    #[error(transparent)]
    Format(#[from] std::fmt::Error),
    #[error("controlsd: {0}")]
    Contract(&'static str),
    #[error(transparent)]
    Policy(#[from] openpilot_control_policy::Error),
    #[error(transparent)]
    Cereal(#[from] capnp::Error),
    #[error(transparent)]
    Schema(#[from] capnp::NotInSchema),
    #[error(transparent)]
    Io(#[from] std::io::Error),
    #[error(transparent)]
    Json(#[from] serde_json::Error),
    #[error(transparent)]
    Parameter(#[from] openpilot_params::Error),
    #[error(transparent)]
    Integer(#[from] openpilot_beepd::IntegerError),
    #[error(transparent)]
    Float(#[from] openpilot_calibrationd::types::Error),
    #[error(transparent)]
    State(#[from] openpilot_messaging::state::Error),
    #[cfg(feature = "native")]
    #[error(transparent)]
    Messaging(#[from] openpilot_messaging::runtime::Error),
    #[cfg(feature = "native")]
    #[error(transparent)]
    Logging(#[from] openpilot_logging::Error),
}

pub const SERVICES: [&str; 16] = [
    "liveDelay",
    "liveParameters",
    "liveTorqueParameters",
    "modelV2",
    "selfdriveState",
    "liveCalibration",
    "livePose",
    "longitudinalPlan",
    "carState",
    "carOutput",
    "carrotMan",
    "lateralPlan",
    "radarState",
    "driverMonitoringState",
    "onroadEvents",
    "driverAssistance",
];
#[cfg(feature = "native")]
pub mod platform;
#[cfg(feature = "native")]
pub mod runtime;
