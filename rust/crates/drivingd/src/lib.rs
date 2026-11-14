pub mod bus;
pub mod camera;
pub mod camera_gap;
pub mod clock;
pub mod diagnostics;
pub mod indicator;
pub mod jetlink;
pub mod navigation;
pub mod parameters;
pub mod publication;
pub mod recovery;
pub mod recovery_policy;
pub mod recovery_prepare;
pub mod recovery_task;
pub mod runtime;
pub mod state;
pub mod usb_model;
pub mod usb_selection;
pub mod wire;

#[derive(Debug, thiserror::Error)]
pub enum Error {
    #[error(transparent)]
    UsbGpu(#[from] openpilot_usbgpu::Error),
    #[error(transparent)]
    Json(#[from] serde_json::Error),
    #[error(transparent)]
    Logging(#[from] openpilot_logging::Error),
    #[error(
        "active Jetlink output has no raw_pred for SEND_RAW_PRED (inherited original boundary)"
    )]
    JetlinkRawPredictionsUnavailable,
    #[error(transparent)]
    Jetlink(#[from] openpilot_jetlink::Error),
    #[error(transparent)]
    Messaging(#[from] openpilot_messaging::runtime::Error),
    #[error(transparent)]
    MessageState(#[from] openpilot_messaging::state::Error),
    #[error("driving model contract: {0}")]
    Contract(&'static str),
    #[error(transparent)]
    Runtime(#[from] openpilot_model_runtime::Error),
    #[error(transparent)]
    Parse(#[from] openpilot_modeld::parse::ParseError),
    #[error(transparent)]
    Features(#[from] openpilot_modeld::inputs::FeatureCount),
    #[error(transparent)]
    Camera(#[from] openpilot_modeld::calibration::UnknownCamera),
    #[error(transparent)]
    Transport(#[from] openpilot_msgq::Error),
    #[error(transparent)]
    Params(#[from] openpilot_params::Error),
    #[error(transparent)]
    Io(#[from] std::io::Error),
    #[error(transparent)]
    Cereal(#[from] capnp::Error),
    #[error(transparent)]
    Schema(#[from] capnp::NotInSchema),
    #[error(transparent)]
    Desire(#[from] openpilot_desire::types::InvalidModel),
    #[error(transparent)]
    Text(#[from] std::str::Utf8Error),
}
