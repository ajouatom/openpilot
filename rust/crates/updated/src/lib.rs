//! Native staged updater. Original policy: system/updated/{updated,process,common}.py.
pub mod agnos;
pub mod common;
pub mod markdown;
mod overlay;
mod params;
pub mod paths;
pub mod process;
pub mod runtime;
pub mod signals;
pub mod updater;
pub use params::{ParamTime, Params};
mod report;

#[derive(Debug, thiserror::Error)]
pub enum Error {
    #[error(transparent)]
    Io(#[from] std::io::Error),
    #[error(transparent)]
    Process(#[from] openpilot_process_supervision::Error),
    #[error(transparent)]
    Params(#[from] openpilot_params::Error),
    #[error(transparent)]
    TypedParams(#[from] openpilot_params_typed::Error),
    #[error(transparent)]
    Metadata(#[from] openpilot_runtime_version::Error),
    #[error(transparent)]
    Hardware(#[from] openpilot_hardware_info::Error),
    #[error(transparent)]
    Logging(#[from] openpilot_logging::Error),
    #[error(transparent)]
    Clock(#[from] openpilot_timed::Error),
    #[error(transparent)]
    PythonJson(#[from] openpilot_logmessaged::JsonError),
    #[error(transparent)]
    Format(#[from] std::fmt::Error),
    #[error(transparent)]
    Json(#[from] serde_json::Error),
    #[error(transparent)]
    Utf8(#[from] std::string::FromUtf8Error),
    #[error("command failed: {command:?}\n{output}")]
    Command {
        command: Vec<String>,
        code: i32,
        output: String,
    },
    #[error("updater interrupted")]
    Interrupted,
    #[error("AGNOS update failed: {0}")]
    Agnos(#[from] openpilot_agnos::Error),
    #[error("{0}")]
    Contract(&'static str),
    #[error("copytree failed: {0:?}")]
    CopyTree(Vec<(std::path::PathBuf, String)>),
    #[error("not enough storage for the update copy: it needs {needed_mb} MB, {available_mb} MB is free and the drive logs keep {floor_mb} MB; the update was not downloaded and no log was deleted")]
    Storage {
        needed_mb: u64,
        available_mb: u64,
        floor_mb: u64,
    },
}
