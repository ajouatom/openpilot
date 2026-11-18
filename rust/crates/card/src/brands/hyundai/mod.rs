pub mod authority;
pub mod blink_hold;
pub mod bus;
pub mod camera_sync;
pub mod canfd_acc;
pub mod canfd_buttons;
pub mod canfd_cluster;
pub mod canfd_maintenance;
pub mod canfd_steering;
pub mod ccnc_display;
pub mod cluster_fields;
pub mod cluster_hud;
pub mod config;
pub mod controller;
mod controller_buttons;
mod controller_control;
mod controller_fd;
mod controller_hud;
mod controller_legacy;
mod controller_model;
mod controller_settings;
pub mod detection;
mod fd_buttons;
mod fd_corner;
mod fd_cruise;
pub mod flags;
mod float_number;
pub mod jerk;
pub mod kick;
pub mod lead;
pub mod legacy_acc;
mod legacy_buttons;
mod legacy_powertrain;
pub mod legacy_steering;
pub mod limits;
pub mod local_time;
pub mod monitor;
pub mod navigation;
pub mod parameters;
pub mod parser_inputs;
pub mod pedal;
mod runtime;
pub mod settings_float;
pub mod spas;
pub mod spas_signals;
pub mod stalks;
mod startup;
pub mod state;
mod state_canfd;
mod state_fields;
mod state_legacy;
mod state_navigation;
pub mod steering_handover;
pub mod steering_touch;
pub mod stock_adopt;
pub mod stopping;
pub mod wire;
pub use crate::core::{ApplyInput, ApplyOutput};
pub use parameters::ParamsInput;
pub use runtime::{Hyundai, Setup};

impl Hyundai {
    pub fn parameter_diagnostics(
        input: &ParamsInput<'_>,
        params: &capnp::message::Builder<capnp::message::HeapAllocator>,
    ) -> Result<Vec<String>, Error> {
        parameter_diagnostics::lines(input, params)
    }
    pub fn parameters(
        input: ParamsInput<'_>,
    ) -> Result<capnp::message::Builder<capnp::message::HeapAllocator>, Error> {
        parameters::parameters(input)
    }
}

#[derive(Debug, thiserror::Error)]
pub enum Error {
    #[error(transparent)]
    Can(#[from] openpilot_can::Error),
    #[error(transparent)]
    Capnp(#[from] capnp::Error),
    #[error(transparent)]
    Params(#[from] openpilot_params::Error),
    #[error(transparent)]
    Baseline(#[from] crate::vehicle_params::Error),
    #[error(transparent)]
    Utf8(#[from] std::str::Utf8Error),
    #[error(transparent)]
    Integer(#[from] std::num::ParseIntError),
    #[error(transparent)]
    Policy(#[from] openpilot_control_policy::Error),
    #[error(transparent)]
    Enum(#[from] capnp::NotInSchema),
    #[error(transparent)]
    Io(#[from] std::io::Error),
    #[error(transparent)]
    Float(#[from] std::num::ParseFloatError),
    #[error("Hyundai floating point setting outside source range: {0}")]
    FloatRange(String),
    #[error("missing copied Hyundai CAN signal {0}")]
    Signal(String),
    #[error("Hyundai numeric conversion")]
    Numeric,
}

mod fd_stopping;

mod cluster_buttons;

mod diagnostics;
mod fd_powertrain;
mod parameter_diagnostics;
mod python_set;

pub mod vehicle_gps;
