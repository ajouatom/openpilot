mod blindspot;
mod geometry;
mod lane;
mod remote;
mod session;
pub use blindspot::BlindspotModel;
pub use geometry::Geometry;
pub use lane::{lane_image, lane_tensor, LaneModel};

#[derive(Debug, thiserror::Error)]
pub enum Error {
    #[error(transparent)]
    OpenCv(#[from] openpilot_opencv_runtime::Error),
    #[error(transparent)]
    Nv12(#[from] crate::nv12::Error),
    #[error(transparent)]
    Policy(#[from] crate::Error),
    #[error("{0}")]
    Contract(&'static str),
}
