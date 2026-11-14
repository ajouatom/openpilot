//! Carrot's model route map (`selfdrive/navigation/mapsd.py`): the 256x256
//! road raster carrot's N16 navigation graph takes, rendered on the CPU from
//! vector tiles that are kept locally and refreshed from a remote archive.
pub mod output;
pub mod pmtiles;
#[cfg(feature = "native")]
pub mod remote;
pub mod renderer;
pub mod route;
pub mod tiles;
pub mod vector_tile;

#[derive(Debug, thiserror::Error)]
pub enum Error {
    #[error("{0}")]
    Format(&'static str),
    #[error(transparent)]
    Io(#[from] std::io::Error),
    #[error("{0}")]
    Remote(String),
}
