use super::Endpoint;
use crate::{geometry::Coordinate, instructions::BannerInstruction, Error};
use serde_json::Value;

#[derive(Clone, Debug)]
pub struct Config {
    /// Mapbox's API host (a fixture server in tests).
    pub host: String,
}

/// One 1 Hz update's inputs, as navd.py's `update` reads them.
#[derive(Clone, Debug, Default)]
pub struct Tick {
    /// Monotonic seconds.
    pub now: f64,
    /// `ParkedPosition.update`: the fused position, or none.
    pub position: Option<crate::position::NavigationPosition>,
    /// carrotMan valid, alive and `activeCarrot > 0`: an external navigation
    /// guides, so navd publishes nothing valid and drops its own route.
    pub external: bool,
    /// `MAPBOX_TOKEN` or `MapboxPublicKey`; empty for none.
    pub token: String,
}

#[derive(Clone, Copy, Debug)]
pub enum SpeedLimitSign {
    Mutcd,
    Vienna,
}

#[derive(Debug)]
pub struct Maneuver {
    pub distance: f64,
    pub kind: Option<String>,
    pub modifier: Option<String>,
}

#[derive(Debug, Default)]
pub struct Instruction {
    pub valid: bool,
    pub maneuver_distance: f64,
    pub banner: Option<BannerInstruction>,
    pub maneuvers: Vec<Maneuver>,
    pub distance_remaining: f64,
    pub time_remaining: f64,
    pub time_remaining_typical: f64,
    pub speed_limit: Option<f64>,
    pub speed_limit_sign: Option<SpeedLimitSign>,
}

#[derive(Debug)]
pub enum Diagnostic {
    NewDestination {
        new: Endpoint,
        previous: Option<Endpoint>,
        place: Option<Value>,
    },
    Calculating {
        from: Endpoint,
        to: Endpoint,
    },
    EmptyRoute,
    RequestFailed(RequestError),
    ComputeFailed(Error),
    RouteLimited {
        original: usize,
        sent: usize,
    },
    DestinationReached,
    SpeedLimit(f64),
    /// Mapbox answered with a route navd refuses (sizes, coordinates).
    InvalidRoute(Error),
}

#[derive(Debug, thiserror::Error)]
pub enum RequestError {
    #[error("route request interrupted")]
    Interrupted,
    #[error("route request failed: {0}")]
    Transport(String),
    #[error("route request returned HTTP {status}: {body}")]
    Status { status: u16, body: String },
    #[error(transparent)]
    Json(#[from] serde_json::Error),
}

pub trait Ports {
    fn parameter(&mut self, name: &str) -> Result<Option<String>, Error>;
    fn remove_parameter(&mut self, name: &str) -> Result<(), Error>;
    /// `/dev/shm/params`: the session-scoped `NavDestinationTemporary`.
    fn memory_parameter(&mut self, name: &str) -> Result<Option<String>, Error>;
    fn remove_memory_parameter(&mut self, name: &str) -> Result<(), Error>;
    /// Starts the one route request off the 1 Hz loop.
    fn submit(&mut self, url: &str) -> Result<(), Error>;
    /// That request's result once it is done; None while it runs.
    fn poll(&mut self) -> Option<Result<Value, RequestError>>;
    fn instruction(&mut self, message: &Instruction) -> Result<(), Error>;
    fn route(&mut self, coordinates: &[Coordinate]) -> Result<(), Error>;
    fn diagnostic(&mut self, event: Diagnostic);
    fn geometry_changed(
        &mut self,
        _coordinates: Result<Vec<Coordinate>, Error>,
    ) -> Result<(), Error> {
        Ok(())
    }
}
