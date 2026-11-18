mod closing;
pub mod constants;
mod corner_motion;
mod corroboration;
mod dispatch;
mod evidence;
mod handoff;
mod moving;
mod radar_moving;
pub mod source;
mod state;
mod stationary;
pub(crate) mod stationary_geometry;
mod stationary_support;

use crate::{
    model::VisionLead,
    point::{Identity, Point, TrackId},
};
use indexmap::IndexMap;
use serde::Serialize;

#[derive(Clone, Copy)]
pub struct Frame<'a> {
    pub vision: Option<VisionLead>,
    pub points: &'a [Point],
    pub path: &'a crate::path::Path,
    pub time: Option<f64>,
    pub prefer_corner: bool,
    pub prefer_primary: bool,
    pub yaw_rate: f64,
    pub allowed_output_sources: Option<&'a std::collections::HashSet<String>>,
}

#[derive(Clone, Debug, serde::Deserialize, Serialize)]
pub struct VisionMatch {
    pub point: Point,
    pub probability: f64,
    pub score: f64,
    pub d_path: f64,
}

#[derive(Clone, Serialize)]
pub struct PositionEvidence {
    pub since_s: f64,
    pub time_s: f64,
    pub point: Point,
    pub anchor_frames: usize,
    pub anchor_time_s: Option<f64>,
    pub position_since_s: Option<f64>,
    pub position_frames: usize,
}

#[derive(Default, Serialize)]
pub struct Matcher {
    #[serde(serialize_with = "entries")]
    pub _closing_range_history: IndexMap<Identity, (f64, f64, Point)>,
    pub closing_range_match: Option<VisionMatch>,
    #[serde(serialize_with = "entries")]
    pub _corner_motion_history: IndexMap<Identity, std::collections::VecDeque<(f64, Point)>>,
    #[serde(serialize_with = "entries")]
    pub _corner_motion_unreliable_until: IndexMap<Identity, f64>,
    pub last_identity: Option<Identity>,
    pub low_probability_hold_frames: usize,
    pub stationary_identity: Option<Identity>,
    pub _stationary_pending_identity: Option<Identity>,
    pub _stationary_pending_since_s: Option<f64>,
    pub _stationary_pending_vision_support_frames: usize,
    pub _stationary_pending_last_support_time_s: Option<f64>,
    pub _stationary_last_point: Option<Point>,
    pub _stationary_last_time_s: Option<f64>,
    pub _stationary_seed_probability: f64,
    pub _stationary_seed_score: f64,
    pub _stationary_path_outlier_since_s: Option<f64>,
    pub _stationary_front_departure_since_s: Option<f64>,
    #[serde(serialize_with = "entries")]
    pub _observed_since_s: IndexMap<Identity, f64>,
    #[serde(serialize_with = "entries")]
    pub _observed_last_s: IndexMap<Identity, f64>,
    #[serde(serialize_with = "entries")]
    pub _stationary_front_evidence: IndexMap<Identity, PositionEvidence>,
    #[serde(serialize_with = "entries")]
    pub _moving_vision_evidence: IndexMap<Identity, PositionEvidence>,
    pub _stationary_corner_supported: bool,
    pub _stationary_weak_pair_identity: Option<(TrackId, String, TrackId)>,
    pub _stationary_weak_pair_last_vision_time_s: Option<f64>,
    pub _stationary_pending_weak_pair_supported: bool,
    pub _stationary_closer_challenger_identity: Option<Identity>,
    pub _stationary_closer_challenger_since_s: Option<f64>,
    pub _stationary_closer_challenger_last_point: Option<Point>,
    pub _stationary_closer_challenger_last_time_s: Option<f64>,
    pub _close_xyv_since_s: Option<f64>,
    /// Held track, the near point and its time at the last close x/y/v match.
    pub _close_xyv_previous: Option<(TrackId, Point, f64)>,
    pub _vision_fallback: Option<VisionLead>,
    pub _vision_fallback_hold_frames: usize,
    pub radar_only_moving_identity: Option<Identity>,
    pub _radar_only_moving_pending_identity: Option<Identity>,
    pub _radar_only_moving_pending_since_s: Option<f64>,
    pub _radar_only_moving_pending_start_d_rel: Option<f64>,
    pub _radar_only_moving_integrated_v_rel_m: f64,
    pub _radar_only_moving_last_point: Option<Point>,
    pub _radar_only_moving_last_time_s: Option<f64>,
    pub _radar_only_moving_challenger_identity: Option<Identity>,
    pub _radar_only_moving_challenger_since_s: Option<f64>,
    pub _radar_only_moving_challenger_last_point: Option<Point>,
    pub _radar_only_moving_challenger_last_time_s: Option<f64>,
    pub _far_vision_radar_identity: Option<Identity>,
    pub _far_vision_radar_since_s: Option<f64>,
    pub _far_vision_radar_last_point: Option<Point>,
    pub _far_vision_radar_last_time_s: Option<f64>,
    pub _rejected_radar_only_moving_identity: Option<Identity>,
    pub _rejected_radar_only_moving_last_point: Option<Point>,
    pub _rejected_radar_only_moving_last_time_s: Option<f64>,
}

pub fn entries<S: serde::Serializer, K: Serialize, V: Serialize>(
    values: &IndexMap<K, V>,
    serializer: S,
) -> Result<S::Ok, S::Error> {
    use serde::ser::SerializeMap;
    let mut map = serializer.serialize_map(Some(usize::from(!values.is_empty())))?;
    if !values.is_empty() {
        map.serialize_entry("entries", &values.iter().collect::<Vec<_>>())?;
    }
    map.end()
}
