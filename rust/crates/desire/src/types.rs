use serde::{Deserialize, Serialize};

pub const DT: f64 = 0.05;
pub const MIN_SPEED: f64 = 30.0 / 3.6;
/// A navigation lane change toward a turn may still start below `MIN_SPEED`,
/// down to this speed, once the turn pocket's lane exists (see `helper.rs`).
pub const POCKET_MIN_SPEED: f64 = 10.0 / 3.6;

#[derive(Clone, Copy, Debug, Default, Deserialize, Serialize, PartialEq, Eq)]
pub enum State {
    #[default]
    Off,
    PreLaneChange,
    Starting,
    Finishing,
}

#[derive(Clone, Copy, Debug, Default, Deserialize, Serialize, PartialEq, Eq)]
pub enum Maneuver {
    #[default]
    None,
    Turn,
    LaneChange,
}

#[derive(Clone, Debug, Default, Deserialize, Serialize)]
pub struct Config {
    pub need_torque: i32,
    pub bsd: i32,
    pub line_check: i32,
    pub delay_tenths: f64,
    /// BlinkerLatchedTurn in m/s, 0 = off.
    pub latched_turn: f64,
    pub lever_wait: bool,
}

/// Physical-indicator observer switches, read every 20 frames on supported
/// cars (`lamp_mode` and the four observer settings).
#[derive(Clone, Copy, Debug, Default, Deserialize, Serialize)]
pub struct IndicatorConfig {
    pub boundary: bool,
    pub manual_turn: bool,
    pub model_desire: bool,
    pub model_path: bool,
}

#[derive(Clone, Debug, Deserialize, Serialize)]
#[serde(default)]
pub struct Car {
    pub can_valid: bool,
    pub left_blinker: bool,
    pub right_blinker: bool,
    pub v_ego: f64,
    pub a_ego: f64,
    pub trailer_connected: bool,
    pub steering_torque: f64,
    pub steering_pressed: bool,
    pub steering_angle_deg: f64,
    pub left_lane_line: i32,
    pub right_lane_line: i32,
    pub left_blindspot: bool,
    pub right_blindspot: bool,
    pub blinker_lever: u8,
    pub blinker_lamp: u8,
    pub stalk_counts: [u8; 2],
    /// `gearShifter == drive`.
    pub drive: bool,
    pub steer_fault_temporary: bool,
    pub steer_fault_permanent: bool,
    /// OEM lane type observation time (ns); zero means unknown.
    pub lane_line_info_mono_time: u64,
    pub cruise_cancel_state: bool,
    /// Tesla automatic turn signals.
    pub turn_signal_driver_valid: bool,
    pub turn_signal_driver_direction: u8,
    pub auto_turn_signal_ready_request_id: u64,
    pub vehicle_sensors_invalid: bool,
    pub standstill: bool,
}

impl Default for Car {
    fn default() -> Self {
        Self {
            can_valid: true,
            left_blinker: false,
            right_blinker: false,
            v_ego: 20.0,
            a_ego: 0.0,
            trailer_connected: false,
            steering_torque: 0.0,
            steering_pressed: false,
            steering_angle_deg: 0.0,
            left_lane_line: 0,
            right_lane_line: 0,
            left_blindspot: false,
            right_blindspot: false,
            blinker_lever: 0,
            blinker_lamp: 0,
            stalk_counts: [0, 0],
            drive: true,
            steer_fault_temporary: false,
            steer_fault_permanent: false,
            lane_line_info_mono_time: 0,
            cruise_cancel_state: false,
            turn_signal_driver_valid: false,
            turn_signal_driver_direction: 0,
            auto_turn_signal_ready_request_id: 0,
            vehicle_sensors_invalid: false,
            standstill: false,
        }
    }
}

#[derive(Clone, Debug, Deserialize, Serialize)]
#[serde(default)]
pub struct Model {
    pub lane_lines: [Vec<f64>; 4],
    pub lane_line_probs: [f64; 4],
    pub road_edges: [Vec<f64>; 2],
    pub desire_state: [f64; 8],
    pub orientation_rate_z: Vec<f64>,
    /// Forward distances of each lane line (`laneLines[i].x`).
    pub lane_line_x: [Vec<f64>; 4],
    /// Planned centre path (`position.t/x/y`).
    pub position_t: Vec<f64>,
    pub position_x: Vec<f64>,
    pub position_y: Vec<f64>,
    pub frame_id: u32,
    pub road_edge_stds: [f64; 2],
    /// `meta.desirePrediction`: 4 horizons x 8 desires.
    pub desire_prediction: Vec<f64>,
}

impl Default for Model {
    fn default() -> Self {
        Self {
            lane_lines: [-5.4, -1.8, 1.8, 5.4].map(|value| vec![value; 33]),
            lane_line_probs: [1.0; 4],
            road_edges: [-7.2, 7.2].map(|value| vec![value; 33]),
            desire_state: [0.0; 8],
            orientation_rate_z: vec![0.0; 33],
            lane_line_x: std::array::from_fn(|_| (0..33).map(|i| f64::from(i) * 3.0).collect()),
            position_t: Vec::new(),
            position_x: Vec::new(),
            position_y: Vec::new(),
            frame_id: 0,
            road_edge_stds: [0.; 2],
            desire_prediction: Vec::new(),
        }
    }
}

#[derive(Clone, Debug, Deserialize, Serialize)]
#[serde(default)]
pub struct Lead {
    pub status: bool,
    pub d_rel: f64,
    pub v_rel: Option<f64>,
    pub v_lead: Option<f64>,
    pub radar_track_id: i64,
}

impl Default for Lead {
    fn default() -> Self {
        Self {
            status: false,
            d_rel: 255.0,
            v_rel: None,
            v_lead: None,
            radar_track_id: -1,
        }
    }
}

impl Lead {
    pub fn v_relative(&self, v_ego: f64) -> f64 {
        self.v_rel.unwrap_or(self.v_lead.unwrap_or(v_ego) - v_ego)
    }

    pub fn corner(&self) -> bool {
        self.status
            && ((200..220).contains(&self.radar_track_id)
                || (240..250).contains(&self.radar_track_id)
                || (300..412).contains(&self.radar_track_id)
                || self.radar_track_id >= 1000)
    }
}

#[derive(Clone, Debug, Default, Deserialize, Serialize)]
#[serde(default)]
pub struct Navigation {
    pub atc_type: String,
    pub command_index: i64,
    pub command: String,
    pub argument: String,
    pub x_turn_info: i32,
    pub x_dist_to_turn: f64,
}

#[derive(Clone, Debug, Deserialize, Serialize)]
#[serde(default)]
pub struct Input {
    pub car: Car,
    pub model: Model,
    pub navigation: Navigation,
    pub leads: [Lead; 2],
    pub objects: [Vec<Lead>; 2],
    pub lateral_active: bool,
    pub lane_change_prob: f64,
    pub indicator: Indicator,
}

/// Inputs of the physical-indicator observers (carrot's `indicator_*`
/// arguments of `DesireHelper.update`).
#[derive(Clone, Debug, Default, Deserialize, Serialize)]
#[serde(default)]
pub struct Indicator {
    /// `time.monotonic()` seconds.
    pub now: f64,
    /// modelV2 valid and carState/carControl fresh.
    pub valid: bool,
    /// `carControl.actuators.curvature`.
    pub curvature: Option<f64>,
    /// Fresh, valid `livePose.angularVelocityDevice.z`.
    pub yaw_rate: Option<f64>,
    /// `physical-hold.json`, read only while an observation is active.
    #[serde(skip)]
    pub feedback: Option<serde_json::Value>,
    /// `boundary-indicator.json` `(time, mask)`, read every 20 frames.
    pub journal: Option<(f64, i64)>,
    /// Observer switches, read every 20 frames.
    pub config: Option<IndicatorConfig>,
    /// `valid` plus fresh radarState: voice-owned lamp masks expire on it.
    pub assistant_valid: bool,
    /// Tesla `turn_signal_context`: a stopped turn may keep its lamp.
    pub turn_hold_allowed: bool,
    /// Tesla `turn_signal_context` heading `(logMonoTime ns, NED z)`.
    pub turn_pose: Option<(u64, f64)>,
    /// `CLOCK_BOOTTIME` seconds for App request deadlines.
    pub boot: f64,
}

impl Default for Input {
    fn default() -> Self {
        Self {
            car: Car::default(),
            model: Model::default(),
            navigation: Navigation::default(),
            leads: std::array::from_fn(|_| Lead::default()),
            objects: std::array::from_fn(|_| Vec::new()),
            lateral_active: true,
            lane_change_prob: 0.1,
            indicator: Indicator::default(),
        }
    }
}

#[derive(Debug, thiserror::Error)]
#[error("desire input requires 33-point lane/edge geometry and 16 orientation samples")]
pub struct InvalidModel;

impl Input {
    pub fn validate(&self) -> Result<(), InvalidModel> {
        if self
            .model
            .lane_lines
            .iter()
            .chain(&self.model.road_edges)
            .any(|values| values.len() != 33)
            || self.model.orientation_rate_z.len() < 16
        {
            return Err(InvalidModel);
        }
        Ok(())
    }
}

pub(crate) fn minimum(first: f64, second: f64) -> f64 {
    if second < first {
        second
    } else {
        first
    }
}

pub(crate) fn maximum(first: f64, second: f64) -> f64 {
    if second > first {
        second
    } else {
        first
    }
}
