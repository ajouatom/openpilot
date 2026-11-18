#[derive(Default)]
pub struct CarState {
    pub speed: f64,
    pub accel: f64,
    pub steer_angle: f64,
    pub steer_rate: f64,
    pub steer_torque: f64,
    pub steering_pressed: bool,
    pub can_valid: bool,
    pub can_timeout: bool,
    pub drive: bool,
    pub parking_brake: bool,
    pub brake_hold: bool,
    pub driving_gear: bool,
    pub lat_enabled: bool,
    pub fault_temporary: bool,
    pub fault_permanent: bool,
    pub standstill: bool,
    pub brake: bool,
    pub gas: bool,
    pub soft_hold: bool,
    pub carrot_cruise: bool,
    pub cruise_enabled: bool,
    pub cruise_standstill: bool,
    pub cruise: f64,
    pub cluster: f64,
    pub cluster_ratio: f64,
    pub steer_curvature: f64,
}
pub use openpilot_control_policy::lateral_readiness::LiveParameters;
#[derive(Default)]
pub struct TorqueParameters {
    pub use_params: bool,
    pub factor: f32,
    pub offset: f32,
    pub friction: f32,
}
#[derive(Default)]
pub struct Selfdrive {
    pub current: bool,
    pub enabled: bool,
    pub active: bool,
    pub soft_disabling: bool,
    pub personality: u16,
    pub visual_alert: u16,
    /// `alertType` ("eventName/eventType"), for the CCNC cluster texts.
    pub alert_type: String,
}
#[derive(Default)]
pub struct Model {
    pub desired_curvature: f64,
    pub jetlink_latched: bool,
    pub jetlink: bool,
    pub lane_change: bool,
    pub lane_left: bool,
    pub lane_right: bool,
    /// meta.assistantLaneSignalId / Route (carrot voice_indicator).
    pub assistant_id: String,
    pub assistant_route: String,
    pub desire: Vec<f64>,
    pub orientation_x: Vec<f64>,
    pub orientation_y: Vec<f64>,
    pub acceleration_y: Vec<f64>,
    /// Ego lane lines (laneLines[1] left, [2] right) and the plan, for
    /// laneless centring.
    pub lane_x: Vec<f64>,
    pub left_y: Vec<f64>,
    pub right_y: Vec<f64>,
    pub left_prob: f64,
    pub right_prob: f64,
    pub path_x: Vec<f64>,
    pub path_y: Vec<f64>,
}
#[derive(Default)]
pub struct LongPlan {
    pub acceleration: f64,
    pub speed: f64,
    pub jerk: f64,
    pub stop: bool,
    pub speeds: Vec<f64>,
    pub has_lead: bool,
    pub x_state: i32,
    pub coast_target: f64,
    pub coast_percent: u8,
    pub cruise_source: bool,
    pub fcw: bool,
    pub cruise_target: f64,
    /// `trafficState` (0 off, 1 red, 2 green) and `desiredDistance` (m).
    pub traffic_state: i32,
    pub desired_distance: f64,
}
#[derive(Default)]
pub struct LateralPlan {
    pub lane_lines: bool,
    pub solution_valid: bool,
    pub psis: Vec<f64>,
    pub curvatures: Vec<f64>,
    pub distances: Vec<f64>,
}
#[derive(Default)]
pub struct Lead {
    pub status: bool,
    pub distance: f64,
    pub relative_speed: f64,
    pub radar: bool,
    pub path: f64,
}
#[derive(Default)]
pub struct Radar {
    pub lead: Lead,
    pub lead_two: bool,
    pub cut_in: bool,
}
#[derive(Default)]
pub struct Carrot {
    pub desired: f64,
    pub turn_speed: f64,
    pub active: i32,
    pub turn_distance: f64,
    pub speed_type: i32,
    pub speed_limit: f64,
    pub turn_type: String,
    pub road_limit: f64,
    pub turn_info: i32,
    pub sdi: String,
    pub desired_source: String,
}
#[derive(Default)]
pub struct Pose {
    pub orientation: [f64; 3],
    pub angular: [f64; 3],
}
#[derive(Default)]
pub struct Inputs {
    pub startup: openpilot_control_policy::lateral_readiness::State,
    pub car: CarState,
    pub live: LiveParameters,
    pub torque: TorqueParameters,
    pub selfdrive: Selfdrive,
    pub model: Model,
    pub longitudinal: LongPlan,
    pub lateral: LateralPlan,
    pub radar: Radar,
    pub carrot: Carrot,
    pub output_torque: f64,
    pub output_angle: f64,
    pub output_curvature: f64,
    pub delay: f64,
    pub override_long: bool,
    pub vehicle_fault: bool,
    pub distracted: bool,
    pub monitoring_disabled: bool,
    pub assistance_valid: bool,
    pub left_depart: bool,
    pub right_depart: bool,
    pub torque_checks: bool,
    pub model_checks: bool,
    pub carrot_fresh: bool,
    pub stop_object_inputs: bool,
    pub driver_monitoring_alert: u8,
    pub center_to_front_ratio: f64,
    pub plan_age: f64,
    pub frame: i64,
    pub long_time: u64,
    pub model_time: u64,
    pub calibration: Option<[f64; 3]>,
    pub pose: Option<Pose>,
}

impl Inputs {
    pub fn startup_inputs(&self) -> openpilot_control_policy::lateral_readiness::Inputs<'_> {
        use openpilot_control_policy::lateral_readiness::{Car, Inputs as StartupInputs, Lane};
        StartupInputs {
            state: &self.startup,
            live: &self.live,
            car: Car {
                can_valid: self.car.can_valid,
                fault_temporary: self.car.fault_temporary,
                fault_permanent: self.car.fault_permanent,
                values: [
                    self.car.speed,
                    self.car.steer_angle,
                    self.car.steer_rate,
                    self.car.steer_torque,
                ],
            },
            desired_curvature: self.model.desired_curvature,
            lane: Lane {
                use_lane_lines: self.lateral.lane_lines,
                solution_valid: self.lateral.solution_valid,
                psis: &self.lateral.psis,
                curvatures: &self.lateral.curvatures,
                distances: &self.lateral.distances,
            },
        }
    }
}
