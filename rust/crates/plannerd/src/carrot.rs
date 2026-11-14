use crate::{
    car_state::CarState,
    driving_mode::{DrivingMode, DrivingModeDetector},
    following::{FollowingTime, GapParameters},
    lane_change_gap::{Plan, Tracker},
    model::Model,
    parameters::Parameters,
    radar::Radar,
    traffic_stop::TrafficStopModelLeadMatcher,
    types::{Personality, PlannerMode},
    window::Window,
    Error,
};
use openpilot_cereal::log_capnp::onroad_event::EventName;
use serde::{Deserialize, Serialize};

mod config;
mod desire;
mod navigation;
mod signal;
mod traffic;
mod update;
pub use config::Config;
pub use signal::{PrepareRead, SignalSource, Signals, StartRead};

#[derive(Clone, Copy, Debug, Deserialize, Serialize, PartialEq, Eq)]
#[serde(rename_all = "camelCase")]
pub enum XState {
    Lead,
    Cruise,
    E2eCruise,
    E2eStop,
    E2ePrepare,
    E2eStopped,
    E2eStopPrepare,
}

#[derive(Clone, Copy, Debug, Deserialize, Serialize, PartialEq, Eq)]
#[serde(rename_all = "lowercase")]
pub enum TrafficState {
    Off,
    Red,
    Green,
}

#[derive(Clone, Debug, Default, Deserialize)]
#[serde(rename_all = "camelCase", default, deny_unknown_fields)]
pub struct Navigation {
    pub atc_type: String,
    pub traffic_state: i32,
    pub active_carrot: i32,
    pub x_dist_to_turn: f64,
    pub desired_speed: f64,
}

pub struct Input<'a> {
    pub car: &'a CarState,
    pub model: &'a Model,
    pub radar: &'a Radar,
    pub personality: Personality,
    pub navigation: Option<&'a Navigation>,
    pub navigation_receive_time: Option<f64>,
    pub mode_checks: bool,
    pub lane_valid: bool,
    pub model_ns: u64,
    pub radar_ns: u64,
    pub pose_ns: u64,
    pub pose_valid: bool,
    pub yaw_rate: f64,
}

#[derive(Debug)]
pub struct CarrotPlanner {
    pub config: Config,
    pub frame: u64,
    params_count: u32,
    pub driving_mode: DrivingMode,
    driving_mode_last: DrivingMode,
    disable_auto: bool,
    detector: DrivingModeDetector,
    pub safe_factor: f64,
    pub follow_factor: f64,
    pub lead_response: i32,
    pub following: FollowingTime,
    pub traffic_state: TrafficState,
    pub x_state: XState,
    stop_filter: Window<3>,
    stop_filter2: Window<15>,
    velocity_filter: Window<10>,
    pub fake_cruise_distance: f64,
    pub x_stop: f64,
    pub actual_stop_distance: f64,
    stopping_count: u32,
    traffic_starting_count: u32,
    pub user_stop_distance: f64,
    start_sign_count: u64,
    stop_sign_count: u64,
    pub comfort_brake: f64,
    pub soft_hold_active: i32,
    pub lane_change_active: bool,
    pub lane_change_gap: Plan,
    lane_change_tracker: Tracker,
    lane_change_model_ns: u64,
    pub desire_state: f64,
    pub desire_state_count: u64,
    traffic_state_carrot: i32,
    carrot_stay_stop: bool,
    pub eco_target_speed: f64,
    pub active_carrot: i32,
    pub distance_to_turn: f64,
    pub atc_type: String,
    pub atc_active: bool,
    stop_x_rate_limited: Option<f64>,
    matcher: TrafficStopModelLeadMatcher,
    pub traffic_stop_model_lead_offset: f64,
    last_event_time: f64,
    pub events: Vec<EventName>,
    pub cruise_speed: f64,
    pub stop_distance: f64,
    pub mode: PlannerMode,
    pub signal_detection_suspended: bool,
    pub signal_observation_color: u8,
    pub signal_green_active: bool,
    pub signal_lead_present: bool,
    pub start_decision: Option<crate::signal::start::Decision>,
    pub prepare_decision: Option<crate::signal::prepare::Decision>,
    pub prepare_control_active: bool,
    start_guard: crate::signal::start::Guard,
    signal_stop_owned: bool,
    stop_prepare: crate::signal::prepare::Prepare,
}

impl CarrotPlanner {
    pub fn new(parameters: &mut impl Parameters) -> Result<Self, Error> {
        let driving_mode = DrivingMode::from_setting(parameters.integer("MyDrivingMode")?);
        let config = Config {
            automatic_mode: parameters.integer("MyDrivingModeAuto")?,
            ..Config::default()
        };
        Ok(Self {
            config,
            frame: 0,
            params_count: 0,
            driving_mode,
            driving_mode_last: driving_mode,
            disable_auto: false,
            detector: DrivingModeDetector::default(),
            safe_factor: 1.,
            follow_factor: 1.,
            lead_response: 0,
            following: FollowingTime::default(),
            traffic_state: TrafficState::Off,
            x_state: XState::Cruise,
            stop_filter: Window::default(),
            stop_filter2: Window::default(),
            velocity_filter: Window::default(),
            fake_cruise_distance: 0.,
            x_stop: 0.,
            actual_stop_distance: 0.,
            stopping_count: 0,
            traffic_starting_count: 0,
            user_stop_distance: -1.,
            start_sign_count: 0,
            stop_sign_count: 0,
            comfort_brake: 2.4,
            soft_hold_active: 0,
            lane_change_active: false,
            lane_change_gap: Plan::default(),
            lane_change_tracker: Tracker::default(),
            lane_change_model_ns: 0,
            desire_state: 0.,
            desire_state_count: 0,
            traffic_state_carrot: 0,
            carrot_stay_stop: false,
            eco_target_speed: 0.,
            active_carrot: 0,
            distance_to_turn: 0.,
            atc_type: String::new(),
            atc_active: false,
            stop_x_rate_limited: None,
            matcher: TrafficStopModelLeadMatcher::default(),
            traffic_stop_model_lead_offset: 0.,
            last_event_time: 0.,
            events: Vec::new(),
            cruise_speed: 0.,
            stop_distance: 0.,
            mode: PlannerMode::Acc,
            signal_detection_suspended: false,
            signal_observation_color: 0,
            signal_green_active: false,
            signal_lead_present: false,
            start_decision: None,
            prepare_decision: None,
            prepare_control_active: false,
            start_guard: crate::signal::start::Guard::default(),
            signal_stop_owned: false,
            stop_prepare: crate::signal::prepare::Prepare::default(),
        })
    }

    /// `teslaTrafficStopTakeover` / `teslaTrafficStopWarning`, added every
    /// cycle as carrot's `events.add` does.
    pub fn add_tesla_event(&mut self, takeover: bool, _clock: &mut impl FnMut() -> f64) {
        let event = if takeover {
            EventName::TeslaTrafficStopTakeover
        } else {
            EventName::TeslaTrafficStopWarning
        };
        if !self.events.contains(&event) {
            self.insert_event(event);
        }
    }

    fn add_event(&mut self, event: EventName, clock: &mut impl FnMut() -> f64) {
        let now = clock();
        if now - self.last_event_time > 5. {
            self.insert_event(event);
            self.last_event_time = now;
        }
    }

    fn insert_event(&mut self, event: EventName) {
        let index = self
            .events
            .partition_point(|value| u16::from(*value) <= u16::from(event));
        self.events.insert(index, event);
    }
    fn add_signal_event(&mut self, event: EventName, clock: &mut impl FnMut() -> f64) {
        if !self.signal_lead_present {
            self.add_event(event, clock);
        }
    }
    fn insert_signal_event(&mut self, event: EventName) {
        if !self.signal_lead_present {
            self.insert_event(event);
        }
    }
}

impl<'a> Input<'a> {
    pub fn current_navigation(&self, clock: &mut impl FnMut() -> f64) -> Option<&'a Navigation> {
        let navigation = self.navigation?;
        if let Some(received) = self.navigation_receive_time {
            let age = clock() - received;
            if !(0. ..=1.).contains(&age) {
                return None;
            }
        }
        (0. < navigation.desired_speed && navigation.desired_speed <= 250.).then_some(navigation)
    }
}

#[cfg(test)]
mod signal_refresh_tests {
    use super::*;
    struct Settings;
    impl Parameters for Settings {
        fn integer(&mut self, key: &'static str) -> Result<i32, Error> {
            Ok(if key == "MyDrivingMode" { 3 } else { 0 })
        }
        fn float(&mut self, _: &'static str) -> Result<f64, Error> {
            Ok(0.)
        }
    }
    #[test]
    fn lead_mutes_only_signal_notices_without_consuming_cooldown() {
        let mut planner = CarrotPlanner::new(&mut Settings).unwrap();
        planner.signal_lead_present = true;
        let mut calls = 0;
        planner.add_signal_event(EventName::TrafficSignGreen, &mut || {
            calls += 1;
            10.
        });
        planner.insert_signal_event(EventName::AudioRefuse);
        assert!(planner.events.is_empty());
        assert_eq!(calls, 0);
        planner.signal_lead_present = false;
        planner.add_signal_event(EventName::TrafficSignGreen, &mut || 10.);
        planner.insert_signal_event(EventName::AudioRefuse);
        assert!(planner.events.contains(&EventName::TrafficSignGreen));
        assert!(planner.events.contains(&EventName::AudioRefuse));
    }
    #[test]
    fn lead_handover_requires_positive_finite_range_and_strict_stop_boundary() {
        for (present, distance, stop, expected) in [
            (true, 11.999, 10., true),
            (true, 12., 10., false),
            (true, 0., 10., false),
            (true, -1., 10., false),
            (false, 5., 10., false),
            (true, f64::NAN, 10., false),
            (true, 5., f64::INFINITY, false),
            (true, f64::INFINITY, 10., false),
        ] {
            assert_eq!(
                super::signal::lead_in_stop_range(present, distance, stop),
                expected
            );
        }
    }
    #[test]
    fn following_lead_temporarily_removes_veto_without_erasing_red_memory() {
        use crate::signal::transport::Context;
        let mut planner = CarrotPlanner::new(&mut Settings).unwrap();
        planner.x_state = XState::E2eStopped;
        let car = CarState::default();
        let model = Model::default();
        let radar = Radar::default();
        let input = Input {
            car: &car,
            model: &model,
            radar: &radar,
            personality: Personality::Standard,
            navigation: None,
            navigation_receive_time: None,
            mode_checks: false,
            lane_valid: false,
            model_ns: 0,
            radar_ns: 0,
            pose_ns: 0,
            pose_valid: false,
            yaw_rate: 0.,
        };
        for i in 0..=4 {
            let t = f64::from(i) * 0.2;
            let read = StartRead {
                context: Context {
                    now: t,
                    enabled: true,
                    valid: true,
                    drive: true,
                },
                observation: Some(
                    serde_json::json!({"timestamp":t,"frame_id":i,"session":"camera","tracks":[{"box":[640,250,680,270],"raw":"red"}]}),
                ),
            };
            planner.signal_start(&input, Some(&read), false);
        }
        assert!(planner.start_decision.as_ref().unwrap().hold);
        for (t, following) in [(1., true), (1.2, true), (1.4, false)] {
            let read = StartRead {
                context: Context {
                    now: t,
                    enabled: true,
                    valid: true,
                    drive: true,
                },
                observation: Some(
                    serde_json::json!({"timestamp":t,"frame_id":t*1000.,"session":"camera","tracks":[{"box":[640,250,680,270],"raw":"unknown"}]}),
                ),
            };
            planner.signal_start(&input, Some(&read), following);
            let decision = planner.start_decision.as_ref().unwrap();
            assert_eq!(decision.hold, !following);
            assert!(!decision.released && !decision.green_confirmed);
            assert_eq!(planner.start_guard.snapshot()["hold"], true);
            if following {
                assert!(!decision.warn);
                assert_eq!(planner.start_guard.snapshot()["warned"], false);
            }
        }
    }
}
