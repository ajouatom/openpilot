use crate::{
    indicator::{
        boundary::BoundaryIndicator,
        lane::{adjacent_lane, MANUAL_INDICATOR_MIN_SPEED},
        observer::{ModelIndicator, ObserverInput},
    },
    lever::{
        lever_maneuver, LaneCrossing, LeverChoice, LeverForcedTurn, LeverRequestWait,
        LEVER_REQUEST_WAIT_S, STALK_QUEUE_GUARD_S,
    },
    side::SideState,
    types::{
        maximum, minimum, Config, IndicatorConfig, Input, InvalidModel, Maneuver, Navigation,
        State, DT, MIN_SPEED, POCKET_MIN_SPEED,
    },
};
use serde::Serialize;

#[derive(Serialize)]
pub struct DesireHelper {
    pub frame: u64,
    pub lane_change_state: State,
    pub lane_change_direction: u8,
    pub lane_change_timer: f64,
    pub lane_change_ll_prob: f64,
    pub lane_change_delay: f64,
    pub maneuver_type: Maneuver,
    pub desire: u8,
    pub turn_direction: u8,
    pub enable_turn_desires: bool,
    pub turn_desire_state: bool,
    pub desire_disable_count: u32,
    pub turn_disable_count: u32,
    pub left: SideState,
    pub right: SideState,
    pub blinker_ignore: bool,
    pub driver_blinker_state: u8,
    pub carrot_blinker_state: u8,
    pub carrot_lane_change_count: u32,
    pub carrot_cmd_index_last: i64,
    pub atc_type: String,
    pub atc_active: u8,
    pub auto_lane_change_enable: bool,
    pub next_lane_change: bool,
    pub keep_pulse_timer: f64,
    pub config: Config,
    pub prev_desire_enabled: bool,
    pub desire_log: String,
    pub lane_change_available_left: bool,
    pub lane_change_available_right: bool,
    #[serde(skip)]
    stalk_counts: Option<[u8; 2]>,
    #[serde(skip)]
    queued_lane_change: u8,
    #[serde(skip)]
    finishing_timer: f64,
    #[serde(skip)]
    queued_wait: f64,
    #[serde(skip)]
    lever_wait: LeverRequestWait,
    #[serde(skip)]
    lever_forced_turn: LeverForcedTurn,
    #[serde(skip)]
    crossing: LaneCrossing,
    /// modelV2.meta.laneChangeBlinkerHold
    #[serde(skip)]
    pub blinker_hold_direction: u8,
    /// modelV2.meta.leverTurn: the turn desire comes from a latched lever.
    #[serde(skip)]
    pub lever_turn: bool,
    /// Physical-indicator observers (carrot `boundary_indicator_supported`:
    /// any non-passive car, or a Kick car).
    #[serde(skip)]
    pub indicator_supported: bool,
    /// carrot `kick_supported`: the panda may kick the lamp itself
    /// (HyundaiSafetyFlags.CANFD_BLINKER_KICK); the lamp is then read through
    /// the verified lever while it is masked.
    #[serde(skip)]
    pub kick_supported: bool,
    #[serde(skip)]
    pub indicator_config: IndicatorConfig,
    #[serde(skip)]
    pub boundary_indicator: BoundaryIndicator,
    #[serde(skip)]
    pub model_indicator: ModelIndicator,
    #[serde(skip)]
    pub indicator_journal_time: f64,
    /// modelV2.meta.autoTurnSignal: an ATC-originated maneuver only.
    #[serde(skip)]
    pub auto_turn_signal: u8,
    #[serde(skip)]
    pub auto_turn_signal_lane_change: bool,
    /// Tesla automatic turn signals (carrot `tesla_auto_signals`).
    #[serde(skip)]
    pub tesla: Tesla,
    /// Voice lane-change intent (carrot `AssistantLane`).
    #[serde(skip)]
    pub assistant: crate::indicator::assistant::AssistantLane,
}

/// The Tesla automatic-signal state of carrot's DesireHelper.
#[derive(Clone, Debug, Default)]
pub struct Tesla {
    pub enabled: bool,
    /// modelV2.meta.autoTurnSignalDirection / RequestId.
    pub signal_direction: u8,
    pub signal_request_id: u64,
    counter: u64,
    lane_completed: bool,
    lane_direction: u8,
    app_direction: u8,
    app_wait: f64,
    app_deadline: f64,
    atc_type: String,
    driver_counts: Option<[u8; 2]>,
    manual_direction: u8,
    manual_grace: f64,
    journey: crate::indicator::tesla_turn::TeslaTurnJourney,
}

impl Default for DesireHelper {
    fn default() -> Self {
        Self {
            frame: 0,
            lane_change_state: State::Off,
            lane_change_direction: 0,
            lane_change_timer: 0.0,
            lane_change_ll_prob: 1.0,
            lane_change_delay: 0.0,
            maneuver_type: Maneuver::None,
            desire: 0,
            turn_direction: 0,
            enable_turn_desires: true,
            turn_desire_state: false,
            desire_disable_count: 0,
            turn_disable_count: 0,
            left: SideState::new("left"),
            right: SideState::new("right"),
            blinker_ignore: false,
            driver_blinker_state: 0,
            carrot_blinker_state: 0,
            carrot_lane_change_count: 0,
            carrot_cmd_index_last: 0,
            atc_type: String::new(),
            atc_active: 0,
            auto_lane_change_enable: false,
            next_lane_change: false,
            keep_pulse_timer: 0.0,
            config: Config::default(),
            prev_desire_enabled: false,
            desire_log: String::new(),
            lane_change_available_left: false,
            lane_change_available_right: false,
            stalk_counts: None,
            queued_lane_change: 0,
            finishing_timer: 0.0,
            queued_wait: 0.0,
            lever_wait: LeverRequestWait::default(),
            lever_forced_turn: LeverForcedTurn::default(),
            crossing: LaneCrossing::default(),
            blinker_hold_direction: 0,
            lever_turn: false,
            indicator_supported: false,
            kick_supported: false,
            indicator_config: IndicatorConfig::default(),
            boundary_indicator: BoundaryIndicator::new(
                None,
                adjacent_lane,
                MANUAL_INDICATOR_MIN_SPEED,
            ),
            model_indicator: ModelIndicator::default(),
            indicator_journal_time: 0.,
            auto_turn_signal: 0,
            auto_turn_signal_lane_change: false,
            tesla: Tesla::default(),
            assistant: crate::indicator::assistant::AssistantLane::default(),
        }
    }
}

impl DesireHelper {
    /// carrot's `DesireHelper(boundary_indicator_supported=...)`; `journal`
    /// is `boundary-indicator.json` as `(time, direction, mask)`.
    pub fn new(supported: bool, journal: Option<(f64, i64, Option<i64>)>) -> Self {
        Self {
            indicator_supported: supported,
            boundary_indicator: BoundaryIndicator::new(
                journal.filter(|_| supported),
                adjacent_lane,
                MANUAL_INDICATOR_MIN_SPEED,
            ),
            ..Self::default()
        }
    }

    /// carrot `DesireHelper(kick_supported=...)`: also runs the observers.
    pub fn with_kick(mut self, kick: bool) -> Self {
        self.kick_supported = kick;
        self
    }

    /// carrot `DesireHelper(tesla_auto_signals=...)`; `counter_ns` seeds the
    /// request id as `time.monotonic_ns()` does.
    /// carrot `AssistantLane(fingerprint=..., native_signal=...)`.
    pub fn with_assistant(mut self, fingerprint: &str, native_signal: bool) -> Self {
        self.assistant = crate::indicator::assistant::AssistantLane::new(
            crate::indicator::assistant::ROOT,
            fingerprint,
            native_signal,
        );
        self
    }

    pub fn with_tesla(mut self, enabled: bool, counter_ns: u64) -> Self {
        self.tesla.enabled = enabled;
        self.tesla.counter = counter_ns;
        self
    }

    /// carrot `_update_driver_blinker` Tesla branch: the stalk, not the lamp.
    /// `lamp` is the masked lamp, `driver_lamps` the lamp compared with the
    /// lever (the raw lamp on a Kick car, carrot 6393f9518).
    fn tesla_driver(
        &mut self,
        car: &crate::types::Car,
        lamp: u8,
        driver_lamps: u8,
        valid: bool,
    ) -> u8 {
        let t = &mut self.tesla;
        let driver = car.turn_signal_driver_direction;
        let counts = car.stalk_counts;
        let last = t.driver_counts.replace(counts);
        let (left, right) = last.map_or((false, false), |l| (counts[0] != l[0], counts[1] != l[1]));
        let press = if left != right {
            if left {
                1
            } else {
                2
            }
        } else {
            0
        };
        if driver_lamps == 3 {
            t.manual_direction = 0;
            t.manual_grace = 0.;
            lamp
        } else if !valid || !car.turn_signal_driver_valid {
            t.manual_direction = 0;
            t.manual_grace = 0.;
            0
        } else if matches!(driver, 1 | 2) || press != 0 {
            t.manual_direction = if driver != 0 { driver } else { press };
            t.manual_grace = 1.;
            t.manual_direction
        } else if driver_lamps == t.manual_direction && driver_lamps != 0 {
            t.manual_grace = 0.5;
            t.manual_direction
        } else if lamp != 3 && t.manual_grace > DT {
            t.manual_grace -= DT;
            0
        } else {
            t.manual_direction = 0;
            t.manual_grace = 0.;
            0
        }
    }

    /// carrot `_update_auto_app_request`.
    fn tesla_app_request(
        &mut self,
        input: &Input,
        remote: Option<&str>,
        driver: u8,
        stalk_press: u8,
    ) {
        let (car, nav) = (&input.car, &input.navigation);
        let new_command =
            nav.command_index != self.carrot_cmd_index_last && nav.command == "LANECHANGE";
        if new_command {
            // Consume even rejected commands: recovery must never replay an old action.
            self.carrot_cmd_index_last = nav.command_index;
        }
        let requested = match remote {
            Some("laneLeft") => 1,
            Some("laneRight") => 2,
            _ if new_command && nav.argument == "LEFT" => 1,
            _ if new_command && nav.argument == "RIGHT" => 2,
            _ => 0,
        };
        let waiting = matches!(self.lane_change_state, State::Off | State::PreLaneChange);
        let now = input.indicator.boot;
        let permitted = input.lateral_active
            && car.can_valid
            && !car.trailer_connected
            && !car.vehicle_sensors_invalid
            && car.drive
            && car.turn_signal_driver_valid
            && driver == 0
            && stalk_press == 0
            && self.tesla.manual_direction == 0
            && !(car.left_blinker && car.right_blinker)
            && (!waiting || car.v_ego >= MIN_SPEED)
            && self.desire_disable_count == 0;
        if self.tesla.app_direction != 0 && waiting {
            self.tesla.app_wait += DT;
        }
        let expired = waiting
            && self.tesla.app_direction != 0
            && (self.tesla.app_wait >= 10. || now >= self.tesla.app_deadline);
        if !permitted || expired {
            if self.tesla.app_direction != 0 && driver == 0 {
                self.lane_change_state = State::Off;
                self.lane_change_direction = 0;
            }
            self.tesla.app_direction = 0;
        }
        if requested != 0
            && permitted
            && self.tesla.app_direction == 0
            && self.lane_change_state == State::Off
            && self.tesla.signal_request_id == 0
        {
            self.tesla.app_direction = requested;
            self.tesla.app_wait = 0.;
            self.tesla.app_deadline = now + 10.;
        }
    }

    /// carrot `_update_auto_signal`.
    fn tesla_auto_signal(
        &mut self,
        input: &Input,
        atc_enabled: bool,
        atc: u8,
        driver: u8,
        previous: State,
    ) {
        let car = &input.car;
        let nav = &input.navigation;
        if self.atc_type != self.tesla.atc_type {
            self.tesla.lane_completed = false;
        }
        self.tesla.atc_type.clone_from(&self.atc_type);
        if previous == State::Finishing && self.lane_change_state != previous {
            self.tesla.lane_completed = true;
        }
        if self.lane_change_state == State::Starting && previous != self.lane_change_state {
            self.tesla.lane_completed = false;
        }
        if (self.carrot_lane_change_count > 0 || self.tesla.app_direction != 0)
            && previous == State::Off
            && self.lane_change_state == State::PreLaneChange
        {
            self.tesla.lane_completed = false;
        }
        let common = !car.cruise_cancel_state
            && car.can_valid
            && !car.trailer_connected
            && !car.vehicle_sensors_invalid
            && car.drive
            && car.turn_signal_driver_valid
            && driver == 0
            && self.tesla.manual_direction == 0
            && !(car.left_blinker && car.right_blinker)
            && self.desire_disable_count == 0;
        let stopped_hold = self.tesla.journey.direction != 0
            && input.indicator.turn_hold_allowed
            && car.standstill
            && car.v_ego.is_finite()
            && car.v_ego.abs() < 0.1
            && self.tesla.signal_request_id != 0
            && car.auto_turn_signal_ready_request_id == self.tesla.signal_request_id;
        let turn_desire = if atc_enabled {
            match self.desire {
                1 => 1,
                2 => 2,
                _ => 0,
            }
        } else {
            0
        };
        let mut direction = self
            .tesla
            .journey
            .update(crate::indicator::tesla_turn::JourneyInput {
                now: input.indicator.now,
                car,
                pose: input.indicator.turn_pose,
                nav: &self.atc_type,
                distance: Some(nav.x_dist_to_turn),
                desire: turn_desire,
                permitted: common && (input.lateral_active || stopped_hold),
                lateral_active: input.lateral_active,
            });
        let permitted = common && input.lateral_active;
        if !permitted || self.lane_change_state == State::Off || self.tesla.lane_completed {
            if !permitted && self.tesla.lane_direction != 0 {
                self.tesla.lane_completed = true;
            }
            self.tesla.lane_direction = 0;
        } else if atc_enabled
            && self.tesla.lane_direction == 0
            && (self.lane_change_state == State::PreLaneChange || previous == State::PreLaneChange)
        {
            // The FSM owns the maneuver, including a bounded App wait.
            self.tesla.lane_direction = if atc == 1 { 1 } else { 2 };
        }
        if permitted && direction == 0 && !self.tesla.journey.consumed {
            if self.tesla.lane_direction != 0 {
                direction = self.tesla.lane_direction;
            } else if !self.tesla.lane_completed
                && self.atc_active != 2
                && !self.blinker_ignore
                && matches!(self.atc_type.as_str(), "atc left" | "atc right")
                && matches!(nav.x_turn_info, 1 | 2)
                && 0. < nav.x_dist_to_turn
                && nav.x_dist_to_turn <= (car.v_ego * 8. + 20.).max(40.).min(250.)
            {
                direction = if nav.x_turn_info == 1 { 1 } else { 2 };
            }
        }
        if direction != 0 && direction != self.tesla.signal_direction {
            self.tesla.counter += 1;
        }
        self.tesla.signal_direction = direction;
        self.tesla.signal_request_id = if direction != 0 {
            self.tesla.counter
        } else {
            0
        };
    }

    fn update_indicators(&mut self, input: &Input) {
        let indicator = &input.indicator;
        if self.indicator_supported && (self.frame == 1 || self.frame.is_multiple_of(20)) {
            if let Some(config) = indicator.config {
                self.indicator_config = config;
            }
            // Restores masks after a restart and acknowledges low-speed TBT ownership.
            if let Some((stamp, direction)) = indicator.journal {
                let now = indicator.now;
                if stamp.is_finite()
                    && self.indicator_journal_time < stamp
                    && stamp <= now
                    && now - stamp < 20.
                    && matches!(direction, 1..=3)
                {
                    self.boundary_indicator.own(now, direction as u8);
                    self.indicator_journal_time = stamp;
                }
            }
        }
        if self.indicator_supported {
            let now = indicator.now;
            // Reset parking evidence while CANCEL is latched; keep lamp
            // isolation so an OEM one-touch tail cannot become steering.
            let busy = input.car.cruise_cancel_state
                || matches!(self.lane_change_state, State::Starting | State::Finishing);
            let config = self.indicator_config;
            self.boundary_indicator.update(
                now,
                &input.car,
                &input.model,
                config.boundary,
                indicator.valid,
                busy,
            );
            if self
                .model_indicator
                .active
                .as_ref()
                .is_some_and(|a| a.source() == "manual")
                && !config.boundary
            {
                self.model_indicator.active = None;
            }
            let manual_edge = self.boundary_indicator.request_direction;
            self.model_indicator.update(
                ObserverInput {
                    now,
                    car: &input.car,
                    model: &input.model,
                    valid: indicator.valid,
                    lateral_active: input.lateral_active,
                    desire_enabled: config.model_desire,
                    path_enabled: config.model_path,
                    busy,
                    manual_edge,
                    command_curvature: indicator.curvature,
                    feedback: indicator.feedback.as_ref(),
                    manual_turn_enabled: config.manual_turn,
                    yaw_rate: indicator.yaw_rate,
                    manual_minimum: config.manual_minimum,
                },
                &mut self.boundary_indicator,
            );
        } else if self.boundary_indicator.blocked != 0 || !self.boundary_indicator.initialized {
            // Expire a voice-owned lamp mask without PE-only geometry observation.
            self.boundary_indicator.update(
                indicator.now,
                &input.car,
                &input.model,
                false,
                indicator.assistant_valid,
                false,
            );
        }
    }

    /// `indicatorPrepareDirection/Source`: display only, never actuation.
    pub fn indicator_preparation(&self) -> (u8, &'static str) {
        if self.model_indicator.active.is_some() {
            return (0, "");
        }
        let config = self.indicator_config;
        let mut preparation = self.model_indicator.candidate.filter(|(_, source)| {
            if *source == "model_path" {
                config.model_path
            } else {
                config.model_desire
            }
        });
        if config.boundary {
            if let Some(episode) = &self.boundary_indicator.episode {
                preparation = Some((episode.direction, "manual"));
            }
        }
        if config.manual_turn {
            if let Some(candidate) = self.model_indicator.manual_turn.candidate {
                preparation = Some(candidate);
            }
        }
        preparation.unwrap_or((0, ""))
    }
    /// Side of a lever press since the last update. The lamp cannot show a press while
    /// already flashing; the counters can, and never miss a press between updates.
    fn stalk_press(&mut self, car: &crate::types::Car, valid: bool) -> u8 {
        let counts = car.stalk_counts;
        let Some(last) = self.stalk_counts.replace(counts) else {
            return 0;
        };
        if self.tesla.enabled || (self.kick_supported && self.boundary_indicator.blocked != 0) {
            let (left, right) = (counts[0] != last[0], counts[1] != last[1]);
            if (car.left_blinker && car.right_blinker)
                || !valid
                || !car.turn_signal_driver_valid
                || left == right
            {
                return 0;
            }
            let direction = if left { 1 } else { 2 };
            let driver = car.turn_signal_driver_direction;
            return if driver == 0 || driver == direction {
                direction
            } else {
                0
            };
        }
        if counts[0] != last[0] {
            1
        } else if counts[1] != last[1] {
            2
        } else {
            0
        }
    }

    fn atc_blinker(
        &mut self,
        navigation: &Navigation,
        driver: u8,
        remote: Option<&str>,
    ) -> (u8, bool) {
        let kind = navigation.atc_type.as_str();
        let tesla = self.tesla.enabled;
        let app = self.tesla.app_direction;
        let mut blinker = 0;
        if tesla && app != 0 {
            blinker = app;
        } else if self.carrot_lane_change_count > 0 {
            blinker = self.carrot_blinker_state;
        } else if !tesla && matches!(remote, Some("laneLeft" | "laneRight")) {
            self.carrot_lane_change_count = 4;
            self.carrot_blinker_state = if remote == Some("laneLeft") { 1 } else { 2 };
            blinker = self.carrot_blinker_state;
        } else if !tesla
            && navigation.command_index != self.carrot_cmd_index_last
            && navigation.command == "LANECHANGE"
        {
            self.carrot_cmd_index_last = navigation.command_index;
            // carrot took every argument other than "LEFT" (empty, CANCEL, a
            // typo) as a right lane change. Only LEFT/RIGHT request one; any
            // other argument is consumed without a request.
            let direction = match navigation.argument.as_str() {
                "LEFT" => 1,
                "RIGHT" => 2,
                _ => 0,
            };
            if direction != 0 {
                self.carrot_lane_change_count = 4;
                self.carrot_blinker_state = direction;
                blinker = direction;
            }
        } else if matches!(kind, "turn left" | "turn right") {
            if self.atc_active != 2 {
                blinker = if kind == "turn left" { 1 } else { 2 };
                self.atc_active = 1;
                self.blinker_ignore = false;
            }
        } else if matches!(kind, "fork left" | "fork right" | "atc left" | "atc right") {
            if self.atc_active != 2 {
                blinker = if matches!(kind, "fork left" | "atc left") {
                    1
                } else {
                    2
                };
                self.atc_active = 1;
            }
        } else {
            self.atc_active = 0;
        }
        if driver != 0 && blinker != 0 && driver != blinker {
            blinker = 0;
            self.atc_active = 2;
        }
        let mut enabled = matches!(blinker, 1 | 2);
        if tesla && app != 0 && !matches!(self.lane_change_state, State::Off | State::PreLaneChange)
        {
            // The FSM now owns execution: keep the side for torque cancel only.
            enabled = false;
        }
        if driver == 0 {
            self.blinker_ignore = false;
        }
        if self.blinker_ignore {
            blinker = 0;
            enabled = false;
        }
        if self.atc_type != kind && !(tesla && app != 0) {
            enabled = false;
        }
        self.atc_type.clone_from(&navigation.atc_type);
        (blinker, enabled)
    }

    fn sides(&mut self, input: &Input) {
        let model = &input.model;
        for (index, side) in [&mut self.left, &mut self.right].into_iter().enumerate() {
            let (outer, current) = if index == 0 { (0, 1) } else { (3, 2) };
            side.update_geometry(
                &model.lane_lines[outer],
                model.lane_line_probs[outer],
                &model.lane_lines[current],
                &model.road_edges[index],
                model.lane_line_probs[current],
            );
            let (line, blindspot) = if index == 0 {
                (input.car.left_lane_line, input.car.left_blindspot)
            } else {
                (input.car.right_lane_line, input.car.right_blindspot)
            };
            side.update_line(line);
            side.update_obstacles(
                input.car.v_ego,
                &input.leads[index],
                blindspot,
                self.config.bsd < 0,
                &input.objects[index],
            );
            let line_allowed = if self.config.line_check >= 1 {
                matches!(side.lane_line_info_mod, 0 | 5)
            } else {
                side.lane_line_info_raw.div_euclid(10) != 2
            };
            side.compute_available(line_allowed, self.config.bsd < 0);
            side.update_triggers();
        }
        self.lane_change_available_left = self.left.lane_change_available;
        self.lane_change_available_right = self.right.lane_change_available;
    }

    pub fn update(
        &mut self,
        input: &Input,
        mut config: impl FnMut() -> Config,
        mut command: impl FnMut(bool) -> Option<String>,
    ) -> Result<u8, InvalidModel> {
        input.validate()?;
        self.frame = self.frame.wrapping_add(1);
        if self.frame.is_multiple_of(100) {
            self.config = config();
        }
        self.update_indicators(input);
        self.carrot_lane_change_count = self.carrot_lane_change_count.saturating_sub(1);
        self.lane_change_delay = maximum(0.0, self.lane_change_delay - DT);
        let car = &input.car;
        let below_speed = car.v_ego < MIN_SPEED;
        let trailer = car.trailer_connected;
        self.sides(input);
        if trailer {
            self.left.lane_change_available = false;
            self.right.lane_change_available = false;
            self.lane_change_available_left = false;
            self.lane_change_available_right = false;
            self.auto_lane_change_enable = false;
            self.next_lane_change = false;
            self.desire_log = "TRAILER:MANEUVER_BLOCKED".to_owned();
        }
        self.turn_desire_state = input.model.desire_state[1] + input.model.desire_state[2] > 0.1;
        if self.maneuver_type == Maneuver::Turn
            && car.steering_angle_deg.abs() > 80.0
            && input.model.orientation_rate_z[15].abs() < input.model.orientation_rate_z[5].abs()
        {
            self.turn_disable_count = 200;
        } else {
            self.turn_disable_count = self.turn_disable_count.saturating_sub(1);
        }
        let raw_lamps = u8::from(car.left_blinker) + 2 * u8::from(car.right_blinker);
        let lamp = self.boundary_indicator.filter_lamp(raw_lamps);
        // carrot 6393f9518: on a Kick car the verified lever, not the masked
        // lamp, says what the driver asked for while the lamp is owned.
        let driver_input_valid = !self.kick_supported || (input.indicator.valid && car.can_valid);
        let kick_masked = self.kick_supported && self.boundary_indicator.blocked != 0;
        let mut driver = if self.tesla.enabled || kick_masked {
            let driver_lamps = if self.kick_supported { raw_lamps } else { lamp };
            self.tesla_driver(car, lamp, driver_lamps, driver_input_valid)
        } else {
            if self.kick_supported {
                self.tesla.manual_direction = 0;
                self.tesla.manual_grace = 0.;
                self.tesla.driver_counts = Some(car.stalk_counts);
            }
            lamp
        };
        let mut driver_changed = driver != self.driver_blinker_state;
        self.driver_blinker_state = driver;
        let mut driver_enabled = matches!(driver, 1 | 2) && self.config.need_torque >= 0;
        let previous_state = self.lane_change_state;
        let physical_stalk_press = self.stalk_press(car, driver_input_valid);
        let stalk_press = if self.kick_supported && car.turn_signal_driver_valid {
            physical_stalk_press
        } else {
            self.boundary_indicator.filter_lamp(physical_stalk_press)
        };
        let lever = car.blinker_lever;
        let forced_turn = self.lever_forced_turn.update(lever, car.blinker_lamp);
        if self.lever_wait.update(
            self.config.lever_wait,
            stalk_press,
            lever,
            driver,
            self.lane_change_state,
            self.config.delay_tenths * 0.1,
        ) {
            // One-touch request timed out: ignore the lamp until it goes dark.
            driver_enabled = false;
            driver_changed = false;
        }
        let mut lever_decided_turn = false;
        let mut remote = command(input.lateral_active && car.can_valid && !below_speed && !trailer);
        // Separate authenticated voice intent: never synthesize remote events,
        // steering torque or a lamp acknowledgement; all FSM gates remain.
        let now = input.indicator.now;
        let assistant_direction = {
            let voice_side = self.assistant.active.as_ref().map(|a| a.direction.clone());
            let opposite_torque = car.steering_pressed
                && match voice_side.as_deref() {
                    Some("left") => car.steering_torque < 0.,
                    Some("right") => car.steering_torque > 0.,
                    _ => false,
                };
            let owned = !self.assistant.signal_id().is_empty();
            let raw_lamps = u8::from(car.left_blinker) | (u8::from(car.right_blinker) << 1);
            let expected = if owned {
                if voice_side.as_deref() == Some("left") {
                    1
                } else {
                    2
                }
            } else {
                0
            };
            if owned
                && raw_lamps != 0
                && raw_lamps != expected
                && !(lever != 0 || physical_stalk_press != 0)
            {
                // Unexplained opposite/hazard feedback must not re-enter as a driver request.
                self.boundary_indicator.own(now, raw_lamps);
                driver = 0;
                driver_changed = false;
                driver_enabled = false;
            }
            let lever_input = lever != 0 || physical_stalk_press != 0;
            let signal_driver_busy = !self.tesla.enabled
                && ((raw_lamps != 0 && raw_lamps != expected)
                    || (lever_input && !self.assistant.own_tap(now, car)));
            let nav = &input.navigation;
            let side_state =
                |side: &SideState, blindspot: bool| crate::indicator::assistant::SideState {
                    available: side.lane_change_available && side.lane_change_available_geom,
                    blindspot: blindspot || side.bsd_hold_counter != 0 || side.side_object_detected,
                };
            let state = crate::indicator::assistant::State {
                valid: input.indicator.assistant_valid
                    && car.can_valid
                    && !car.vehicle_sensors_invalid,
                lateral_active: input.lateral_active,
                drive: car.drive,
                trailer,
                speed: car.v_ego,
                phase: match self.lane_change_state {
                    State::Off => "off",
                    State::PreLaneChange => "preLaneChange",
                    State::Starting => "laneChangeStarting",
                    State::Finishing => "laneChangeFinishing",
                },
                driver_busy: driver_enabled || stalk_press != 0 || opposite_torque,
                indicator_driver_busy: signal_driver_busy,
                driver_override: stalk_press != 0
                    || opposite_torque
                    || (owned && signal_driver_busy),
                other_request: remote.is_some()
                    || self.desire_disable_count != 0
                    || self.turn_desire_state
                    || self.turn_direction != 0
                    || !matches!(
                        nav.atc_type.as_str(),
                        "" | "none" | "none prepare" | "straight" | "straight prepare"
                    )
                    || (nav.command == "LANECHANGE"
                        && nav.command_index != self.carrot_cmd_index_last),
                left: side_state(&self.left, car.left_blindspot),
                right: side_state(&self.right, car.right_blindspot),
            };
            let (direction, cancel) = self.assistant.before(now, &state);
            if cancel {
                self.carrot_lane_change_count = 0;
                self.tesla.app_direction = 0;
                if self.lane_change_state == State::PreLaneChange {
                    self.lane_change_state = State::Off;
                    self.lane_change_direction = 0;
                }
            }
            if let Some(d) = &direction {
                remote = Some(if d == "left" { "laneLeft" } else { "laneRight" }.to_owned());
            }
            direction
        };
        if self.tesla.enabled {
            self.tesla_app_request(input, remote.as_deref(), driver, stalk_press);
        }
        let (atc, atc_enabled) = self.atc_blinker(&input.navigation, driver, remote.as_deref());
        let mut enabled = driver_enabled || atc_enabled;
        let blinker = if driver_enabled { driver } else { atc };
        let side = match blinker {
            1 => Some(&self.left),
            2 => Some(&self.right),
            _ => None,
        };
        let atc_requested = atc_enabled
            && self.tesla.app_direction == 0
            && matches!(
                self.atc_type.as_str(),
                "fork left" | "fork right" | "atc left" | "atc right"
            );
        let atc_manual = atc_enabled
            && !driver_enabled
            && matches!(self.atc_type.as_str(), "fork left" | "atc left");
        let atc_only = atc_requested && !driver_enabled;
        // Turn pocket: navigation asks for the lane toward a turn ("atc left/
        // right", never a fork or a driver's lever). A pocket behind centre-line
        // bollards only opens just before the junction, usually while the car
        // slows for it; carrot cancelled the request below MIN_SPEED and the
        // turn was missed (route 00000ccb, 단계사거리, cancelled at 29 km/h).
        // Such a request now waits below MIN_SPEED and may start there once
        // `pocket_start_allowed`; every other gate, including the left side's
        // steering nudge (atc_manual), stays as it is.
        let pocket = atc_only && matches!(self.atc_type.as_str(), "atc left" | "atc right");
        let pocket_blindspot = match blinker {
            1 => car.left_blindspot,
            2 => car.right_blindspot,
            _ => false,
        };
        let retry_blocked =
            atc_only && side.is_some_and(|side| !matches!(side.lane_line_info_mod, 0 | 5));
        let mut auto_trigger = false;
        if let Some(side) = side.filter(|_| enabled && !trailer) {
            auto_trigger = if self.carrot_lane_change_count > 0
                || (self.tesla.enabled && self.tesla.app_direction != 0)
            {
                side.lane_change_available
            } else {
                self.auto_lane_change_enable
                    && !atc_manual
                    && side.edge_available
                    && (side.lane_available_trigger || side.lane_appeared)
                    && !side.side_object_detected
                    && side.bsd_hold_counter == 0
            };
            self.desire_log = format!(
                "{}:ALC={}, ",
                side.name,
                if self.auto_lane_change_enable {
                    "True"
                } else {
                    "False"
                }
            );
        } else {
            self.auto_lane_change_enable = false;
            self.next_lane_change = false;
        }
        if !input.lateral_active
            || self.lane_change_timer > 10.0
            || trailer
            || self.desire_disable_count > 0
        {
            self.lane_change_state = State::Off;
            self.lane_change_direction = 0;
            self.turn_direction = 0;
            self.maneuver_type = Maneuver::None;
        } else {
            let mut new_type = match side.filter(|_| enabled) {
                Some(side) => classify(
                    input,
                    side,
                    self.turn_desire_state,
                    &self.atc_type,
                    self.maneuver_type,
                ),
                None => Maneuver::None,
            };
            if self.tesla.enabled && self.tesla.app_direction != 0 {
                new_type = Maneuver::LaneChange;
            }
            if self.assistant.active.as_ref().is_some_and(|a| !a.began)
                && new_type != Maneuver::LaneChange
            {
                self.assistant.publish(now, "rejected", "not_a_lane_change");
                self.carrot_lane_change_count = 0;
                self.tesla.app_direction = 0;
                enabled = false;
                new_type = Maneuver::None;
            }
            // BlinkerLatchedTurn: the driver's lever replaces the speed/lane-line guess.
            let (lever_type, lever_undecided) = if driver_enabled && side.is_some() {
                lever_maneuver(lever, car.v_ego, self.config.latched_turn, forced_turn)
            } else {
                (None, false)
            };
            match lever_type {
                Some(LeverChoice::Turn) => new_type = Maneuver::Turn,
                Some(LeverChoice::LaneChange) => new_type = Maneuver::LaneChange,
                None => {}
            }
            lever_decided_turn = lever_type == Some(LeverChoice::Turn);
            if self.maneuver_type == Maneuver::LaneChange
                && new_type == Maneuver::Turn
                && !matches!(
                    self.lane_change_state,
                    State::PreLaneChange | State::Starting
                )
            {
                self.maneuver_type = Maneuver::Turn;
                self.lane_change_state = State::Off;
            } else if matches!(self.lane_change_state, State::Off | State::PreLaneChange) {
                self.maneuver_type = new_type;
            }
            if enabled && self.maneuver_type == Maneuver::Turn && self.enable_turn_desires {
                self.lane_change_state = State::Off;
                if self.turn_disable_count > 0 {
                    self.turn_direction = 0;
                    self.lane_change_direction = 0;
                } else {
                    self.turn_direction = if blinker == 1 { 1 } else { 2 };
                    self.lane_change_direction = self.turn_direction;
                }
            } else {
                self.turn_direction = 0;
                match self.lane_change_state {
                    State::Off => {
                        if let Some(side) = side.filter(|_| {
                            enabled
                                && (!self.prev_desire_enabled
                                    || (driver_enabled
                                        && (driver_changed || stalk_press == blinker)))
                                && (!below_speed || pocket)
                        }) {
                            self.lane_change_state = State::PreLaneChange;
                            self.lane_change_ll_prob = 1.0;
                            self.lane_change_delay = self.config.delay_tenths * 0.1;
                            self.auto_lane_change_enable = last_lane(side);
                            self.next_lane_change = false;
                        }
                    }
                    State::PreLaneChange => {
                        if let Some(side) = side {
                            self.lane_change_direction = if blinker == 1 { 1 } else { 2 };
                            if self.next_lane_change && driver_enabled && stalk_press == blinker {
                                self.next_lane_change = false;
                            }
                            let torque = car.steering_pressed
                                && if blinker == 1 {
                                    car.steering_torque > 0.0
                                } else {
                                    car.steering_torque < 0.0
                                };
                            let bsd_active = side.bsd_hold_counter > 0 && self.config.bsd >= 0;
                            let clear = (side.lane_available || side.edge_available)
                                && !side.side_object_detected
                                && !bsd_active;
                            let line_release = ((atc_requested && driver_enabled)
                                || (atc_only && auto_trigger))
                                && clear;
                            if atc_only && last_lane(side) {
                                self.auto_lane_change_enable = true;
                            }
                            if !enabled || (below_speed && !pocket) {
                                self.lane_change_state = State::Off;
                                self.lane_change_direction = 0;
                            } else {
                                let solid = self.config.line_check >= 2
                                    && !side.lane_change_available_geom
                                    && (side.lane_available || side.edge_available);
                                let block_released = side.lane_change_available_released
                                    && (driver_enabled || self.auto_lane_change_enable)
                                    && !retry_blocked;
                                let gate = (side.lane_change_available_geom
                                    && self.lane_change_delay == 0.0)
                                    || side.lane_line_info_edge_detect
                                    || solid
                                    || block_released
                                    || line_release;
                                let start = if solid {
                                    line_release
                                        || (torque && !(bsd_active && self.config.bsd == 1))
                                } else if bsd_active {
                                    torque && self.config.bsd != 1
                                } else if self.config.need_torque > 0 || self.next_lane_change {
                                    torque
                                } else if driver_enabled {
                                    side.lane_change_available || line_release
                                } else {
                                    (torque
                                        || (!atc_manual
                                            && (auto_trigger
                                                || side.lane_line_info_edge_detect
                                                || block_released)))
                                        && (side.lane_change_available || line_release)
                                };
                                // Only automatic Tesla requests wait for their own
                                // echoed, physically confirmed lamp session.
                                let signal_ready = !self.tesla.enabled
                                    || driver_enabled
                                    || (self.tesla.signal_request_id != 0
                                        && car.auto_turn_signal_ready_request_id
                                            == self.tesla.signal_request_id);
                                let mut can_start = gate
                                    && start
                                    && !lever_undecided
                                    && signal_ready
                                    && (!below_speed
                                        || pocket_start_allowed(car.v_ego, side, pocket_blindspot));
                                if can_start
                                    && assistant_direction.is_some()
                                    && !self.tesla.enabled
                                    && self.assistant.signal_route() != "ordinary"
                                {
                                    // Reserve our lamp before publishing its intent: a
                                    // delayed blink must never become driver input.
                                    self.boundary_indicator.own(now, blinker);
                                    let d = assistant_direction.clone().unwrap_or_default();
                                    can_start = self.assistant.request_signal(now, &d, car);
                                } else if !self.assistant.signal_id().is_empty() {
                                    // Once armed, losing admission cancels the attempt.
                                    can_start = false;
                                    self.assistant.publish(
                                        now,
                                        "rejected",
                                        "lane_conditions_changed",
                                    );
                                    self.carrot_lane_change_count = 0;
                                    self.lane_change_state = State::Off;
                                    self.lane_change_direction = 0;
                                }
                                if can_start {
                                    self.lane_change_state = State::Starting;
                                }
                            }
                        } else {
                            self.lane_change_state = State::Off;
                            self.lane_change_direction = 0;
                        }
                    }
                    State::Starting => {
                        // A press before the centre crossed the line re-lights this
                        // change's lamp and is ignored, never a second request.
                        let since = self
                            .crossing
                            .update(&input.model.lane_lines, self.lane_change_direction == 1);
                        if stalk_press != 0 && since >= 0.0 {
                            self.queued_lane_change = stalk_press;
                        }
                        self.lane_change_ll_prob =
                            maximum(self.lane_change_ll_prob - 2.0 * DT, 0.0);
                        if input.lane_change_prob < 0.02 && self.lane_change_ll_prob < 0.01 {
                            self.lane_change_state = State::Finishing;
                            self.finishing_timer = 0.0;
                        }
                    }
                    State::Finishing => {
                        self.finishing_timer += DT;
                        if stalk_press != 0
                            && (self.crossing.since >= 0.0
                                || self.finishing_timer >= STALK_QUEUE_GUARD_S)
                        {
                            self.queued_lane_change = stalk_press;
                        }
                        self.lane_change_ll_prob = minimum(self.lane_change_ll_prob + DT, 1.0);
                        if self.lane_change_ll_prob > 0.99 {
                            self.lane_change_direction = 0;
                            if enabled {
                                self.lane_change_state = State::PreLaneChange;
                                self.next_lane_change =
                                    !(driver_enabled && self.queued_lane_change == blinker);
                                self.queued_wait = 0.0;
                            } else {
                                self.lane_change_state = State::Off;
                            }
                        }
                    }
                }
            }
        }
        if matches!(self.lane_change_state, State::Off | State::PreLaneChange) {
            self.lane_change_timer = 0.0;
        } else {
            self.lane_change_timer += DT;
        }
        self.left.commit_last();
        self.right.commit_last();
        self.prev_desire_enabled = enabled;
        let cancel = car.steering_pressed
            && ((car.steering_torque < 0.0 && blinker == 1)
                || (car.steering_torque > 0.0 && blinker == 2));
        if cancel && self.lane_change_state != State::Off {
            self.lane_change_direction = 0;
            self.lane_change_state = State::Off;
            self.blinker_ignore = true;
        }
        if matches!(self.lane_change_state, State::Off | State::PreLaneChange) {
            self.crossing = LaneCrossing::default();
        }
        if self.lane_change_state == State::PreLaneChange && self.queued_lane_change != 0 {
            self.queued_wait += DT;
        }
        if self.lane_change_state == State::Off
            || (self.lane_change_state == State::Starting && previous_state == State::PreLaneChange)
            || (self.lane_change_state == State::PreLaneChange
                && (self.queued_wait >= LEVER_REQUEST_WAIT_S || self.queued_lane_change != blinker))
        {
            self.queued_lane_change = 0;
        }
        let hold = self.lever_wait.update_hold(self.lane_change_state, blinker);
        self.blinker_hold_direction = if self.queued_lane_change != 0 {
            self.queued_lane_change
        } else {
            hold
        };
        self.lever_turn = lever_decided_turn && self.turn_direction != 0;
        self.desire = if self.turn_direction != 0 {
            self.lane_change_direction = self.turn_direction;
            self.turn_direction
        } else if matches!(self.lane_change_state, State::Starting | State::Finishing)
            && self.lane_change_direction != 0
        {
            self.lane_change_direction + 2
        } else {
            0
        };
        if self.tesla.enabled {
            self.tesla_auto_signal(input, atc_enabled, atc, driver, previous_state);
            if self.lane_change_state == State::Off || self.tesla.lane_completed {
                self.tesla.app_direction = 0;
            }
        }
        // Missing feedback is not completion; the caller reports unknown.
        let phase = match self.lane_change_state {
            State::Off => "off",
            State::PreLaneChange => "preLaneChange",
            State::Starting => "laneChangeStarting",
            State::Finishing => "laneChangeFinishing",
        };
        let direction = match self.lane_change_direction {
            1 => "left",
            2 => "right",
            _ => "none",
        };
        let crossed = self.crossing.since >= 0.;
        self.assistant.after(now, phase, direction, crossed);
        if matches!(self.lane_change_state, State::Off | State::Starting) {
            self.keep_pulse_timer = 0.0;
        } else if self.lane_change_state == State::PreLaneChange {
            self.keep_pulse_timer += DT;
            if self.keep_pulse_timer > 1.0 {
                self.keep_pulse_timer = 0.0;
            } else if matches!(self.desire, 5 | 6) {
                self.desire = 0;
            }
        }
        // Origin is latched before injected lamps can look like driver input.
        // preLaneChange is excluded: indicate only actual lane changes.
        let turning = matches!(self.desire, 1 | 2);
        let changing = matches!(self.lane_change_state, State::Starting | State::Finishing);
        let continuing = if self.auto_turn_signal_lane_change {
            changing
        } else {
            turning
        };
        if !continuing || self.lane_change_direction != self.auto_turn_signal {
            self.auto_turn_signal = 0;
        }
        if self.auto_turn_signal == 0 && atc_enabled && !driver_enabled && self.atc_active == 1 {
            if turning && matches!(self.atc_type.as_str(), "turn left" | "turn right") {
                self.auto_turn_signal = self.lane_change_direction;
                self.auto_turn_signal_lane_change = false;
            } else if self.lane_change_state == State::Starting && atc_only {
                self.auto_turn_signal = self.lane_change_direction;
                self.auto_turn_signal_lane_change = true;
            }
        }
        Ok(self.desire)
    }
}

/// Below `MIN_SPEED` a waiting turn-pocket request starts only at walking pace
/// or faster, into a lane the model sees (`lane_change_available`), with the
/// blind spot and side clear. The car's own blind-spot signal counts even when
/// `LaneChangeBsd` tells the side state to ignore it.
fn pocket_start_allowed(v_ego: f64, side: &SideState, blindspot: bool) -> bool {
    v_ego >= POCKET_MIN_SPEED
        && side.lane_change_available
        && side.bsd_hold_counter == 0
        && !side.side_object_detected
        && !blindspot
}

fn last_lane(side: &SideState) -> bool {
    side.lane_exist_count.counter <= 0 && !side.lane_change_available_geom
}

fn classify(
    input: &Input,
    side: &SideState,
    turn_desire: bool,
    atc: &str,
    old: Maneuver,
) -> Maneuver {
    let speed = input.car.v_ego * 3.6;
    let mut score = i32::from(speed < 30.0 || (speed < 40.0 && input.car.a_ego < -1.0));
    score += i32::from(speed < 40.0 && !side.lane_available && !side.edge_available);
    score += i32::from(speed < 40.0 && side.lane_exist_count.counter < 10);
    score += i32::from(turn_desire);
    if matches!(atc, "turn left" | "turn right") {
        score += 2;
    } else if matches!(atc, "fork left" | "fork right" | "atc left" | "atc right") {
        score -= 2;
    }
    if score >= 2 {
        // carrot required the road edge on that side to be over 4 m away at
        // every speed. At an intersection the model's edge sits on the centre
        // line (~2-4 m), so a blinker below the lane-change speed - where it can
        // only mean a turn - waited 1.5-3 s for the edge to recede while the
        // model kept following the lane (route 00000cb1: 7 of 9 turns, the
        // driver steering each). Below MIN_SPEED the edge no longer gates it.
        if side.dist_to_edge_far > 4.0 || speed < crate::types::MIN_SPEED * 3.6 {
            Maneuver::Turn
        } else {
            old
        }
    } else {
        Maneuver::LaneChange
    }
}

#[cfg(test)]
mod lane_command_tests {
    use super::DesireHelper;
    use crate::types::Navigation;

    #[test]
    fn only_left_or_right_lane_change_commands_request_a_side() {
        for (argument, expected) in [
            ("LEFT", 1),
            ("RIGHT", 2),
            ("", 0),
            ("CANCEL", 0),
            ("left", 0),
        ] {
            let mut helper = DesireHelper::new(false, None);
            let navigation = Navigation {
                command_index: 7,
                command: "LANECHANGE".into(),
                argument: argument.into(),
                ..Navigation::default()
            };
            let (blinker, _) = helper.atc_blinker(&navigation, 0, None);
            assert_eq!(blinker, expected, "{argument:?}");
            assert_eq!(helper.carrot_cmd_index_last, 7, "{argument:?} is consumed");
        }
    }
}

#[cfg(test)]
mod classify_tests {
    use super::{classify, Maneuver};
    use crate::{side::SideState, types::Input};

    #[test]
    fn a_blinker_below_lane_change_speed_turns_even_with_a_near_road_edge() {
        let mut side = SideState::new("left");
        side.dist_to_edge_far = 2.5; // centre line of a two-way road
        let mut input = Input::default();
        for (kph, expected) in [
            (15., Maneuver::Turn),
            (29., Maneuver::Turn),
            (35., Maneuver::None),
        ] {
            input.car.v_ego = kph / 3.6;
            // score: below 30 km/h (+1 only there) and no side lane seen (+1)
            let got = classify(&input, &side, true, "", Maneuver::None);
            assert_eq!(got, expected, "{kph}");
        }
        side.dist_to_edge_far = 5.;
        input.car.v_ego = 35. / 3.6;
        assert_eq!(
            classify(&input, &side, true, "", Maneuver::None),
            Maneuver::Turn
        );
    }
}

#[cfg(test)]
mod pocket_tests {
    use super::DesireHelper;
    use crate::types::{Config, Input, State};

    const N: usize = 33;

    /// Ego lane ±1.75 m. Before the pocket opens the left road edge (the
    /// centre-line bollards) is right next to the left line and no outer line
    /// is seen; once it opens a left lane 3.5 m wide appears.
    fn input(kph: f64, atc: &str, pocket_open: bool) -> Input {
        let mut input = Input {
            lateral_active: true,
            ..Input::default()
        };
        input.car.v_ego = kph / 3.6;
        input.car.drive = true;
        input.car.turn_signal_driver_valid = true;
        input.navigation.atc_type = atc.into();
        let line = |y: f64| vec![y; N];
        input.model.lane_lines = [line(-5.25), line(-1.75), line(1.75), line(5.25)];
        input.model.lane_line_probs = [if pocket_open { 0.9 } else { 0.0 }, 0.9, 0.9, 0.0];
        input.model.road_edges = [line(if pocket_open { -5.6 } else { -2.0 }), line(5.6)];
        input.model.orientation_rate_z = vec![0.; N];
        input
    }

    fn step(helper: &mut DesireHelper, input: &Input) {
        helper
            .update(input, Config::default, |_| None)
            .expect("valid model");
    }

    fn run(helper: &mut DesireHelper, input: &Input, frames: usize) {
        for _ in 0..frames {
            step(helper, input);
        }
    }

    /// The approach to 단계사거리 on route 00000ccb, then the pocket opening.
    fn waiting_below_lane_change_speed(atc: &str) -> DesireHelper {
        let mut helper = DesireHelper::new(false, None);
        run(&mut helper, &input(65., atc, false), 40);
        assert_eq!(
            helper.lane_change_state,
            State::PreLaneChange,
            "{atc} at 65 km/h"
        );
        run(&mut helper, &input(29., atc, false), 40);
        helper
    }

    #[test]
    fn a_turn_lane_request_keeps_waiting_below_lane_change_speed() {
        let helper = waiting_below_lane_change_speed("atc left");
        assert_eq!(helper.lane_change_state, State::PreLaneChange);
        let mut helper = waiting_below_lane_change_speed("atc left");
        run(&mut helper, &input(0., "atc left", false), 100);
        assert_eq!(
            helper.lane_change_state,
            State::PreLaneChange,
            "stopped at the light"
        );
    }

    #[test]
    fn the_left_pocket_starts_with_the_drivers_nudge_once_it_opens() {
        let mut helper = waiting_below_lane_change_speed("atc left");
        run(&mut helper, &input(20., "atc left", true), 20);
        // atc_manual: the left side never starts by itself.
        assert_eq!(helper.lane_change_state, State::PreLaneChange);
        let mut nudge = input(20., "atc left", true);
        nudge.car.steering_pressed = true;
        nudge.car.steering_torque = 1.;
        step(&mut helper, &nudge);
        assert_eq!(helper.lane_change_state, State::Starting);
        assert_eq!(helper.desire, 3, "laneChangeLeft");
    }

    #[test]
    fn a_nudge_does_not_start_a_closed_pocket_too_slow_or_with_the_blind_spot_lit() {
        for (kph, open, blindspot, why) in [
            (20., false, false, "pocket not open"),
            (8., true, false, "below walking pace"),
            (20., true, true, "blind spot"),
        ] {
            let mut helper = waiting_below_lane_change_speed("atc left");
            run(&mut helper, &input(kph, "atc left", open), 20);
            let mut nudge = input(kph, "atc left", open);
            nudge.car.left_blindspot = blindspot;
            nudge.car.steering_pressed = true;
            nudge.car.steering_torque = 1.;
            run(&mut helper, &nudge, 5);
            assert_ne!(helper.lane_change_state, State::Starting, "{why}");
        }
    }

    #[test]
    fn a_fork_lane_change_still_ends_below_lane_change_speed() {
        let mut helper = DesireHelper::new(false, None);
        run(&mut helper, &input(65., "fork left", false), 40);
        assert_eq!(helper.lane_change_state, State::PreLaneChange);
        run(&mut helper, &input(29., "fork left", false), 5);
        assert_eq!(helper.lane_change_state, State::Off);
    }
}
