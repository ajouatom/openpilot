//! Lamp-only observations of model motion; no lane-change desire is
//! produced. Port of carrot's `desire_lib/indicator_observer.py`.
use super::{
    boundary::{BoundaryIndicator, QUIET_SEC},
    echo::TapEcho,
    lane::{
        adjacent_lane, anticipated_lane, boundary_approach, finite, following_road_curve,
        path_direction, Sample,
    },
    manual_turn::{ManualTurnIndicator, ManualTurnMovement},
};
use crate::types::{Car, Model};
use std::collections::VecDeque;

pub const CONFIRM_SEC: f64 = 0.20;
pub const MANUAL_TURN_QUIET_SEC: f64 = 2.;

fn near(model: &Model) -> Option<(f64, f64)> {
    Some((*model.lane_lines[1].first()?, *model.lane_lines[2].first()?))
}

/// Bounded observation lifetime, including return-before-crossing cancel.
#[derive(Clone, Debug)]
pub struct Movement {
    pub direction: u8,
    pub source: &'static str,
    started: f64,
    initial: f64,
    minimum: f64,
    last: (f64, f64, f64),
    crossed: bool,
    settled: Option<f64>,
    peak_angle: f64,
    /// Bounded retreat before a lane reassignment (carrot 171f80bf7).
    return_since: Option<f64>,
    return_anchor: Option<(f64, f64)>,
    /// `(start, anchor far line, peak far line)` of a boundary reassignment.
    geometry_recovery: Option<(f64, f64, f64)>,
    pub reason: &'static str,
}

impl Movement {
    pub fn new(now: f64, direction: u8, source: &'static str, model: &Model) -> Option<Self> {
        let initial = model.lane_lines[usize::from(direction)].first()?.abs();
        let (left, right) = near(model)?;
        Some(Self {
            direction,
            source,
            started: now,
            initial,
            minimum: initial,
            last: (now, left, right),
            crossed: false,
            settled: None,
            peak_angle: 0.,
            return_since: None,
            return_anchor: None,
            geometry_recovery: None,
            reason: "active",
        })
    }

    pub fn update(&mut self, now: f64, car: &Car, model: &Model) -> bool {
        let Some((left, right)) = near(model) else {
            self.reason = "geometry";
            return false;
        };
        let angle = car.steering_angle_deg;
        let turn = self.source == "model_turn";
        let width = right - left;
        let width_ok = if self.geometry_recovery.is_some() {
            2. < width && width < 6.
        } else {
            2.5 < width && width < 4.8
        };
        if !finite(&[now, left, right, angle]) || (!turn && !width_ok) {
            self.reason = "geometry";
            return false;
        }
        let (stamp, old_left, old_right) = self.last;
        self.last = (now, left, right);
        if !(now - stamp > 0. && now - stamp <= 0.15)
            || now - self.started >= if turn { 30. } else { 12. }
        {
            self.reason = "timeout";
            return false;
        }
        let done = if turn {
            let signed = angle * if self.direction == 1 { 1. } else { -1. };
            self.peak_angle = self.peak_angle.max(signed);
            // carrot f89f40a37: a turn desire the model withdrew also ends it.
            let probability = model
                .desire_state
                .get(usize::from(self.direction))
                .copied()
                .unwrap_or(f64::NAN);
            let withdrawn = probability.is_finite() && probability < 0.3;
            (self.peak_angle >= 30. || withdrawn) && angle.abs() <= 10. && car.v_ego > 1. / 3.6
        } else {
            let left_side = self.direction == 1;
            let selected = if left_side { -left } else { right };
            let old_selected = if left_side { -old_left } else { old_right };
            let far = if left_side { right } else { -left };
            let old_far = if left_side { old_right } else { -old_left };
            let sign = if left_side { 1. } else { -1. };
            let driver_return =
                car.steering_pressed && car.steering_torque * sign < 0. && angle * sign < 0.;
            // One uncertain crossed boundary moving away while the opposite line
            // continues the approach is a widening/reassignment, not a measured
            // return (carrot 171f80bf7).
            if self.geometry_recovery.is_none()
                && self.source == "manual"
                && !self.crossed
                && now - self.started <= 0.35
                && car.steering_pressed
                && car.steering_torque * sign > 0.
                && self.minimum < 1.2
                && selected - self.minimum > 0.2
                && far - old_far > 0.1
                && model.lane_line_probs[usize::from(self.direction)] < 0.5
                && !driver_return
            {
                self.geometry_recovery = Some((now, old_far, far));
            }
            if let Some((started, anchor_far, peak_far)) = self.geometry_recovery {
                let peak_far = peak_far.max(far);
                self.geometry_recovery = Some((started, anchor_far, peak_far));
                if driver_return
                    || now - started >= 3.
                    || (now - started >= 0.25 && peak_far - anchor_far < 0.15)
                {
                    self.reason = if driver_return {
                        "returned"
                    } else {
                        "geometry"
                    };
                    return false;
                }
                let centered = 2.5 < right - left
                    && right - left < 4.8
                    && left < -1.
                    && right > 1.
                    && model.lane_line_probs[1] >= 0.5
                    && model.lane_line_probs[2] >= 0.5
                    && (left - old_left).abs().max((right - old_right).abs()) < 0.08;
                self.settled = if centered {
                    Some(self.settled.unwrap_or(now))
                } else {
                    None
                };
                if self.settled.is_some_and(|since| now - since >= 0.4) {
                    self.reason = "completed";
                    return false;
                }
                return true;
            }
            // A pair shifting one lane after reaching its edge is a completed
            // crossing; an arbitrary relabel far from the boundary is invalid.
            let (anchor_left, anchor_right) = self.return_anchor.unwrap_or((old_left, old_right));
            let anchor_selected = if left_side {
                -anchor_left
            } else {
                anchor_right
            };
            let jump = (left - anchor_left).abs().max((right - anchor_right).abs()) > 0.5;
            let relabel = jump
                && self.minimum < 0.9
                && selected - anchor_selected > 1.5
                && ((left - anchor_left) - (right - anchor_right)).abs() < 0.5;
            if jump && !relabel && self.return_anchor.is_none() {
                self.reason = "relabel";
                return false;
            }
            self.crossed |= selected <= 0. || relabel;
            self.minimum = self.minimum.min(selected);
            if !self.crossed && selected - self.minimum > 0.2 {
                // A model boundary can retreat briefly before both lines are
                // assigned to the next lane: keep the pre-retreat pair for that
                // bounded rebase (carrot 171f80bf7).
                if self.source != "manual"
                    || driver_return
                    || self.minimum >= 0.3
                    || (self.return_since.is_none() && now - self.started > 0.35)
                {
                    self.reason = "returned";
                    return false;
                }
                if self.return_since.is_none() {
                    self.return_since = Some(now);
                    self.return_anchor = Some((old_left, old_right));
                }
                if self.return_since.is_some_and(|since| now - since >= 0.15) {
                    self.reason = "returned";
                    return false;
                }
                self.settled = None;
                return true;
            }
            self.return_since = None;
            self.return_anchor = None;
            self.crossed
                && left < 0.
                && 0. < right
                && (-left).min(right) > 1.0
                && (selected - old_selected).abs() < 0.08
        };
        self.settled = if done {
            Some(self.settled.unwrap_or(now))
        } else {
            None
        };
        if self.settled.is_some_and(|since| now - since >= 0.4) {
            self.reason = "completed";
            return false;
        }
        true
    }
}

/// A split can widen before the model assigns the new lane's boundaries.
#[derive(Clone, Debug)]
pub struct ForecastLaneMovement {
    base: Movement,
    far_start: f64,
    far_peak: f64,
    progress_at: f64,
    split: bool,
}

impl ForecastLaneMovement {
    fn new(now: f64, direction: u8, source: &'static str, model: &Model) -> Option<Self> {
        let base = Movement::new(now, direction, source, model)?;
        let far = model.lane_lines[usize::from(3 - direction)].first()?.abs();
        Some(Self {
            base,
            far_start: far,
            far_peak: far,
            progress_at: now,
            split: false,
        })
    }

    fn update(&mut self, now: f64, car: &Car, model: &Model) -> bool {
        let Some((left, right)) = near(model) else {
            self.base.reason = "geometry";
            return false;
        };
        let m = &mut self.base;
        let (near_v, far) = if m.direction == 1 {
            (-left, right)
        } else {
            (right, -left)
        };
        let (stamp, old_left, old_right) = m.last;
        let (old_near, old_far) = if m.direction == 1 {
            (-old_left, old_right)
        } else {
            (old_right, -old_left)
        };
        if !finite(&[now, left, right, car.v_ego])
            || !(now - stamp > 0. && now - stamp <= 0.15)
            || car.steering_pressed
            || !(left < 0. && 0. < right)
            || !(2.5 < right - left && right - left < 5.5)
            || car.v_ego < 30. / 3.6
        {
            m.reason = "cancelled";
            return false;
        }
        let widening = near_v > m.minimum + 0.20
            && far >= self.far_start + 0.20
            && near_v + far >= m.initial + self.far_start + 0.30;
        if !self.split && !widening {
            return m.update(now, car, model);
        }
        self.split = true;
        let index = usize::from(m.direction) + 2;
        let (pred, current) = (&model.desire_prediction, &model.desire_state);
        if pred.len() != 32 {
            m.reason = "inputs";
            return false;
        }
        let rows: [&[f64]; 2] = [&pred[0..8], &current[..]];
        let d = usize::from(m.direction);
        let max = |row: &[f64]| row.iter().copied().fold(f64::NEG_INFINITY, f64::max);
        if !rows
            .iter()
            .all(|row| row.iter().all(|p| p.is_finite() && (0. ..=1.).contains(p)))
            || rows.iter().any(|row| row[3 - d].max(row[5 - d]) >= 0.5)
            || !rows
                .iter()
                .any(|row| row[index] >= 0.55 && row[index] == max(row))
        {
            m.reason = "completed";
            return false;
        }
        // Both labels moving together into the new lane differs from the
        // target line alone moving away: require far-side evidence first.
        let relabel = self.far_peak >= self.far_start + 0.55
            && self.far_peak - far >= 0.60
            && near_v - m.minimum >= 0.55
            && near_v - old_near > 0.15
            && old_far - far > 0.15;
        if (near_v - old_near).abs().max((far - old_far).abs()) > 1.5
            || (!m.crossed && !relabel && self.far_peak - far > 0.45)
        {
            m.reason = "returned";
            return false;
        }
        m.crossed |= relabel;
        let settled = m.crossed && near_v.min(far) > 1. && (near_v - old_near).abs() < 0.08;
        m.settled = if settled {
            Some(m.settled.unwrap_or(now))
        } else {
            None
        };
        if m.settled.is_some_and(|since| now - since >= 0.4) {
            m.reason = "completed";
            return false;
        }
        if far >= self.far_peak + 0.05 {
            self.far_peak = far;
            self.progress_at = now;
        }
        if now - m.started >= 12. || (!m.crossed && now - self.progress_at > 0.75) {
            m.reason = "timeout";
            return false;
        }
        m.last = (now, left, right);
        true
    }
}

#[derive(Clone, Debug)]
pub enum Active {
    Lane(Movement),
    Forecast(ForecastLaneMovement),
    ManualTurn(ManualTurnMovement),
}

impl Active {
    pub fn direction(&self) -> u8 {
        match self {
            Self::Lane(m) => m.direction,
            Self::Forecast(m) => m.base.direction,
            Self::ManualTurn(m) => m.direction,
        }
    }
    pub fn source(&self) -> &'static str {
        match self {
            Self::Lane(m) => m.source,
            Self::Forecast(m) => m.base.source,
            Self::ManualTurn(_) => "manual_turn",
        }
    }
    fn reason(&self) -> &'static str {
        match self {
            Self::Lane(m) => m.reason,
            Self::Forecast(m) => m.base.reason,
            Self::ManualTurn(m) => m.reason,
        }
    }
    fn update(&mut self, now: f64, car: &Car, model: &Model, yaw_rate: Option<f64>) -> bool {
        match self {
            Self::Lane(m) => m.update(now, car, model),
            Self::Forecast(m) => m.update(now, car, model),
            Self::ManualTurn(m) => m.update(now, car, yaw_rate),
        }
    }
}

pub struct ObserverInput<'a> {
    pub now: f64,
    pub car: &'a Car,
    pub model: &'a Model,
    pub valid: bool,
    pub lateral_active: bool,
    pub desire_enabled: bool,
    pub path_enabled: bool,
    pub busy: bool,
    pub manual_edge: u8,
    pub command_curvature: Option<f64>,
    pub feedback: Option<&'a serde_json::Value>,
    pub manual_turn_enabled: bool,
    pub yaw_rate: Option<f64>,
    /// carrot a31a59ca2 `manual_minimum`: native Kick has no OEM one-touch
    /// tail, so a manual request's normal completion waits this long (s).
    pub manual_minimum: f64,
}

type Candidate = Option<(u8, &'static str)>;

/// The first Kick frame time the policy worker reports for this very request
/// (`physical-hold.json` `native_start`), when fresh and consistent.
fn native_start(
    feedback: Option<&serde_json::Value>,
    request_time: f64,
    direction: u8,
    now: f64,
) -> Option<f64> {
    let feedback = feedback?;
    let native = feedback.get("native_start")?;
    let request = native.get("request")?.as_f64()?;
    let time = native.get("time")?.as_f64()?;
    let stamp = feedback
        .get("stamp")
        .and_then(serde_json::Value::as_f64)
        .unwrap_or(-1.);
    let side = if direction == 1 { "left" } else { "right" };
    ((request - request_time).abs() < 1e-6
        && native.get("direction")?.as_str()? == side
        && time.is_finite()
        && request_time <= time
        && time <= now
        && (0. ..0.15).contains(&(now - stamp)))
    .then_some(time)
}

#[derive(Clone, Debug)]
pub struct ModelIndicator {
    pub active: Option<Active>,
    pub request_time: f64,
    pub candidate: Candidate,
    approach: VecDeque<Sample>,
    evidence: VecDeque<(f64, Candidate)>,
    previous: Option<(f64, u32, f64, f64)>,
    pub reason: &'static str,
    manual_seen: f64,
    tap_echo: TapEcho,
    pub manual_turn: ManualTurnIndicator,
    /// `(ended, required side)` after a recognized driver turn ended on an
    /// opposite steer, so a consecutive turn can be followed (carrot 171f80bf7).
    follow_turn: Option<(f64, u8)>,
    follow_observer: ManualTurnIndicator,
}

impl Default for ModelIndicator {
    fn default() -> Self {
        Self {
            active: None,
            request_time: 0.,
            candidate: None,
            approach: VecDeque::with_capacity(10),
            evidence: VecDeque::with_capacity(12),
            previous: None,
            reason: "idle",
            manual_seen: 0.,
            tap_echo: TapEcho::default(),
            manual_turn: ManualTurnIndicator::default(),
            follow_turn: None,
            follow_observer: ManualTurnIndicator::default(),
        }
    }
}

fn push<T>(queue: &mut VecDeque<T>, value: T, limit: usize) {
    if queue.len() == limit {
        queue.pop_front();
    }
    queue.push_back(value);
}

impl ModelIndicator {
    pub fn update(&mut self, i: ObserverInput<'_>, isolation: &mut BoundaryIndicator) {
        self.reason = "idle";
        if self.active.is_none()
            && i.manual_edge == 0
            && !(i.desire_enabled || i.path_enabled || i.manual_turn_enabled)
        {
            self.manual_turn.reset("disabled");
            self.follow_turn = None;
            self.follow_observer.reset("disabled");
            self.evidence.clear();
            self.previous = None;
            self.reason = "disabled";
            return;
        }
        if self.inner(&i, isolation).is_none() {
            self.active = None;
            self.candidate = None;
            self.previous = None;
            self.reason = "invalid";
            self.manual_turn.reset("invalid");
        }
    }

    fn inner(&mut self, i: &ObserverInput<'_>, isolation: &mut BoundaryIndicator) -> Option<()> {
        let (now, car, model) = (i.now, i.car, i.model);
        let echo = match &self.active {
            Some(a) => {
                self.tap_echo
                    .matches(now, car, a.direction(), self.request_time, i.feedback)
            }
            None => false,
        };
        let common = i.valid
            && car.can_valid
            && car.drive
            && !car.trailer_connected
            && !car.steer_fault_temporary
            && !car.steer_fault_permanent
            && (car.blinker_lever == 0 || echo)
            && car.v_ego.is_finite()
            && car.steering_torque.is_finite();
        let mut promoted = false;
        if let Some(manual) = self.active.as_ref().filter(|a| a.source() == "manual") {
            // carrot f89f40a37: a manual lane request that turns into a driver
            // turn becomes that turn, keeping the original request identity and
            // its acknowledged lamp lease (request_time is not renewed).
            let direction = manual.direction();
            let curve = following_road_curve(model, car.v_ego, i.yaw_rate);
            let allowed = i.manual_turn_enabled && common && !i.busy;
            if let Some(turn) = self.manual_turn.update(
                now,
                car,
                model.frame_id,
                i.yaw_rate,
                allowed,
                curve,
                true,
                false,
            ) {
                if turn.direction == direction {
                    self.active = Some(Active::ManualTurn(turn));
                    promoted = true;
                }
            }
        } else if self.active.is_some() {
            self.manual_turn.reset("busy");
        }
        if let Some(a) = &mut self.active {
            let source = a.source();
            let allowed = if source == "manual_turn" {
                i.manual_turn_enabled
            } else {
                source == "manual"
                    || (i.lateral_active
                        && if source == "model_path" {
                            i.path_enabled
                        } else {
                            i.desire_enabled
                        })
            };
            let opposite = if a.direction() == 1 {
                car.right_blinker
            } else {
                car.left_blinker
            };
            let moving = promoted || a.update(now, car, model, i.yaw_rate);
            // Delay normal completion only; every input, driver and geometry
            // cancellation still ends the request.
            let minimum = i.manual_minimum > 0.
                && matches!(source, "manual" | "manual_turn")
                && a.reason() == "completed"
                && {
                    let started = native_start(i.feedback, self.request_time, a.direction(), now)
                        .unwrap_or(self.request_time);
                    (0. ..i.manual_minimum).contains(&(now - started))
                };
            if !common || i.busy || !allowed || opposite || (!moving && !minimum) {
                self.reason = if common && allowed && !i.busy && !opposite {
                    a.reason()
                } else {
                    "cancelled"
                };
                // Only an independently recognized manual turn ended by an
                // opposite steer seeds its successor; a lane crossing or an
                // input/lever cancellation cannot (carrot 171f80bf7).
                self.follow_turn = (source == "manual_turn" && self.reason == "driver_opposite")
                    .then(|| (now, 3 - a.direction()));
                self.follow_observer.reset("previous_turn");
                self.active = None;
            } else {
                // Keep ownership throughout the hold, including a long turn.
                isolation.own(now, a.direction());
                self.reason = source;
            }
            self.candidate = None;
            self.evidence.clear();
            self.previous = None;
            return Some(());
        }
        if i.manual_edge != 0 && common && isolation.request_time != self.manual_seen {
            // The manual turn tracker keeps observing so the request can be
            // promoted to a driver turn (carrot f89f40a37).
            self.manual_seen = isolation.request_time;
            self.active = Some(Active::Lane(Movement::new(
                now,
                i.manual_edge,
                "manual",
                model,
            )?));
            self.request_time = now;
            self.reason = "manual";
            return Some(());
        }
        // A new driver turn may be observed before other sources rearm; lamp
        // masking and the uninterrupted dark interval still apply.
        let manual_turn_after = isolation.block_until - QUIET_SEC + MANUAL_TURN_QUIET_SEC;
        // Junction lane relabels must not erase the steering onset (carrot
        // 569f3191e). Manual turns have independent torque, angle-growth, yaw
        // and road-curve checks.
        let turn_ready = !car.left_blinker
            && !car.right_blinker
            && now >= manual_turn_after
            && isolation.dark_since.is_some_and(|since| now - since >= 1.);
        if let Some((ended, required_side)) = self.follow_turn {
            let direction = if !car.steering_pressed {
                0
            } else if car.steering_torque > 0. {
                1
            } else if car.steering_torque < 0. {
                2
            } else {
                0
            };
            if !common
                || i.busy
                || !i.manual_turn_enabled
                || car.v_ego < 10. / 3.6
                || !(0. ..=3.).contains(&(now - ended))
                || (required_side != 0 && direction != 0 && direction != required_side)
            {
                self.follow_turn = None;
                self.follow_observer.reset("admission");
            }
        }
        let allowed = i.manual_turn_enabled && common && !i.busy && turn_ready;
        let curve = i.manual_turn_enabled && following_road_curve(model, car.v_ego, i.yaw_rate);
        let turn = self.manual_turn.update(
            now,
            car,
            model.frame_id,
            i.yaw_rate,
            allowed,
            curve,
            true,
            false,
        );
        // Ordinary fresh-steering detection stays independent: expiry of the
        // retained intent must never erase a turn it would recognize.
        let follow = self.follow_observer.update(
            now,
            car,
            model.frame_id,
            i.yaw_rate,
            self.follow_turn.is_some(),
            curve,
            turn_ready,
            true,
        );
        if let Some(turn) = turn.or(follow) {
            self.follow_turn = None;
            isolation.own(now, turn.direction);
            self.active = Some(Active::ManualTurn(turn));
            self.request_time = now;
            self.reason = "manual_turn";
            self.candidate = None;
            self.previous = None;
            self.evidence.clear();
            return Some(());
        }
        if !common
            || !i.lateral_active
            || i.busy
            || car.left_blinker
            || car.right_blinker
            || car.steering_pressed
            || car.v_ego < 10. / 3.6
            || now < isolation.block_until
            || now < isolation.relabel_until
            || isolation.blocked != 0
        {
            self.candidate = None;
            self.previous = None;
            self.evidence.clear();
            self.reason = "admission";
            return Some(());
        }
        let (left, right) = near(model)?;
        let old = self.previous.replace((now, model.frame_id, left, right));
        let geometry_ok = old.is_some_and(|old| {
            finite(&[left, right])
                && left < 0.
                && 0. < right
                && 2.5 < right - left
                && right - left < 4.8
                && (0.02..=0.15).contains(&(now - old.0))
                && model.frame_id > old.1
                && (left - old.2).abs().max((right - old.3).abs()) <= 0.4
        });
        if !geometry_ok {
            self.candidate = None;
            self.reason = "geometry";
            self.approach.clear();
            self.evidence.clear();
            return Some(());
        }
        let mut candidate: Candidate = None;
        let mut forecast = false;
        if i.desire_enabled && car.v_ego >= 30. / 3.6 {
            push(
                &mut self.approach,
                (
                    now,
                    left,
                    right,
                    model.lane_line_probs[1],
                    model.lane_line_probs[2],
                ),
                10,
            );
        } else {
            self.approach.clear();
        }
        if i.desire_enabled {
            let probs = &model.desire_state;
            if probs
                .iter()
                .all(|p| p.is_finite() && (0. ..=1.).contains(p))
            {
                // max(range(1, 5)) keeps the first maximum.
                let mut index = 1;
                for k in 2..5 {
                    if probs[k] > probs[index] {
                        index = k;
                    }
                }
                let direction = if matches!(index, 1 | 3) { 1 } else { 2 };
                // Dominant categorical prediction, not merely a nonzero value.
                let opposite = if direction == 1 {
                    probs[2].max(probs[4])
                } else {
                    probs[1].max(probs[3])
                };
                // carrot f89f40a37: a turn desire needs the car turning that way.
                let sign = if direction == 1 { 1. } else { -1. };
                let turn_motion = i.yaw_rate.is_some_and(|yaw| {
                    yaw.is_finite()
                        && car.steering_angle_deg.is_finite()
                        && car.steering_angle_deg * sign >= 15.
                        && -yaw * sign >= 3f64.to_radians()
                });
                if probs[index] >= 0.5
                    && probs[index] >= probs[0]
                    && probs[index] - opposite >= 0.3
                    && (index >= 3 || turn_motion)
                    // A lane-change desire also needs a measured approach
                    // (carrot 569f3191e): no straight-road false indications.
                    && (index < 3
                        || (car.v_ego >= 30. / 3.6
                            && boundary_approach(
                                self.approach.make_contiguous(),
                                direction,
                                0.15,
                            )))
                {
                    candidate = Some((
                        direction,
                        if index < 3 {
                            "model_turn"
                        } else {
                            "model_desire"
                        },
                    ));
                }
                let max = probs.iter().copied().fold(f64::NEG_INFINITY, f64::max);
                if candidate.is_none() && probs[0] == max && car.v_ego >= 30. / 3.6 {
                    let samples: Vec<Sample> = self.approach.iter().copied().collect();
                    let direction = anticipated_lane(model, &samples);
                    if direction != 0 {
                        forecast = true;
                        candidate = Some((direction, "model_desire"));
                    }
                }
            }
        }
        if candidate.is_none() && i.path_enabled && car.v_ego >= 30. / 3.6 {
            let direction = i
                .command_curvature
                .map_or(0, |curvature| path_direction(model, curvature));
            if direction != 0 {
                candidate = Some((direction, "model_path"));
            }
        }
        if let Some((direction, source)) = candidate {
            if source != "model_turn" && !adjacent_lane(car, model, direction, now) {
                candidate = None;
            }
        }
        push(&mut self.evidence, (now, candidate), 12);
        let mut evidence: Vec<(f64, Candidate)> = self
            .evidence
            .iter()
            .filter(|(t, _)| now - t <= 0.4)
            .copied()
            .collect();
        let matches: Vec<f64> = evidence
            .iter()
            .filter(|(_, c)| c.is_some() && *c == candidate)
            .map(|(t, _)| *t)
            .collect();
        if let Some(&first) = matches.first() {
            evidence.retain(|(t, _)| *t >= first);
        }
        let opposite = evidence.iter().any(|(_, c)| match (c, candidate) {
            (Some(c), Some(k)) => c.0 != k.0,
            _ => false,
        });
        self.candidate = candidate;
        if let Some((direction, source)) = candidate {
            if !opposite
                && matches.len() >= 4
                && matches.len() * 2 >= evidence.len()
                && now - matches[0] >= CONFIRM_SEC
            {
                self.active = Some(if forecast {
                    Active::Forecast(ForecastLaneMovement::new(now, direction, source, model)?)
                } else {
                    Active::Lane(Movement::new(now, direction, source, model)?)
                });
                self.request_time = now;
                isolation.own(now, direction);
                self.reason = source;
            }
        }
        Some(())
    }
}
