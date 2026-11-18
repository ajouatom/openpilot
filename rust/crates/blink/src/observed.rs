//! Fresh lamp-only edges consumed once, with ownership journalled before
//! any CAN request. Port of carrot's `tools/jetlink/blink_observation.py`,
//! `blink_boundary.py` and `blink_feedback.py`.
use super::{policy::Command, policy::Hold, Side};
use openpilot_desire::{
    indicator::lane::{adjacent_lane, MANUAL_INDICATOR_MIN_SPEED},
    types::{Car, Model},
};
use serde_json::{json, Value};

/// The modelV2 fields the worker reads (`sm['modelV2'].meta`).
#[derive(Clone, Debug, Default)]
pub struct Meta {
    pub observed_indicator: u8,
    pub observed_mono_time: f64,
    pub observed_source: String,
    pub isolation_mask: u8,
    pub journal_mono_time: f64,
    pub manual_lane_indicator: u8,
    pub manual_lane_mono_time: f64,
    pub prepare_direction: u8,
    pub prepare_source: String,
    /// `"off" | "preLaneChange" | "laneChangeStarting" | "laneChangeFinishing"`.
    pub lane_change_state: &'static str,
    /// `"none" | "left" | "right"`.
    pub lane_change_direction: &'static str,
    pub desire: &'static str,
    pub lane_change_available_right: bool,
    pub lane_width_right: f64,
    pub assistant_signal_id: String,
    pub assistant_signal_route: String,
}

/// A journalled ownership record (`boundary-indicator.json`).
pub type Record = Value;

/// `blink_boundary.vehicle_allowed`.
pub fn vehicle_allowed(car: &Car, direction: u8) -> bool {
    matches!(direction, 1 | 2)
        && car.can_valid
        && car.drive
        && car.v_ego.is_finite()
        && car.v_ego >= MANUAL_INDICATOR_MIN_SPEED
        // The fresh model-owned edge already proved recent driver intent.
        && car.steering_torque.is_finite()
        && !car.trailer_connected
        && !car.steer_fault_temporary
        && !car.steer_fault_permanent
        && !car.left_blinker
        && !car.right_blinker
        && car.blinker_lever == 0
}

/// `blink_observation.allowed`.
#[allow(clippy::too_many_arguments)]
pub fn allowed(
    car: &Car,
    lat_active: bool,
    model: &Model,
    meta: &Meta,
    source: &str,
    now: f64,
    holding: bool,
    tap_echo: bool,
) -> bool {
    if !car.can_valid
        || !car.drive
        || car.trailer_connected
        || car.steer_fault_temporary
        || car.steer_fault_permanent
        || (car.blinker_lever != 0 && !(holding && tap_echo))
        || !car.v_ego.is_finite()
        // carrot 9eb26059a: a held lamp survives the filtered speed's
        // small negative swing at a D stop.
        || (car.v_ego < 0. && !(holding && car.standstill && car.v_ego.abs() < 0.1))
    {
        return false;
    }
    let manual = matches!(source, "manual" | "manual_turn");
    if !manual && !lat_active {
        return false;
    }
    if holding {
        return true;
    }
    let direction = meta.observed_indicator;
    let minimum = if manual {
        MANUAL_INDICATOR_MIN_SPEED
    } else if source == "model_turn" {
        10. / 3.6
    } else {
        30. / 3.6
    };
    direction != 0
        && !car.left_blinker
        && !car.right_blinker
        && car.v_ego >= minimum
        && matches!(meta.lane_change_state, "off" | "preLaneChange")
        // The manual boundary observer already admitted this exact fresh
        // crossing; a newer marking label must not drop its one-shot edge.
        && (matches!(source, "manual" | "model_turn" | "manual_turn")
            || adjacent_lane(car, model, direction, now))
}

fn journal_mask(journal: &Value) -> Option<i64> {
    journal
        .get("mask")
        .or_else(|| journal.get("direction"))
        .and_then(Value::as_i64)
}

#[derive(Debug, Default)]
pub struct BoundaryEvents {
    last: Option<f64>,
    pub journal: Value,
}

impl BoundaryEvents {
    pub fn new(journal: Value) -> Self {
        Self {
            last: None,
            journal: if journal.is_object() {
                journal
            } else {
                Value::Null
            },
        }
    }

    pub fn read(
        &mut self,
        now: f64,
        direction: u8,
        stamp: f64,
        allowed: bool,
        record: &mut dyn FnMut(&Record) -> std::io::Result<()>,
    ) -> Vec<Command> {
        let previous = self.last.replace(stamp);
        if previous.is_none_or(|p| p == stamp)
            || !matches!(direction, 1 | 2)
            || !allowed
            || !stamp.is_finite()
            || !(0. ..0.2).contains(&(now - stamp))
        {
            return Vec::new();
        }
        // Persist origin before sending; a failed write forbids output.
        let prior_time = self.journal.get("time").and_then(Value::as_f64);
        let mut mask = i64::from(direction);
        if let (Some(time), Some(prior)) = (prior_time, journal_mask(&self.journal)) {
            if time.is_finite() && (0. ..20.).contains(&(stamp - time)) && (1..=3).contains(&prior)
            {
                mask |= prior;
            }
        }
        let mut value = json!({"time": stamp, "direction": direction});
        if mask != i64::from(direction) {
            value["mask"] = json!(mask);
        }
        if record(&value).is_err() {
            return Vec::new();
        }
        self.journal = value;
        vec![Command {
            id: format!("boundary:{}", (stamp * 1e9) as i64),
            time: stamp,
            direction: if direction == 1 {
                Side::Left
            } else {
                Side::Right
            },
            device: None,
        }]
    }
}

#[derive(Debug, Default)]
pub struct ObservationSignals {
    pub events: BoundaryEvents,
    /// `(stamp, direction)` of the owned observation.
    pub owned: Option<(f64, u8)>,
    pub reason: String,
    saved_at: f64,
}

impl ObservationSignals {
    pub fn new(journal: Value) -> Self {
        Self {
            events: BoundaryEvents::new(journal),
            reason: "idle".into(),
            ..Self::default()
        }
    }

    pub fn update(
        &mut self,
        now: f64,
        meta: Option<&Meta>,
        allowed: bool,
        busy: bool,
        record: &mut dyn FnMut(&Record) -> std::io::Result<()>,
    ) -> Option<(String, Side)> {
        let direction = meta.map_or(0, |m| m.observed_indicator);
        let stamp = meta.map_or(0., |m| m.observed_mono_time);
        let source = meta.map_or("", |m| m.observed_source.as_str()).to_owned();
        let isolated = meta.map_or(0, |m| m.isolation_mask) & direction != 0;
        let edge = !self
            .events
            .read(now, direction, stamp, allowed && !busy && isolated, record)
            .is_empty();
        if self.owned.is_some()
            && (!allowed
                || busy
                || direction == 0
                || !isolated
                || self.owned != Some((stamp, direction))
                || !(0. ..30.).contains(&(now - stamp)))
        {
            self.owned = None;
        }
        if edge {
            self.owned = Some((stamp, direction));
            self.saved_at = now;
        }
        if self.owned.is_some() && now - self.saved_at >= 1. {
            let mask = self
                .events
                .journal
                .get("mask")
                .and_then(Value::as_i64)
                .unwrap_or(i64::from(direction))
                | i64::from(direction);
            let value = json!({"time": now, "direction": direction, "mask": mask});
            if record(&value).is_err() {
                self.owned = None;
            } else {
                self.events.journal = value;
                self.saved_at = now;
            }
        }
        self.reason = if self.owned.is_some() {
            source
        } else {
            "idle".into()
        };
        self.owned.map(|(_, d)| {
            (
                "observation".to_owned(),
                if d == 1 { Side::Left } else { Side::Right },
            )
        })
    }
}

#[derive(Debug, Default)]
pub struct LowSpeedOwnership {
    stamp: Option<f64>,
    journal: Value,
}

impl LowSpeedOwnership {
    pub fn ready(
        &mut self,
        now: f64,
        started: f64,
        meta: Option<&Meta>,
        record: &mut dyn FnMut(&Record) -> std::io::Result<()>,
    ) -> bool {
        if self.stamp != Some(started) {
            // Source-created low-speed TBT has no detector edge: persist once,
            // then wait for modeld to acknowledge lamp-feedback isolation.
            let mut mask = 2;
            if let Some(time) = self.journal.get("time").and_then(Value::as_f64) {
                if (0. ..20.).contains(&(now - time)) {
                    mask |= journal_mask(&self.journal).unwrap_or(0) & 3;
                }
            }
            let value = json!({"time": started, "direction": 2, "mask": mask});
            if record(&value).is_err() {
                return false;
            }
            self.journal = value;
            self.stamp = Some(started);
        }
        let ack = meta.map_or(0., |m| m.journal_mono_time);
        ack.is_finite() && ack >= started - 1e-8 && meta.is_some_and(|m| m.isolation_mask & 2 != 0)
    }
}

/// `blink_feedback.ObservationFeedback`: comma-side transmission context and
/// bounded early lamp evidence.
#[derive(Debug, Default)]
pub struct ObservationFeedback {
    ident: Option<String>,
    tap: Option<Value>,
    pub value: Value,
    seen: Option<f64>,
}

fn healthy(now: f64, peer: &super::source::Feedback) -> bool {
    (0. ..0.35).contains(&(now - peer.stamp))
        && matches!(
            peer.state,
            "ready" | "sending" | "waiting" | "waiting_previous_signal"
        )
}

impl ObservationFeedback {
    pub fn prepare(&mut self, now: f64, hold: Option<&Hold>, meta: Option<&Meta>, car: &Car) {
        let hold = hold.filter(|h| h.source == "observation");
        self.prepare_tap(now, hold, meta.map_or(0., |m| m.observed_mono_time), car);
    }

    pub fn prepare_tap(&mut self, now: f64, hold: Option<&Hold>, request: f64, car: &Car) {
        let ident = hold.map(|h| h.id.clone());
        if ident != self.ident {
            self.ident = ident;
            self.tap = None;
            self.value = Value::Null;
            self.seen = None;
            if let Some(h) = hold {
                if car.blinker_lever == 0 && !car.left_blinker && !car.right_blinker {
                    let direction = if h.direction == Side::Left { 1 } else { 2 };
                    self.tap = Some(
                        json!({"id": h.id, "direction": direction, "request": request,
                        "start": now, "counts": car.stalk_counts}),
                    );
                }
            }
        }
    }

    /// Published only after successful local submission, never extending
    /// the initial 0.6 s window.
    pub fn sent(&mut self, now: f64, peer: &super::source::Feedback, ok: bool) -> Value {
        let within = self.tap.as_ref().is_some_and(|t| {
            t["start"]
                .as_f64()
                .is_some_and(|start| (0. ..0.6).contains(&(now - start)))
        });
        self.value = if ok && healthy(now, peer) && within {
            json!({"stamp": now, "tap": self.tap})
        } else {
            Value::Null
        };
        self.value.get("tap").cloned().unwrap_or(Value::Null)
    }

    pub fn activation(
        &mut self,
        now: f64,
        hold: Option<&Hold>,
        car: &Car,
        peer: &super::source::Feedback,
        acknowledged: bool,
        activated: Option<String>,
        ok: bool,
    ) -> Option<String> {
        let Some(hold) = hold.filter(|h| h.source == "observation") else {
            self.seen = None;
            return activated;
        };
        let (lamp, opposite) = match hold.direction {
            Side::Left => (car.left_blinker, car.right_blinker),
            Side::Right => (car.right_blinker, car.left_blinker),
        };
        if !(ok && healthy(now, peer)) || opposite || self.ident.as_deref() != Some(&hold.id) {
            self.seen = None;
            return None;
        }
        if lamp {
            self.seen = Some(now);
        }
        // Lamp feedback may precede the receiver's ACK; never enable the
        // physical hold before that exact-ID ACK.
        if acknowledged
            && (activated.as_deref() == Some(&hold.id)
                || self.seen.is_some_and(|s| (0. ..0.75).contains(&(now - s))))
        {
            return Some(hold.id.clone());
        }
        None
    }
}
