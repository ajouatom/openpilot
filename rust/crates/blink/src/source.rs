//! carrot's `tools/jetlink/blink_source.py` main loop, run in-process at the
//! worker's 20 Hz instead of as a Python process: inside card for the SPAS
//! CAN path (`spas`) and inside the Bluetooth daemon for the B1 path (`ble`).
//!
//! Intents come from explicit Bluetooth remote taps, the navigation turn
//! signal, the TBT right-indicator add-on, lamp-only observations, manual
//! boundary edges and a confirmed voice lane change. TBT, boundary and
//! observation intents are off in SPAS mode, as in carrot.
use super::{
    display::{DisplayInput, Publisher},
    observed::{self, BoundaryEvents, LowSpeedOwnership, ObservationFeedback, ObservationSignals},
    policy::{AutomaticInput, AutomaticSignals, Command, Hold, Policy, PolicyInput},
    remote::{Remote, RUNTIME},
    snapshot,
    tbt::{right_front_clear, Nav, TbtBlinker, TbtInput},
    turn_journey::Travel,
    voice::VoiceLaneSignal,
    Side,
};
use openpilot_cereal::{
    car_capnp::car_state,
    log_capnp::{event, Desire, LaneChangeDirection},
};
use openpilot_desire::indicator::{echo::TapEcho, lane::adjacent_lane};
use openpilot_messaging::state::{State as Subscribers, Topic};
use openpilot_params::Params;
use serde_json::{json, Value};
use std::path::Path;

const HOLD_PATH: &str = "/dev/shm/carrot-bluetooth/physical-hold.json";
const ASSISTANT_LANE_ROOT: &str = "/dev/shm/carrot-assistant-lane";
const PE: &str = "HYUNDAI_IONIQ_5_PE";

/// What the worker hands the transport each cycle (`encode(...)`).
#[derive(Clone, Debug, Default)]
pub struct Record {
    pub stamp: f64,
    pub ok: bool,
    pub hold: Option<Hold>,
    /// Short taps not held (`events`), most recent last.
    pub events: Vec<Command>,
    pub turn_phase: &'static str,
    pub devices: Vec<String>,
    pub activated: bool,
    pub relights: u32,
    pub b1_hold: bool,
    pub turn_position: bool,
    pub tbt_phase: &'static str,
    pub observation_phase: String,
    pub indicator_origin: Option<String>,
}

/// What the transport reports back (`worker.send(...)`).
#[derive(Clone, Debug, Default)]
pub struct Feedback {
    pub stamp: f64,
    pub accepted_id: Option<String>,
    pub available: bool,
    /// `"driver_override" | "ready" | "waiting" | "sending" | ...`.
    pub state: &'static str,
    /// B1 boards' own lamp sample (1 left, 2 right), when reported.
    pub lamps: Option<u8>,
}

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum Mode {
    Spas,
    Ble,
}

pub struct Worker {
    mode: Mode,
    fingerprint: String,
    front_radar: bool,
    policy: Policy,
    automatic: AutomaticSignals,
    travel: Travel,
    remote: Remote,
    tbt: TbtBlinker,
    boundary: BoundaryEvents,
    observation: ObservationSignals,
    low_speed: LowSpeedOwnership,
    display: Publisher,
    voice: VoiceLaneSignal,
    voice_tap: ObservationFeedback,
    observation_feedback: ObservationFeedback,
    tap_echo: TapEcho,
    activated_id: Option<String>,
    release_until: f64,
    trial_until: f64,
    last_trial: Option<String>,
    next: f64,
    pub record: Option<Record>,
}

/// The SPAS worker card runs.
pub type SpasWorker = Worker;

fn fresh_topic(topic: &Topic, now: f64, limit: f64) -> bool {
    topic.valid && topic.alive && (0. ..limit).contains(&(now - topic.receive_time))
}

fn fresh(stamp: f64, now: f64, ttl: f64) -> bool {
    stamp.is_finite() && (0. ..ttl).contains(&(now - stamp))
}

fn gear_name(gear: car_state::GearShifter) -> &'static str {
    match gear {
        car_state::GearShifter::Park => "park",
        car_state::GearShifter::Neutral => "neutral",
        car_state::GearShifter::Drive => "drive",
        car_state::GearShifter::Reverse => "reverse",
        car_state::GearShifter::Unknown => "unknown",
        _ => "other",
    }
}

fn side(direction: LaneChangeDirection) -> Option<Side> {
    match direction {
        LaneChangeDirection::Left => Some(Side::Left),
        LaneChangeDirection::Right => Some(Side::Right),
        _ => None,
    }
}

fn desire_name(desire: Desire) -> &'static str {
    match desire {
        Desire::TurnLeft => "turnLeft",
        Desire::TurnRight => "turnRight",
        _ => "other",
    }
}

fn read_json(path: &Path) -> Value {
    std::fs::read(path)
        .ok()
        .and_then(|bytes| serde_json::from_slice(&bytes).ok())
        .unwrap_or(Value::Null)
}

/// `atomic_json`: write then rename.
fn atomic_json(path: &Path, value: &Value) -> std::io::Result<()> {
    if let Some(parent) = path.parent() {
        std::fs::create_dir_all(parent)?;
    }
    let temporary = path.with_extension("tmp");
    std::fs::write(&temporary, value.to_string())?;
    std::fs::rename(&temporary, path)
}

/// `record_ownership`: keep a fresh earlier mask across rapid requests.
/// Marks a lamp the system lights so the desire helper does not take it as a
/// driver request (`boundary-indicator.json`, read by drivingd).
pub fn record_ownership(now: f64, value: &Value) -> std::io::Result<()> {
    let path = Path::new(RUNTIME).join("boundary-indicator.json");
    let previous = read_json(&path);
    let mut value = value.clone();
    let stamp = previous.get("time").and_then(Value::as_f64);
    let mask = previous
        .get("mask")
        .or_else(|| previous.get("direction"))
        .and_then(Value::as_i64);
    if let (Some(stamp), Some(mask @ 1..=3)) = (stamp, mask) {
        if stamp.is_finite() && (0. ..20.).contains(&(now - stamp)) {
            let own = value
                .get("mask")
                .or_else(|| value.get("direction"))
                .and_then(Value::as_i64)
                .unwrap_or(0);
            value["mask"] = json!(own | mask);
        }
    }
    atomic_json(&path, &value)
}

impl Default for Worker {
    fn default() -> Self {
        Self::new(Mode::Spas, String::new(), false)
    }
}

impl Worker {
    pub fn new(mode: Mode, fingerprint: String, front_radar: bool) -> Self {
        let journal = read_json(&Path::new(RUNTIME).join("boundary-indicator.json"));
        Self {
            mode,
            fingerprint,
            front_radar,
            policy: Policy::new(),
            automatic: AutomaticSignals::new(),
            travel: Travel::default(),
            remote: Remote::default(),
            tbt: TbtBlinker::default(),
            boundary: BoundaryEvents::new(journal.clone()),
            observation: ObservationSignals::new(journal),
            low_speed: LowSpeedOwnership::default(),
            display: Publisher::default(),
            voice: VoiceLaneSignal::default(),
            voice_tap: ObservationFeedback::default(),
            observation_feedback: ObservationFeedback::default(),
            tap_echo: TapEcho::default(),
            activated_id: None,
            release_until: 0.,
            trial_until: 0.,
            last_trial: None,
            next: 0.,
            record: None,
        }
    }
}

fn event_of<'a>(topic: Option<&'a Topic>) -> Option<event::Reader<'a>> {
    topic.and_then(|t| t.event().ok())
}

impl Worker {
    /// Runs one worker cycle when 50 ms have passed; `record` keeps the last
    /// result otherwise, as the transport keeps the worker's latest message.
    /// `car_fresh` is carState freshness (card passes its own state: true).
    #[allow(clippy::too_many_lines)]
    pub fn update(
        &mut self,
        now: f64,
        cs: car_state::Reader<'_>,
        car_fresh: bool,
        subs: &Subscribers,
        settings: &Params,
        feedback: &Feedback,
    ) -> Result<(), crate::Error> {
        if now < self.next {
            return Ok(());
        }
        self.next = now + 0.05;
        let spas = self.mode == Mode::Spas;
        let ble = self.mode == Mode::Ble;
        let topic = |name: &str| subs.topic(name).ok();
        let control = event_of(topic("carControl")).and_then(|e| match e.which().ok()? {
            event::Which::CarControl(v) => v.ok(),
            _ => None,
        });
        let selfdrive = event_of(topic("selfdriveState")).and_then(|e| match e.which().ok()? {
            event::Which::SelfdriveState(v) => v.ok(),
            _ => None,
        });
        let lat_active = control.is_some_and(|cc| cc.get_lat_active());
        let pandas_ok = topic("pandaStates").is_some_and(|t| {
            fresh_topic(t, now, 0.75)
                && matches!(t.event().ok().and_then(|e| e.which().ok()),
                    Some(event::Which::PandaStates(Ok(list))) if !list.is_empty())
        });
        let gear = gear_name(
            cs.get_gear_shifter()
                .unwrap_or(car_state::GearShifter::Unknown),
        );
        let (car, tbt_car) = snapshot::car(cs);
        // vehicle_ready: manual indicators work while driving or engaged;
        // unknown or stale CAN never does.
        let mut ok = (spas || self.fingerprint == PE)
            && car_fresh
            && ["carControl", "selfdriveState"]
                .iter()
                .all(|name| topic(name).is_some_and(|t| fresh_topic(t, now, 0.35)))
            && pandas_ok
            && cs.get_can_valid()
            && cs.get_v_ego().is_finite()
            && gear != "unknown";
        let bluetooth = self.remote.load();
        let commands = self.remote.commands(&bluetooth, now);
        let remote = self.policy.hold.as_ref().is_some_and(|h| {
            h.source == "bluetooth"
                && Remote::holds(&bluetooth, h.device.as_deref(), h.direction, h.time, now)
        });
        let mode = crate::setting_int(settings, "AutoBlinkerControl");
        let mode = if (0..=2).contains(&mode) { mode } else { 0 };
        let flag = |key: &str| settings.get_bool(key).unwrap_or(false);
        let auto_paused = cs.get_cruise_cancel_state();
        let carrot_limit = if self.automatic.direction.is_some() {
            1.0
        } else {
            0.35
        };
        let model_topic = topic("modelV2").filter(|t| fresh_topic(t, now, 0.35));
        let carrot_topic = topic("carrotMan").filter(|t| fresh_topic(t, now, carrot_limit));
        let auto_fresh = model_topic.is_some() && carrot_topic.is_some();
        let travelled = self.travel.update(now, f64::from(cs.get_v_ego()), ok);
        let always_lateral = flag("AlwaysLateral");
        let lateral_allowed = gear == "drive"
            && (lat_active
                || (selfdrive.is_some_and(|ss| ss.get_active() || always_lateral)
                    && cs.get_lat_enabled()
                    && !cs.get_steer_fault_temporary()
                    && !cs.get_steer_fault_permanent()));
        // carrotNavi is not subscribed: navigation_cancelled() is false.
        let auto_valid = !auto_paused && auto_fresh && ok && lateral_allowed;
        let model_reader = event_of(model_topic).and_then(|e| match e.which().ok()? {
            event::Which::ModelV2(v) => v.ok(),
            _ => None,
        });
        let carrot = event_of(topic("carrotMan")).and_then(|e| match e.which().ok()? {
            event::Which::CarrotMan(v) => v.ok(),
            _ => None,
        });
        let snapshot = model_reader.map(snapshot::model).transpose()?;
        let heading = topic("livePose")
            .filter(|t| fresh_topic(t, now, 0.35))
            .and_then(|t| match t.event().ok()?.which().ok()? {
                event::Which::LivePose(Ok(pose)) => {
                    let o = pose.get_orientation_n_e_d().ok()?;
                    let z = f64::from(o.get_z());
                    (pose.get_inputs_o_k()
                        && pose.get_sensors_o_k()
                        && pose.get_posenet_o_k()
                        && o.get_valid()
                        && z.is_finite())
                    .then_some(z)
                }
                _ => None,
            });
        let mut auto = None;
        if let (true, Some(md), Some(cm), Some(ss)) = (auto_fresh, model_reader, carrot, selfdrive)
        {
            let meta = md.get_meta()?;
            let atc_type = cm.get_atc_type()?.to_str()?.to_owned();
            let lane = if meta.get_auto_turn_signal_lane_change() && lat_active {
                side(meta.get_auto_turn_signal()?)
            } else {
                None
            };
            auto = self.automatic.update(AutomaticInput {
                now,
                mode,
                valid: auto_valid,
                cue: ss.get_atc_turn_start_mono_time() as f64 / 1e9,
                direction: side(ss.get_atc_turn_direction()?),
                atc_type: &atc_type,
                desire: desire_name(meta.get_desire()?),
                lane,
                speed: f64::from(cs.get_v_ego()),
                angle: f64::from(cs.get_steering_angle_deg()),
                heading,
                distance: Some(f64::from(cm.get_x_dist_to_turn())),
                position: None,
                target: None,
                travel: travelled,
                use_travel: true,
                pressed: cs.get_steering_pressed(),
                torque: f64::from(cs.get_steering_torque()),
            });
        } else {
            self.automatic.update(AutomaticInput {
                now,
                mode,
                valid: false,
                cue: 0.,
                direction: None,
                atc_type: "",
                desire: "none",
                lane: None,
                speed: 0.,
                angle: 0.,
                heading: None,
                distance: None,
                position: None,
                target: None,
                travel: None,
                use_travel: true,
                pressed: false,
                torque: 0.,
            });
        }
        // The ATC add-on lights the real right indicator; its CAN feedback
        // goes through the unchanged driver-blinker lane-change path.
        let tbt_distance = match crate::setting_int(settings, "AutoTurnControlRightLaneChange") {
            1 => 250,
            d @ 0..=500 => d,
            _ => 0,
        };
        let tbt_enabled = !spas
            && mode == 2
            && matches!(crate::setting_int(settings, "AutoTurnControl"), 1 | 2)
            && tbt_distance > 0;
        let radar = event_of(topic("radarState").filter(|t| fresh_topic(t, now, 0.35))).and_then(
            |e| match e.which().ok()? {
                event::Which::RadarState(v) => v.ok(),
                _ => None,
            },
        );
        let front_clear = tbt_enabled
            && self.front_radar
            && radar.is_some_and(|r| {
                let errors = r.get_radar_errors().is_ok_and(|e| {
                    e.get_can_error()
                        || e.get_radar_fault()
                        || e.get_wrong_config()
                        || e.get_radar_unavailable_temporary()
                });
                let lead = |l: openpilot_cereal::log_capnp::radar_state::lead_data::Reader<'_>| {
                    openpilot_desire::types::Lead {
                        status: l.get_status(),
                        d_rel: l.get_d_rel().into(),
                        v_rel: Some(l.get_v_rel().into()),
                        v_lead: Some(l.get_v_lead().into()),
                        radar_track_id: l.get_radar_track_id().into(),
                    }
                };
                let mut leads: Vec<_> = r
                    .get_leads_right()
                    .map(|list| list.iter().map(lead).collect())
                    .unwrap_or_default();
                if let Ok(l) = r.get_lead_right() {
                    leads.push(lead(l));
                }
                right_front_clear(errors, &leads)
            });
        let peer_fresh = fresh(feedback.stamp, now, 0.35);
        let peer_override = peer_fresh && feedback.state == "driver_override";
        let nav = carrot.map(|cm| Nav {
            turn_info: cm.get_x_turn_info(),
            distance: f64::from(cm.get_x_dist_to_turn()),
            atc_type: cm
                .get_atc_type()
                .ok()
                .and_then(|t| t.to_str().ok())
                .unwrap_or_default()
                .to_owned(),
        });
        let carrot_short = carrot_topic.is_some_and(|t| fresh_topic(t, now, 0.35));
        let tbt_auto = self.tbt.update(TbtInput {
            now,
            enabled: tbt_enabled,
            valid: auto_valid
                && carrot_short
                && lat_active
                && crate::setting_int(settings, "LaneChangeNeedTorque") == 0,
            car: &car,
            tbt: tbt_car,
            model: snapshot
                .as_ref()
                .filter(|_| auto_fresh)
                .map(|(m, meta)| (m, meta)),
            nav: nav.as_ref().filter(|_| auto_fresh),
            manual: !commands.is_empty() || peer_override,
            busy: auto.is_some()
                || (self.policy.hold.is_some()
                    && !matches!(self.tbt.phase, "waiting" | "changing" | "finishing")),
            front_clear,
            distance_limit: f64::from(tbt_distance),
        });
        let meta = snapshot.as_ref().map(|(_, m)| m);
        if let Some((kind, direction)) = tbt_auto {
            let started = self.tbt.started;
            if kind != "tbt_display"
                || self
                    .low_speed
                    .ready(now, started, meta, &mut |v| record_ownership(now, v))
            {
                auto = Some((kind.to_owned(), direction));
            }
        }
        let model_fresh = model_topic.is_some();
        let observed_source = meta.map_or(String::new(), |m| m.observed_source.clone());
        let observation_enabled = match observed_source.as_str() {
            "manual" => flag("ManualLaneChangeIndicator"),
            "manual_turn" => flag("ManualTurnIndicator"),
            "model_path" => flag("ModelPathIndicator"),
            "model_desire" | "model_turn" => flag("ModelDesireIndicator"),
            _ => false,
        };
        let own_tap = self.observation.owned.is_some_and(|(stamp, direction)| {
            self.tap_echo.matches(
                now,
                &car,
                direction,
                stamp,
                Some(&self.observation_feedback.value),
            )
        });
        let holding = self.observation.owned.is_some();
        let observation_allowed = !auto_paused
            && !spas
            && matches!(mode, 1 | 2)
            && observation_enabled
            && ok
            && model_fresh
            && snapshot.as_ref().is_some_and(|(model, meta)| {
                observed::allowed(
                    &car,
                    lat_active,
                    model,
                    meta,
                    &observed_source,
                    now,
                    holding,
                    own_tap,
                )
            })
            && (holding || (peer_fresh && feedback.state == "ready"));
        let observation_busy = !commands.is_empty()
            || auto.is_some()
            || self
                .policy
                .hold
                .as_ref()
                .is_some_and(|h| h.source != "observation")
            || peer_override;
        let observe_auto =
            self.observation
                .update(now, meta, observation_allowed, observation_busy, &mut |v| {
                    record_ownership(now, v)
                });
        if auto.is_none() {
            auto = observe_auto;
        }
        // Only a worker with live receiver feedback advertises a voice path.
        let fingerprint = self.fingerprint.clone();
        self.voice.advertise(
            now,
            ok && mode == 2
                && peer_fresh
                && (!spas || feedback.available)
                && matches!(
                    feedback.state,
                    "ready" | "waiting" | "sending" | "waiting_previous_signal" | "driver_override"
                ),
            &mut |route, time| {
                let _ = atomic_json(
                    &Path::new(ASSISTANT_LANE_ROOT).join("indicator-device.json"),
                    &json!({"fingerprint": fingerprint, "route": route, "time": time}),
                );
            },
        );
        let voice_auto = self.voice.select(
            meta,
            &car,
            mode == 2 && auto_valid && lat_active,
            peer_fresh
                && feedback.lamps.is_none()
                && feedback.accepted_id.is_none()
                && (feedback.state == "ready" || (spas && feedback.state == "waiting")),
            self.policy.hold.as_ref(),
            auto.is_some()
                || !commands.is_empty()
                || !peer_fresh
                || feedback.state == "driver_override",
        );
        if voice_auto.is_some() {
            auto = voice_auto;
        }
        let local_observation = auto.as_ref().is_some_and(|a| a.0 == "observation")
            || (auto.is_none()
                && self
                    .policy
                    .hold
                    .as_ref()
                    .is_none_or(|h| h.source == "observation"));
        let policy_valid = if local_observation {
            !auto_paused && ok && model_fresh
        } else {
            auto_valid
        };
        let (mut taps, hold) = self.policy.update(PolicyInput {
            now,
            mode,
            ok,
            commands: &commands,
            speed: f64::from(cs.get_v_ego()),
            angle: f64::from(cs.get_steering_angle_deg()),
            gear,
            remote,
            automatic: auto,
            auto_valid: policy_valid,
        });
        if self.voice.bind(hold.as_ref()) {
            // Persist ownership before sending: a restart cannot reinterpret
            // the lamp as a lever press.
            let direction = if self.voice.direction == Some(Side::Left) {
                1
            } else {
                2
            };
            let _ = record_ownership(
                now,
                &json!({"time": now, "direction": direction, "voice_fingerprint": self.fingerprint}),
            );
        }
        let voice_hold = hold
            .as_ref()
            .filter(|h| {
                self.voice.direction.is_some() && Some(&h.id) == self.voice.hold_id.as_ref()
            })
            .cloned();
        self.voice_tap
            .prepare_tap(now, voice_hold.as_ref(), 0., &car);
        self.observation_feedback
            .prepare(now, hold.as_ref(), meta, &car);
        let native_hold = !ble || self.fingerprint == PE;
        let tbt_turn_hold = !spas
            && native_hold
            && hold.as_ref().is_some_and(|h| h.source == "turn")
            && self.automatic.journey.active.is_some();
        // A confirmed TBT lamp is kept by the comma-side lease; a receiver
        // one-touch ending must not erase the ongoing turn.
        let continuing_turn = tbt_turn_hold
            && hold.as_ref().is_some_and(|h| {
                self.activated_id.as_deref() == Some(&h.id)
                    && peer_fresh
                    && matches!(feedback.state, "ready" | "waiting" | "driver_override")
                    && feedback.accepted_id.as_ref().is_none_or(|a| *a == h.id)
            });
        let hold = self
            .policy
            .acknowledged(
                now,
                self.activated_id.as_deref(),
                peer_override && !tbt_turn_hold,
            )
            .cloned();
        let acknowledged = hold.as_ref().is_some_and(|h| {
            peer_fresh
                && feedback.accepted_id.as_deref() == Some(h.id.as_str())
                && matches!(
                    feedback.state,
                    "ready" | "waiting" | "waiting_previous_signal"
                )
        }) || continuing_turn;
        let (left, right) = (cs.get_left_blinker(), cs.get_right_blinker());
        if !acknowledged {
            if self.activated_id.is_some() {
                self.release_until = now + 0.8;
            }
            self.activated_id = None;
        } else if let Some(h) = &hold {
            if (h.direction == Side::Left && left && !right)
                || (h.direction == Side::Right && right && !left)
            {
                self.activated_id = Some(h.id.clone());
            }
        }
        self.activated_id = self.observation_feedback.activation(
            now,
            hold.as_ref(),
            &car,
            feedback,
            acknowledged,
            self.activated_id.take(),
            ok,
        );
        if feedback.state == "driver_override" && !continuing_turn {
            self.release_until = 0.;
        }
        if !taps.is_empty() || ((spas || !native_hold) && feedback.state == "driver_override") {
            self.automatic.pending = None;
            self.automatic.journey.pending = None;
            self.automatic.journey.active = None;
            self.automatic.journey.cancelled_until = now + 3.;
        }
        let mut hold = hold;
        if !spas
            && self.policy.relight(
                now,
                self.activated_id.as_deref(),
                acknowledged && !peer_override,
                (left, right),
                auto_valid && self.automatic.direction.is_some() && feedback.state == "ready",
            )
        {
            hold.clone_from(&self.policy.hold);
            self.activated_id = None;
        }
        let model_fresh = topic("modelV2").is_some_and(|t| fresh_topic(t, now, 0.35));
        let meta = snapshot.as_ref().filter(|_| model_fresh).map(|(_, m)| m);
        let boundary_direction = meta.map_or(0, |m| m.manual_lane_indicator);
        // carrot: `not hasattr(meta, 'observedIndicator')` is always false
        // with the current schema, so the boundary path never emits.
        let has_observed_field = meta.is_some();
        let boundary_allowed = !auto_paused
            && !has_observed_field
            && !spas
            && matches!(mode, 1 | 2)
            && flag("ManualLaneChangeIndicator")
            && ok
            && model_fresh
            && taps.is_empty()
            && hold.is_none()
            && observed::vehicle_allowed(&car, boundary_direction)
            && snapshot
                .as_ref()
                .is_some_and(|(model, _)| adjacent_lane(&car, model, boundary_direction, now))
            && meta.is_some_and(|m| matches!(m.lane_change_state, "off" | "preLaneChange"))
            && peer_fresh
            && feedback.state == "ready";
        let boundary_commands = self.boundary.read(
            now,
            boundary_direction,
            meta.map_or(0., |m| m.manual_lane_mono_time),
            boundary_allowed,
            &mut |v| record_ownership(now, v),
        );
        taps.extend(boundary_commands);
        let peer = if peer_fresh {
            feedback.state
        } else {
            "missing"
        };
        self.display.update(DisplayInput {
            now,
            mode,
            ok,
            car: &car,
            meta,
            hold: hold.as_ref(),
            activated: hold
                .as_ref()
                .is_some_and(|h| self.activated_id.as_deref() == Some(&h.id)),
            peer,
            automatic_reason: self.automatic.reason,
            tbt_phase: self.tbt.phase,
            tbt_reason: self.tbt.reason,
            tbt_enabled: tbt_enabled && auto_fresh,
            front_clear,
            observed_source: &observed_source,
            auto_valid,
            atc_type: if auto_fresh {
                nav.as_ref().map_or("", |n| n.atc_type.as_str())
            } else {
                ""
            },
        });
        let devices = bluetooth.devices(now);
        // Physical, local-only service test: a fresh nonce permits one tap
        // while parked, rechecking P/standstill/control for its whole lease.
        if ble {
            let trial = read_json(&Path::new(RUNTIME).join("b1-park-test.json"));
            let id = trial.get("id").and_then(Value::as_str).map(str::to_owned);
            let time = trial
                .get("time")
                .and_then(Value::as_f64)
                .unwrap_or(f64::NAN);
            let direction = trial
                .get("direction")
                .and_then(Value::as_str)
                .and_then(Side::parse);
            if id.is_some() && id != self.last_trial && fresh(time, now, 0.4) {
                if let (Some(text), Some(_)) = (&id, direction) {
                    if text.starts_with("park-test:") {
                        self.last_trial.clone_from(&id);
                        self.trial_until = now + 1.;
                    }
                }
            }
            if now < self.trial_until {
                let engaged = control.is_some_and(|cc| {
                    cc.get_enabled() || cc.get_lat_active() || cc.get_long_active()
                }) || selfdrive.is_some_and(|ss| ss.get_enabled() || ss.get_active());
                ok = ok
                    && gear == "park"
                    && cs.get_standstill()
                    && f64::from(cs.get_v_ego()).abs() < 0.01
                    && !engaged;
                taps = match (ok, &id, direction) {
                    (true, Some(text), Some(d))
                        if id == self.last_trial && fresh(time, now, 0.4) =>
                    {
                        vec![Command {
                            id: text.clone(),
                            time,
                            direction: d,
                            device: None,
                        }]
                    }
                    _ => Vec::new(),
                };
                hold = None;
            }
        }
        // wire_hold: observation and low-speed TBT holds travel as "lane".
        let origin = hold.as_ref().map(|h| h.source.clone());
        let wire = hold.clone().map(|mut h| {
            if matches!(h.source.as_str(), "observation" | "tbt_display") {
                h.source = "lane".into();
            }
            h
        });
        self.record = Some(Record {
            stamp: now,
            ok,
            hold: wire,
            events: taps,
            turn_phase: self.automatic.reason,
            devices,
            activated: self.activated_id.is_some(),
            relights: self.policy.relights,
            b1_hold: ble && self.fingerprint != PE,
            turn_position: false,
            tbt_phase: self.tbt.reason,
            observation_phase: observed_source,
            indicator_origin: origin,
        });
        if let (Some(vh), Some(h)) = (&voice_hold, &hold) {
            if vh.id == h.id {
                // The board's own lamp observation can supply the missing lamp sample.
                let receiver = if ble && feedback.lamps.is_some() {
                    feedback.accepted_id.clone()
                } else {
                    self.activated_id.clone()
                };
                let confirmation = ok
                    && acknowledged
                    && self
                        .voice
                        .confirmed(Some(h), receiver.as_deref(), feedback, &car);
                let tap = self.voice_tap.sent(now, feedback, ok);
                let _ = atomic_json(
                    &Path::new(ASSISTANT_LANE_ROOT).join("signal.json"),
                    &json!({"id": self.voice.request, "route": self.voice.route,
                        "direction": self.voice.direction.map(Side::name), "time": now,
                        "confirmed": confirmation,
                        "board_lamp": ble && confirmation && feedback.lamps.is_some(), "tap": tap}),
                );
            }
        }
        if mode != 0 && !spas {
            let direction = hold
                .as_ref()
                .filter(|h| self.activated_id.as_deref() == Some(&h.id))
                .map(|h| h.direction.name());
            let tap = self.observation_feedback.sent(now, feedback, ok);
            let _ = atomic_json(
                Path::new(HOLD_PATH),
                &json!({"stamp": now, "mode": mode, "fingerprint": self.fingerprint,
                    "direction": direction, "tap": tap,
                    "override": self.activated_id.is_some() || now < self.release_until
                        || (self.policy.suppressed && self.release_until > 0.)}),
            );
        }
        Ok(())
    }
}
