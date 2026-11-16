//! Comma SPAS transport for the physical-indicator intent policy.
//! Port of carrot's `openpilot/selfdrive/car/hyundai_spas.py` (`SpasSignals`)
//! and `opendbc/car/hyundai/stalk_trial.py`.
//!
//! The intent source (`tools/jetlink/blink_source.py`, a separate worker in
//! carrot) runs in-process as `crate::blink::source::SpasWorker`.
use super::{
    bus::CanBus,
    canfd_maintenance,
    spas::{self, Direction, PatchSequence, Session, Update, SPAS},
    wire::CanWriter,
    Error,
};
use openpilot_can::{Frame, Packet};
use openpilot_cereal::{
    car_capnp::car_state,
    log_capnp::{event, panda_state},
};
use openpilot_messaging::state::State as Subscribers;
use openpilot_params::Params;
use std::collections::BTreeMap;

const LEVER: u32 = 0x3c1;

fn lever_crc(body: &[u8]) -> u8 {
    let mut value = 0u8;
    for &byte in body {
        value ^= byte;
        for _ in 0..8 {
            value = (value << 1) ^ if value & 0x80 != 0 { 0x1d } else { 0 };
        }
    }
    value ^ 0x65
}

/// joongyu01's counter-aware lever replay, for the bounded parked trial.
pub fn replay(original: &[u8], direction: Direction, bus: u8) -> Option<Frame> {
    if original.len() != 8
        || original[0] != lever_crc(&original[1..])
        || original[3] & 0x40 != 0
        || original[4] & 0x51 != 0
    {
        return None;
    }
    let mut data = original.to_vec();
    data[1] = ((((data[1] >> 4) + 1) % 16) << 4) | (data[1] & 15);
    data[3] = (data[3] & !0x40)
        | if direction == Direction::Left {
            0x40
        } else {
            0
        };
    data[4] = (data[4] & !0x51) | u8::from(direction == Direction::Right);
    data[0] = lever_crc(&data[1..]);
    Some(Frame {
        address: LEVER,
        data,
        bus,
    })
}

fn to_direction(side: crate::blink::Side) -> Direction {
    match side {
        crate::blink::Side::Left => Direction::Left,
        crate::blink::Side::Right => Direction::Right,
    }
}

pub struct Inputs<'a> {
    pub cs: car_state::Reader<'a>,
    pub subscribers: &'a Subscribers,
    pub settings: &'a Params,
    pub now: f64,
    pub initialized: bool,
    pub controller_frame: u64,
    pub writer: &'a mut CanWriter,
    pub bus: CanBus,
}

pub struct SpasSignals {
    pub enabled: bool,
    pub session: Session,
    settings_at: f64,
    pub test_setting: i32,
    pub test_request: Option<String>,
    test_until: f64,
    test_direction: Option<Direction>,
    test_pulse: bool,
    pub test_direct: bool,
    test_hold: bool,
    direct_next: f64,
    direct_started: f64,
    direct_cooldown: f64,
    direct_counter: [u8; 2],
    phase_sent: BTreeMap<u32, (f64, f64, u32)>,
    phase_until: f64,
    phase_confirmed: bool,
    stock_received: BTreeMap<u32, f64>,
    patch: Option<PatchSequence>,
    controller_frame: u64,
    last_update: Option<f64>,
    test_blocked: Vec<u32>,
    test_can_health: Option<Vec<u32>>,
    test_serial: u64,
    stalks: Option<(u8, u8)>,
    driver_override: bool,
    pub hold_value: u8,
    hold_supported: bool,
    pulse_started: Option<f64>,
    lamp_confirmed: bool,
    driver_data: Option<Vec<u8>>,
    trial_stalk: Option<Vec<u8>>,
    driver_at: f64,
    driver_counter: i32,
    driver_count: u32,
    driver_active: bool,
    pub status: String,
    /// Driving lane-change indicator on a car with no stock SPAS sender.
    pub solo: spas::SoloDrive,
    worker: Option<crate::blink::source::SpasWorker>,
    feedback: crate::blink::source::Feedback,
    feedback_at: f64,
}

/// The lane change direction while one is under way (pre, starting or
/// finishing), from a fresh modelV2.
fn lane_change(subscribers: &Subscribers, now: f64) -> Option<Direction> {
    use openpilot_cereal::log_capnp::{LaneChangeDirection, LaneChangeState};
    if !fresh(subscribers, "modelV2", now, 0.35) {
        return None;
    }
    let topic = subscribers.topic("modelV2").ok()?;
    let event::Which::ModelV2(Ok(model)) = topic.event().ok()?.which().ok()? else {
        return None;
    };
    let meta = model.get_meta().ok()?;
    match meta.get_lane_change_state().ok()? {
        LaneChangeState::PreLaneChange
        | LaneChangeState::LaneChangeStarting
        | LaneChangeState::LaneChangeFinishing => {}
        _ => return None,
    }
    match meta.get_lane_change_direction().ok()? {
        LaneChangeDirection::Left => Some(Direction::Left),
        LaneChangeDirection::Right => Some(Direction::Right),
        _ => None,
    }
}

fn fresh(subscribers: &Subscribers, name: &str, now: f64, limit: f64) -> bool {
    subscribers.topic(name).is_ok_and(|topic| {
        topic.valid && topic.alive && (0. ..limit).contains(&(now - topic.receive_time))
    })
}

fn pandas<'a>(
    subscribers: &'a Subscribers,
) -> Option<capnp::struct_list::Reader<'a, panda_state::Owned>> {
    let topic = subscribers.topic("pandaStates").ok()?;
    match topic.event().ok()?.which().ok()? {
        event::Which::PandaStates(list) => list.ok(),
        _ => None,
    }
}

/// carrot `SpasSignals.can_health`: error counters of every physical
/// controller, or None while any controller is passive, warning or off.
fn can_health(subscribers: &Subscribers) -> Option<Vec<u32>> {
    let list = pandas(subscribers)?;
    if list.is_empty() {
        return None;
    }
    let mut counts = Vec::new();
    for panda in list.iter() {
        for state in [
            panda.get_can_state0().ok()?,
            panda.get_can_state1().ok()?,
            panda.get_can_state2().ok()?,
        ] {
            if state.get_bus_off() || state.get_error_warning() || state.get_error_passive() {
                return None;
            }
            counts.extend([
                state.get_total_error_cnt(),
                state.get_bus_off_cnt(),
                state.get_total_tx_lost_cnt(),
                state.get_total_rx_lost_cnt(),
            ]);
        }
    }
    Some(counts)
}

fn tx_blocked(subscribers: &Subscribers) -> Vec<u32> {
    pandas(subscribers).map_or_else(Vec::new, |list| {
        list.iter().map(|p| p.get_safety_tx_blocked()).collect()
    })
}

impl SpasSignals {
    pub fn new(enabled: bool, bus: u8, candidate: &str) -> Self {
        Self {
            enabled,
            session: Session::new(bus),
            settings_at: 0.,
            test_setting: 0,
            test_request: None,
            test_until: 0.,
            test_direction: None,
            test_pulse: false,
            test_direct: false,
            test_hold: false,
            direct_next: 0.,
            direct_started: 0.,
            direct_cooldown: 0.,
            direct_counter: [0; 2],
            phase_sent: BTreeMap::new(),
            phase_until: 0.,
            phase_confirmed: false,
            stock_received: BTreeMap::new(),
            patch: None,
            controller_frame: 0,
            last_update: None,
            test_blocked: Vec::new(),
            test_can_health: None,
            test_serial: 0,
            stalks: None,
            driver_override: false,
            hold_value: 1,
            hold_supported: candidate == "HYUNDAI_IONIQ_5_PE",
            pulse_started: None,
            lamp_confirmed: false,
            driver_data: None,
            trial_stalk: None,
            driver_at: f64::NEG_INFINITY,
            driver_counter: -1,
            driver_count: 0,
            driver_active: false,
            status: if enabled {
                "waiting_stock".into()
            } else {
                "disabled".into()
            },
            worker: None,
            solo: spas::SoloDrive::default(),
            feedback: crate::blink::source::Feedback::default(),
            feedback_at: 0.,
        }
    }

    pub fn observe(&mut self, packets: &[Packet], now: f64) {
        if !self.enabled {
            return;
        }
        for packet in packets {
            let received = packet.mono_time as f64 * 1e-9;
            for frame in &packet.frames {
                self.session
                    .observe(frame.address, frame.bus, &frame.data, now);
                if frame.bus != self.session.bus {
                    continue;
                }
                if SPAS.iter().any(|(a, _)| *a == frame.address)
                    && (0. ..0.1).contains(&(now - received))
                {
                    self.stock_received.insert(frame.address, received);
                }
                if frame.address == LEVER {
                    let data = &frame.data;
                    if data.len() == 8 && data[0] == lever_crc(&data[1..]) {
                        let count = i32::from(data[1] >> 4);
                        // The event counter is constant while idle; repeated
                        // valid 5 Hz reception is what makes it fresh.
                        let continuous = (0. ..0.5).contains(&(now - self.driver_at));
                        self.driver_count = if continuous {
                            (self.driver_count + 1).min(10)
                        } else {
                            1
                        };
                        self.driver_data = Some(data.clone());
                        self.driver_at = now;
                        self.driver_counter = count;
                        self.driver_active = data[3] & 0x40 != 0 || data[4] & 0x51 != 0;
                        self.driver_override |= self.driver_active;
                    } else {
                        self.driver_count = 0;
                    }
                }
            }
        }
    }

    fn parked(&self, healthy: bool, input: &Inputs<'_>) -> Result<bool, Error> {
        let (cs, subs, now) = (input.cs, input.subscribers, input.now);
        // A missing or undecodable service is never "parked".
        let control = subs.topic("carControl").ok().and_then(|topic| {
            match topic.event().ok()?.which().ok()? {
                event::Which::CarControl(value) => value.ok(),
                _ => None,
            }
        });
        let selfdrive = subs.topic("selfdriveState").ok().and_then(|topic| {
            match topic.event().ok()?.which().ok()? {
                event::Which::SelfdriveState(value) => value.ok(),
                _ => None,
            }
        });
        let (Some(control), Some(selfdrive)) = (control, selfdrive) else {
            return Ok(false);
        };
        let pandas = pandas(subs);
        Ok(healthy
            && cs.get_gear_shifter()? == car_state::GearShifter::Park
            && cs.get_standstill()
            && cs.get_v_ego().abs() < 0.01
            && !(control.get_lat_active()
                || control.get_long_active()
                || selfdrive.get_active()
                || selfdrive.get_enabled())
            && fresh(subs, "pandaStates", now, 0.75)
            && pandas.is_some_and(|list| {
                !list.is_empty() && !list.iter().any(|p| p.get_controls_allowed())
            }))
    }

    pub fn update(&mut self, input: Inputs<'_>) -> Result<Vec<Frame>, Error> {
        if !self.enabled {
            return Ok(Vec::new());
        }
        let now = input.now;
        let cs = input.cs;
        let controller_frame = input.controller_frame;
        self.controller_frame = controller_frame + 1;
        let continuous = self
            .last_update
            .is_some_and(|last| (0. ..0.2).contains(&(now - last)));
        self.last_update = Some(now);
        let gear = cs.get_gear_shifter()?;
        let mut healthy = input.initialized
            && cs.get_can_valid()
            && cs.get_v_ego().is_finite()
            && self.driver_count >= 3
            && (0. ..0.5).contains(&(now - self.driver_at))
            && matches!(
                gear,
                car_state::GearShifter::Park
                    | car_state::GearShifter::Neutral
                    | car_state::GearShifter::Drive
            )
            && fresh(input.subscribers, "carControl", now, 0.35)
            && fresh(input.subscribers, "selfdriveState", now, 0.35);
        let stalks = (
            cs.get_left_blinker_stalk_count(),
            cs.get_right_blinker_stalk_count(),
        );
        let override_now = cs.get_blinker_lever() != 0
            || (cs.get_left_blinker() && cs.get_right_blinker())
            || self.stalks.is_some_and(|previous| previous != stalks);
        self.stalks = Some(stalks);
        if override_now {
            self.driver_override = true;
        }
        if self.session.state == spas::State::Idle
            && !override_now
            && !self.driver_active
            && self.driver_count >= 3
        {
            self.driver_override = false;
        }
        if now >= self.settings_at {
            let setting = super::parameters::setting_int(input.settings, "CanfdBlinkerTest")?;
            if setting != self.test_setting && (1..21).contains(&setting) {
                // Consume the edge even when the parked guard fails; never run later.
                let parked = self.parked(healthy, &input)?;
                let direct = setting >= 5;
                // 19/20: a car with no stock SPAS sender (HDA1, no APRK).
                let solo = [19, 20].contains(&setting);
                let stock_ok = if solo {
                    self.session.stock_absent(now)
                } else {
                    self.session.stock_ready(now) && self.session.stock_direction == Some(0)
                };
                let direct_ready = !direct
                    || (now >= self.direct_cooldown
                        && can_health(input.subscribers).is_some()
                        && stock_ok
                        && !(cs.get_left_blinker() || cs.get_right_blinker()));
                if parked
                    && (setting <= 2 || self.hold_supported)
                    && self.session.state == spas::State::Idle
                    && direct_ready
                {
                    self.test_serial += 1;
                    self.test_request = Some(format!("park:{}", self.test_serial));
                    self.test_until = now
                        + if [7, 8, 11, 12, 13, 14, 15, 16, 17, 18].contains(&setting) {
                            10.
                        } else {
                            3.
                        };
                    self.trial_stalk = if [17, 18].contains(&setting) {
                        self.driver_data.clone()
                    } else {
                        None
                    };
                    self.test_direction = Some(if setting % 2 == 1 {
                        Direction::Left
                    } else {
                        Direction::Right
                    });
                    self.test_pulse = [3, 4, 7, 8, 9, 10, 11, 12].contains(&setting);
                    self.test_direct = direct;
                    self.test_hold = [3, 4, 7, 8, 11, 12].contains(&setting);
                    self.test_blocked = tx_blocked(input.subscribers);
                    if [15, 16].contains(&setting) && self.patch.is_none() {
                        self.patch = Some(PatchSequence::new(self.session.bus));
                    }
                    if direct {
                        self.test_can_health = can_health(input.subscribers);
                        self.direct_started = now;
                        self.direct_next = now;
                        self.phase_sent.clear();
                        self.phase_until = now + 0.5;
                        self.phase_confirmed = false;
                        self.direct_cooldown = now + 30.;
                        self.direct_counter = self.session.next_counters();
                    }
                    self.pulse_started = None;
                    self.lamp_confirmed = false;
                }
            }
            self.test_setting = setting;
            self.settings_at = now + 0.1;
        }
        if self.worker.is_none() && input.initialized {
            self.worker = Some(crate::blink::source::Worker::new(
                crate::blink::source::Mode::Spas,
                String::new(),
                false,
            ));
        }
        if let Some(worker) = &mut self.worker {
            // A failed cycle publishes no intent rather than a stale one.
            if worker
                .update(
                    now,
                    cs,
                    true,
                    input.subscribers,
                    input.settings,
                    &self.feedback,
                )
                .is_err()
            {
                worker.record = None;
            }
        }
        let mut request: Option<String> = None;
        let mut direction: Option<Direction> = None;
        if let Some(record) = self.worker.as_ref().and_then(|w| w.record.as_ref()) {
            if (0. ..0.2).contains(&(now - record.stamp)) && record.ok {
                if let Some(hold) = &record.hold {
                    request = Some(hold.id.clone());
                    direction = Some(to_direction(hold.direction));
                } else if let Some(event) = record
                    .events
                    .iter()
                    .rev()
                    .find(|event| (0. ..0.4).contains(&(now - event.time)))
                {
                    request = Some(event.id.clone());
                    direction = Some(to_direction(event.direction));
                }
            }
        }
        // A car with no stock SPAS sender lights automatic lane changes
        // directly, until the lane change completes (owner's car only).
        let solo_ready = self.hold_supported
            && self.test_request.is_none()
            && self.test_setting == 0
            && now >= self.direct_cooldown
            && self.session.state == spas::State::Idle
            && self.session.stock_absent(now);
        if solo_ready || self.solo.busy() {
            let want = lane_change(input.subscribers, now);
            let allowed = solo_ready && healthy && continuous;
            let driver = override_now || self.driver_active;
            let reason = if !solo_ready {
                "conditions"
            } else if !healthy {
                "inputs"
            } else if !continuous {
                "loop"
            } else {
                "lane_change_end"
            };
            let (started, frames) =
                self.solo
                    .update(now, want, allowed, driver, reason, self.session.bus);
            if let Some(side) = started {
                // Our lamp must not come back as a driver lane change request.
                let direction = if side == Direction::Left { 1 } else { 2 };
                let _ = crate::blink::source::record_ownership(
                    now,
                    &serde_json::json!({"time": now, "direction": direction, "source": "spas_solo"}),
                );
            }
            if self.solo.busy() || !frames.is_empty() {
                self.hold_value = 1;
                self.status = if self.solo.status == "tail" {
                    format!("solo_drive_tail_{}", self.solo.ended)
                } else {
                    format!("solo_drive_{}", self.solo.status)
                };
                return Ok(frames);
            }
        }
        if self.test_request.is_some() {
            healthy = self.parked(healthy, &input)?;
            if now < self.test_until && self.test_setting != 0 {
                request.clone_from(&self.test_request);
                direction = self.test_direction;
            } else {
                request = None;
                direction = None;
            }
        }
        let same_guard = |this: &Self| {
            this.test_can_health.is_some()
                && can_health(input.subscribers) == this.test_can_health
                && this.test_blocked == tx_blocked(input.subscribers)
        };
        if [17, 18].contains(&self.test_setting) || self.trial_stalk.is_some() {
            let allowed = [17, 18].contains(&self.test_setting)
                && self.test_request.is_some()
                && request == self.test_request
                && healthy
                && continuous
                && !self.driver_override
                && self.trial_stalk.is_some()
                && self.driver_data == self.trial_stalk
                && same_guard(self);
            self.hold_value = 1;
            self.status = if allowed {
                "stalk_trial"
            } else {
                "stalk_trial_ended"
            }
            .into();
            if !allowed {
                self.test_until = self.test_until.min(now);
                self.test_request = None;
                self.trial_stalk = None;
            } else if now >= self.direct_next {
                self.direct_next = now + 0.05;
                if let (Some(stalk), Some(side)) = (&self.trial_stalk, direction) {
                    return Ok(replay(stalk, side, self.session.bus).into_iter().collect());
                }
            }
            return Ok(Vec::new());
        }
        if let Some(patch) = &self.patch {
            if patch.active() || [15, 16].contains(&self.test_setting) {
                let allowed = [15, 16].contains(&self.test_setting)
                    && self.test_request.is_some()
                    && request == self.test_request
                    && healthy
                    && continuous
                    && !self.driver_override
                    && same_guard(self);
                let writer = input.writer;
                let bus = input.bus;
                let side = direction;
                let mut error = None;
                let sends =
                    self.patch
                        .as_mut()
                        .expect("checked")
                        .step(controller_frame, allowed, |_| {
                            canfd_maintenance::spas(
                                writer,
                                bus,
                                [
                                    side == Some(Direction::Left),
                                    side == Some(Direction::Right),
                                ],
                            )
                            .unwrap_or_else(|e| {
                                error = Some(e);
                                Vec::new()
                            })
                        });
                if let Some(error) = error {
                    return Err(error);
                }
                self.hold_value = 1;
                self.status = if allowed {
                    "patch_sequence"
                } else {
                    "patch_sequence_ended"
                }
                .into();
                if !allowed {
                    self.test_until = self.test_until.min(now);
                    self.test_request = None;
                }
                return Ok(sends);
            }
        }
        let direct = self.test_request.is_some() && self.test_direct;
        // A selected parked direct test never falls through to automatic/UDS intent.
        let parked_selected = direct || self.test_setting >= 5 || now < self.direct_cooldown;
        let mut sends = self.session.update(
            now,
            Update {
                healthy,
                request: if parked_selected {
                    None
                } else {
                    request.as_deref()
                },
                direction: if parked_selected { None } else { direction },
                duration: if self.test_request.is_some() {
                    3.
                } else {
                    300.
                },
                driver_override: self.driver_override,
            },
        );
        // Suppresses the old automatic-lane hold while SPAS is selected.
        self.hold_value = 1;
        let direct_active = direct
            && request == self.test_request
            && healthy
            && !self.driver_override
            && same_guard(self)
            && continuous
            && if [19, 20].contains(&self.test_setting) {
                self.session.stock_absent(now)
            } else {
                self.session.stock_ready(now) && self.session.stock_direction == Some(0)
            };
        if direct && !direct_active {
            // No resumption after guard loss.
            self.test_until = self.test_until.min(now);
        }
        let pulse_active = if [11, 12].contains(&self.test_setting) {
            now < self.phase_until
        } else {
            now - self.direct_started < 0.2
        };
        if direct_active && now >= self.direct_next && (!self.test_pulse || pulse_active) {
            if [11, 12, 13, 14].contains(&self.test_setting) {
                // At most one packet per new stock RX within the trial.
                for (i, (address, _)) in SPAS.iter().enumerate() {
                    let received = self
                        .stock_received
                        .get(address)
                        .copied()
                        .unwrap_or(f64::NEG_INFINITY);
                    let (previous_rx, previous_tx, count) = self
                        .phase_sent
                        .get(address)
                        .copied()
                        .unwrap_or((f64::NEG_INFINITY, f64::NEG_INFINITY, 0));
                    let limit = if [13, 14].contains(&self.test_setting) {
                        500
                    } else {
                        25
                    };
                    if count < limit
                        && self.direct_started.max(previous_rx) < received
                        && received <= now
                        && now - received <= 0.005
                        && now - previous_tx >= 0.015
                        && (*address == 0x165 || self.phase_sent.contains_key(&0x165))
                    {
                        let counter = if count == 0 {
                            self.session.next_counters()[i]
                        } else {
                            self.direct_counter[i]
                        };
                        sends.push(spas::frame(*address, counter, direction, self.session.bus));
                        self.direct_counter[i] = counter.wrapping_add(1);
                        self.phase_sent.insert(*address, (received, now, count + 1));
                    }
                }
            } else {
                let side = if [19, 20].contains(&self.test_setting) {
                    spas::solo_direction(
                        now - self.direct_started,
                        self.test_until - self.direct_started,
                        direction,
                    )
                } else {
                    direction
                };
                for (i, (address, _)) in SPAS.iter().enumerate() {
                    sends.push(spas::frame(
                        *address,
                        self.direct_counter[i],
                        side,
                        self.session.bus,
                    ));
                    self.direct_counter[i] = self.direct_counter[i].wrapping_add(1);
                }
                self.direct_next = now + 0.02;
            }
        }
        if (direct_active || self.session.state == spas::State::Active)
            && self.test_request.is_some()
            && self.test_pulse
        {
            let started = *self.pulse_started.get_or_insert(now);
            self.lamp_confirmed |= (direction == Some(Direction::Left)
                && cs.get_left_blinker()
                && !cs.get_right_blinker())
                || (direction == Some(Direction::Right)
                    && cs.get_right_blinker()
                    && !cs.get_left_blinker());
            if self.lamp_confirmed && self.test_hold {
                self.hold_value = if direction == Some(Direction::Left) {
                    2
                } else {
                    3
                };
                if [11, 12].contains(&self.test_setting) && !self.phase_confirmed {
                    self.phase_until = self.phase_until.min(now + 0.1);
                    self.phase_confirmed = true;
                }
            }
            if !direct && now - started >= 0.2 {
                // Keep the session alive at its cadence but drop the
                // directional command; only the confirmed 0x1EA hold remains.
                sends = sends
                    .into_iter()
                    .map(|m| {
                        if m.address == 0x16a {
                            spas::frame(m.address, m.data[2], None, m.bus)
                        } else {
                            m
                        }
                    })
                    .collect();
            }
        }
        self.status = if direct {
            if direct_active {
                "parked_direct".into()
            } else {
                "parked_direct_ended".into()
            }
        } else if self.session.reason.is_empty() {
            self.session.state.name().into()
        } else {
            self.session.reason.clone()
        };
        if self.worker.is_some() && now >= self.feedback_at {
            let active = self.session.state == spas::State::Active;
            self.feedback = crate::blink::source::Feedback {
                lamps: None,
                stamp: now,
                accepted_id: if active {
                    self.session.request().map(str::to_owned)
                } else {
                    None
                },
                available: healthy
                    && self.test_setting == 0
                    && (self.session.stock_ready(now) || active),
                state: if self.driver_override {
                    "driver_override"
                } else if active {
                    "ready"
                } else {
                    "waiting"
                },
            };
            self.feedback_at = now + 0.05;
        }
        if self.test_request.is_some()
            && matches!(self.session.state, spas::State::Idle | spas::State::Failed)
            && now >= self.test_until
        {
            self.test_request = None;
        }
        Ok(sends)
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn stalk_replay_matches_contributor_and_preserves_knob() {
        let original = [0xf8, 0x80, 0x30, 0, 0, 0, 0, 0];
        let hex: String = replay(&original, Direction::Left, 0)
            .unwrap()
            .data
            .iter()
            .map(|b| format!("{b:02x}"))
            .collect();
        assert_eq!(hex, "de90304000000000");
        for knob in [0u8, 0x10, 0x20, 0x24, 0x30, 0x34] {
            for counter in 0..16u8 {
                let mut data = original;
                data[1] = counter << 4;
                data[2] = knob;
                data[0] = lever_crc(&data[1..]);
                for side in [Direction::Left, Direction::Right] {
                    let out = replay(&data, side, 0).unwrap().data;
                    assert_eq!(out[2], knob);
                    assert_eq!(out[1] >> 4, (counter + 1) % 16);
                    assert_eq!(out[0], lever_crc(&out[1..]));
                }
            }
        }
    }
}
