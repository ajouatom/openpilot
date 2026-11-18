//! Native CAN-FD lane-change lamp "Kick" (carrot PR580 and follow-ups:
//! 00d9cae3b, 6393f9518, 2accb1fa3, a31a59ca2, 9ebd9326e, 6af3073e3,
//! acb8816fe, 0dae168f6).
//!
//! Port of carrot's `openpilot/selfdrive/car/hyundai_kick.py` (`KickOutput`)
//! and `carcontroller.aprk_blinkers`. For a camera-SCC car whose stock SPAS
//! sender (APRK) is on ECAN: the common indicator policy worker runs in Kick
//! mode, a fresh hold or tap becomes SPAS2 BLINKER_CONTROL, and the parking
//! ECU is asked to go quiet around it (`10 03`, `28 83 01`) or, unanswered
//! and with `CanfdBlinkerKickMode` 1, panda holds each of our SPAS frames and
//! sends it right behind the stock one (`CANFD_BLINKER_KICK`).
//!
//! It complements the SPAS solo/lease path (`spas_signals`), which owns cars
//! without a stock SPAS sender; the two flags are exclusive at fingerprint.
use super::{
    bus::CanBus,
    canfd_maintenance,
    spas::{self, Session, SPAS},
    spas_signals::lever_crc,
    wire::CanWriter,
    Error,
};
use crate::blink::{
    owner::Owner,
    source::{record_ownership, Feedback, Mode, Record, Worker},
    Side,
};
use openpilot_can::{Frame, Packet};
use openpilot_cereal::car_capnp::car_state;
use openpilot_messaging::state::State as Subscribers;
use openpilot_params::Params;
use serde_json::json;
use std::collections::VecDeque;

const LEVER: u32 = 0x3c1;
/// `10 03`: extended session; `28 83 01`: communication control off,
/// no response requested; `10 01`: default session.
const APRK_COM_DISABLE: [u8; 8] = [0x03, 0x28, 0x83, 0x01, 0, 0, 0, 0];
/// 0.4 s at 100 Hz to answer `10 03`.
const APRK_REPLY_FRAMES: u32 = 40;
/// The stock SPAS stream must stay off ECAN this long before we send.
const APRK_QUIET: f64 = 0.1;

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum Aprk {
    Idle,
    Ask,
    WaitQuiet,
    Active,
    Failed,
}

fn fresh(stamp: f64, now: f64, ttl: f64) -> bool {
    stamp.is_finite() && (0. ..ttl).contains(&(now - stamp))
}

fn side_code(side: Side) -> u8 {
    match side {
        Side::Left => 3,
        Side::Right => 4,
    }
}

pub struct Kick {
    /// `CANFD_BLINKER_KICK` in the car's safety config.
    pub configured: bool,
    bus: CanBus,
    owner: Owner,
    contract: Session,
    contract_admitted: bool,
    next_update: f64,
    last_journal: f64,
    journal_direction: u8,
    driver_stamp: f64,
    driver_pressed_at: f64,
    driver_direction: Option<u8>,
    driver_counts: [u8; 2],
    source_counts: Option<[u8; 2]>,
    worker: Option<Worker>,
    record: Option<Record>,
    seen: VecDeque<String>,
    pub request_id: Option<String>,
    request_side: u8,
    accepted_id: Option<String>,
    last_sent: f64,
    first_sent: Option<f64>,
    request_started: f64,
    tap_until: f64,
    tapping: bool,
    ready: bool,
    feedback: Feedback,
    feedback_at: f64,
    // Settings, refreshed every 0.5 s.
    settings_at: f64,
    kick_setting: bool,
    onroad: bool,
    kick_mode: i32,
    // CAN timing for the APRK handshake (carstate `_update_aprk`).
    can_at: f64,
    stock_at: f64,
    ack_at: f64,
    // carcontroller `aprk_blinkers`.
    pub aprk: Aprk,
    aprk_frame: u32,
    aprk_at: f64,
    aprk_acked: bool,
    aprk_buses: Vec<u8>,
    aprk_request_id: Option<String>,
    /// The direction this cycle's request asks for (3 left, 4 right, 0).
    pub direction: u8,
    pub output_ready: bool,
}

impl Kick {
    pub fn new(configured: bool, bus: CanBus) -> Self {
        Self {
            configured,
            bus,
            owner: Owner::new("CANFD Kick"),
            contract: Session::new(bus.ecan),
            contract_admitted: false,
            next_update: 0.,
            last_journal: 0.,
            journal_direction: 0,
            driver_stamp: -1.,
            driver_pressed_at: -1.,
            driver_direction: None,
            driver_counts: [0; 2],
            source_counts: None,
            worker: None,
            record: None,
            seen: VecDeque::with_capacity(256),
            request_id: None,
            request_side: 0,
            accepted_id: None,
            last_sent: -1.,
            first_sent: None,
            request_started: 0.,
            tap_until: 0.,
            tapping: false,
            ready: false,
            feedback: Feedback::default(),
            feedback_at: 0.,
            settings_at: f64::NEG_INFINITY,
            kick_setting: false,
            onroad: false,
            kick_mode: 1,
            can_at: 0.,
            stock_at: 0.,
            ack_at: 0.,
            aprk: Aprk::Idle,
            aprk_frame: 0,
            aprk_at: 0.,
            aprk_acked: false,
            aprk_buses: Vec::new(),
            aprk_request_id: None,
            direction: 0,
            output_ready: false,
        }
    }

    /// `CanfdBlinkerKick` can switch the Kick off mid-drive; switching it on
    /// waits for the next start (the safety flag is set at fingerprint).
    pub fn refresh(&mut self, settings: &Params, now: f64) {
        if now - self.settings_at < 0.5 {
            return;
        }
        self.settings_at = now;
        self.kick_setting = settings.get_bool("CanfdBlinkerKick").unwrap_or(false);
        self.onroad = settings.get_bool("IsOnroad").unwrap_or(false);
        self.kick_mode =
            super::parameters::setting_int(settings, "CanfdBlinkerKickMode").unwrap_or(1);
    }

    fn enabled(&self) -> bool {
        self.configured && self.onroad && self.kick_setting
    }

    fn exclusive(&self, now: f64, can_valid: bool) -> bool {
        self.owner.held && fresh(self.owner.renewed(), now, 0.2) && self.enabled() && can_valid
    }

    /// carrot `observe_driver` plus carstate `_update_aprk`. Rewrites the
    /// lever fields of this frame's carState.
    pub fn observe(&mut self, packets: &[Packet], mut cs: car_state::Builder<'_>, now: f64) {
        if !self.configured {
            return;
        }
        let ecan = self.bus.ecan;
        for packet in packets {
            let received = packet.mono_time as f64 * 1e-9;
            self.can_at = self.can_at.max(received);
            for frame in &packet.frames {
                if frame.bus == ecan && SPAS.iter().any(|(a, _)| *a == frame.address) {
                    self.stock_at = self.stock_at.max(received);
                } else if frame.address == spas::RESPONSE
                    && (frame.bus == ecan || frame.bus == self.bus.acan)
                    && frame.data.len() >= 3
                    && frame.data[1] == 0x50
                    && frame.data[2] == 0x03
                {
                    self.ack_at = self.ack_at.max(received);
                }
            }
        }
        let can_valid = cs.reborrow_as_reader().get_can_valid();
        // USB/BLE can synthesize 0x3C1. Trust this independent lever message
        // only after the Kick holds the shared lease and drained theirs.
        let exclusive = self.exclusive(now, can_valid);
        let source = {
            let reader = cs.reborrow_as_reader();
            [
                reader.get_left_blinker_stalk_count(),
                reader.get_right_blinker_stalk_count(),
            ]
        };
        // Keep ordinary lever events while the Kick is off or awaiting
        // ownership; apply deltas so a source switch cannot invent a press.
        match self.source_counts {
            None => self.driver_counts = source,
            Some(previous) if !exclusive => {
                for i in 0..2 {
                    self.driver_counts[i] = self.driver_counts[i]
                        .wrapping_add(source[i])
                        .wrapping_sub(previous[i]);
                }
            }
            Some(_) => {}
        }
        self.source_counts = Some(source);
        if !exclusive || !fresh(self.driver_stamp, now, 0.35) {
            self.driver_direction = None;
        }
        if exclusive {
            for packet in packets {
                let stamp = packet.mono_time as f64 * 1e-9;
                for frame in &packet.frames {
                    if frame.bus != ecan {
                        continue;
                    }
                    if SPAS.iter().any(|(a, _)| *a == frame.address) {
                        if fresh(stamp, now, 0.1) {
                            self.contract
                                .observe(frame.address, frame.bus, &frame.data, stamp);
                            let run = self.contract.stock_consecutive(frame.address);
                            if run == 0 || (self.request_id.is_some() && run < 2) {
                                self.contract_admitted = false;
                            }
                        } else {
                            self.contract.break_stock(frame.address);
                            self.contract_admitted = false;
                        }
                    }
                    if frame.address != LEVER {
                        continue;
                    }
                    let data = &frame.data;
                    if data.len() != 8
                        || data[0] != lever_crc(&data[1..])
                        || !fresh(stamp, now, 0.35)
                    {
                        self.driver_direction = None;
                        self.driver_stamp = -1.;
                        continue;
                    }
                    let direction =
                        u8::from(data[3] & 0x40 != 0) | (u8::from(data[4] & 1 != 0) << 1);
                    // Establish neutral after startup, loss or an invalid
                    // frame before accepting a press; count a press and
                    // release that land in one batch.
                    match self.driver_direction {
                        Some(previous) => {
                            if matches!(direction, 1 | 2) && previous & direction == 0 {
                                let i = usize::from(direction - 1);
                                self.driver_counts[i] = self.driver_counts[i].wrapping_add(1);
                                self.driver_pressed_at = now;
                            }
                            self.driver_direction = Some(direction);
                        }
                        None if direction == 0 => self.driver_direction = Some(0),
                        None => {}
                    }
                    self.driver_stamp = stamp;
                }
            }
        }
        let valid =
            exclusive && self.driver_direction.is_some() && fresh(self.driver_stamp, now, 0.35);
        cs.set_turn_signal_driver_valid(valid);
        cs.set_turn_signal_driver_direction(if valid {
            self.driver_direction.unwrap_or(0)
        } else {
            0
        });
        cs.set_left_blinker_stalk_count(self.driver_counts[0]);
        cs.set_right_blinker_stalk_count(self.driver_counts[1]);
    }

    /// carrot `update`: the shared lease and a fresh stock contract.
    pub fn update(&mut self, valid: bool, now: f64) -> bool {
        let enabled = self.enabled() && valid;
        if !enabled || now >= self.next_update {
            self.owner.acquire(enabled, now);
            self.next_update = now + 0.05;
        }
        if !enabled || !self.owner.held {
            self.contract = Session::new(self.bus.ecan);
            self.contract_admitted = false;
        }
        // Ten advancing CRC-valid stock frames before a new request. During
        // an admitted session the ECU may go quiet; the independent lever
        // message must stay fresh throughout.
        let stock_ready = self.contract.stock_ready(now);
        if self.request_id.is_none() {
            self.contract_admitted = stock_ready;
        }
        let driver_active = self.driver_direction.is_some_and(|d| d != 0)
            || fresh(self.driver_pressed_at, now, 0.2);
        let driver_valid = self.driver_direction.is_some() && fresh(self.driver_stamp, now, 0.35);
        self.output_ready =
            enabled && self.owner.held && self.contract_admitted && driver_valid && !driver_active;
        self.output_ready
    }

    /// carrot `request`: the policy worker's fresh hold or tap as a SPAS
    /// direction (3 left, 4 right) while healthy.
    #[allow(clippy::too_many_arguments)]
    pub fn request(
        &mut self,
        cs: car_state::Reader<'_>,
        now: f64,
        initialized: bool,
        subs: &Subscribers,
        settings: &Params,
    ) -> Result<u8, Error> {
        let enabled = self.enabled();
        if !enabled {
            self.worker = None;
            self.record = None;
        } else if initialized && self.worker.is_none() {
            self.worker = Some(Worker::new(Mode::Kick, String::new(), false, false));
        }
        if let Some(worker) = &mut self.worker {
            // A failed cycle publishes no intent rather than a stale one.
            if worker
                .update(now, cs, true, subs, settings, &self.feedback)
                .is_err()
            {
                worker.record = None;
            }
            self.record.clone_from(&worker.record);
        }
        let gear = cs
            .get_gear_shifter()
            .unwrap_or(car_state::GearShifter::Unknown);
        let (left, right) = (cs.get_left_blinker(), cs.get_right_blinker());
        self.ready = enabled
            && self.output_ready
            && initialized
            && cs.get_can_valid()
            && gear == car_state::GearShifter::Drive
            && cs.get_blinker_lever() == 0
            && !(left && right);
        let record = self.record.as_ref();
        let healthy = self.ready && record.is_some_and(|r| fresh(r.stamp, now, 0.2) && r.ok);
        let hold = record.and_then(|r| r.hold.as_ref());
        // (id, side, time) of the hold, else of a recent tap.
        let candidate = hold
            .map(|h| (h.id.clone(), h.direction, h.time))
            .or_else(|| {
                record.and_then(|r| {
                    let start = r.events.len().saturating_sub(4);
                    r.events[start..]
                        .iter()
                        .rev()
                        .find(|e| fresh(e.time, now, 0.4))
                        .map(|e| (e.id.clone(), e.direction, e.time))
                })
            });
        let valid_request = candidate
            .as_ref()
            .filter(|(id, _, _)| !id.is_empty() && id.len() <= 100);
        if let Some(current) = self.request_id.clone() {
            let continuing = if self.tapping {
                now < self.tap_until
            } else {
                valid_request.is_some_and(|(id, side, _)| {
                    *id == current && side_code(*side) == self.request_side
                })
            };
            let opposite = if self.request_side == 3 { right } else { left };
            if !healthy || !continuing || opposite || now - self.request_started >= 300. {
                self.request_id = None;
                self.request_side = 0;
                self.accepted_id = None;
            }
        }
        if let Some((id, side, time)) = valid_request {
            if !self.seen.contains(id) {
                // Consume rejected or expired requests too: recovery needs a
                // new policy edge.
                if self.seen.len() == 256 {
                    self.seen.pop_front();
                }
                self.seen.push_back(id.clone());
                if healthy
                    && self.request_id.is_none()
                    && fresh(*time, now, 0.4)
                    && !(left || right)
                {
                    self.request_id = Some(id.clone());
                    self.request_side = side_code(*side);
                    self.request_started = now;
                    self.tap_until = now + 1.6;
                    self.first_sent = None;
                    self.tapping = hold.is_none();
                }
            }
        }
        self.direction = if healthy { self.request_side } else { 0 };
        Ok(self.direction)
    }

    /// carcontroller `aprk_blinkers`, every 10 ms on a CAN-FD car.
    pub fn aprk_blinkers(
        &mut self,
        writer: &mut CanWriter,
        cs: car_state::Reader<'_>,
        frame: u32,
    ) -> Result<Vec<Frame>, Error> {
        let mut out = Vec::new();
        let direction = self.direction;
        let opposite = if direction == 3 {
            cs.get_right_blinker()
        } else {
            cs.get_left_blinker()
        };
        let drive = cs
            .get_gear_shifter()
            .unwrap_or(car_state::GearShifter::Unknown)
            == car_state::GearShifter::Drive;
        let want =
            self.enabled() && self.output_ready && matches!(direction, 3 | 4) && drive && !opposite;
        let uds = |data: [u8; 8], buses: &[u8]| {
            buses
                .iter()
                .map(|bus| Frame {
                    address: spas::APRK,
                    data: data.to_vec(),
                    bus: *bus,
                })
                .collect::<Vec<_>>()
        };
        if self.request_id != self.aprk_request_id {
            // A fresh policy request gets one handshake, including after an
            // earlier failure; a continuing one never retries the ECU.
            out.extend(uds(spas::DEFAULT, &self.aprk_buses));
            self.aprk = Aprk::Idle;
            self.aprk_buses.clear();
            self.aprk_request_id.clone_from(&self.request_id);
        }
        let side = if want { direction } else { 0 };
        let now = self.can_at;
        let give_back = |kick: &mut Self, mut out: Vec<Frame>| {
            out.extend(uds(spas::DEFAULT, &kick.aprk_buses));
            kick.aprk = if want { Aprk::Failed } else { Aprk::Idle };
            kick.aprk_buses.clear();
            out
        };
        match self.aprk {
            _ if !want => return Ok(give_back(self, out)),
            Aprk::Idle => {
                self.aprk_buses = vec![self.bus.ecan, self.bus.acan];
                out.extend(uds(spas::EXTENDED, &self.aprk_buses));
                self.aprk = Aprk::Ask;
                self.aprk_frame = frame;
                self.aprk_at = now;
            }
            Aprk::Ask => {
                if self.ack_at >= self.aprk_at && self.ack_at > 0. {
                    out.extend(uds(APRK_COM_DISABLE, &self.aprk_buses));
                    self.aprk = Aprk::WaitQuiet;
                    self.aprk_frame = frame;
                } else if frame.wrapping_sub(self.aprk_frame) >= APRK_REPLY_FRAMES {
                    if self.kick_mode == 0 {
                        // Mode 0 lights only after an acknowledgement.
                        return Ok(give_back(self, out));
                    }
                    // No answer (IONIQ 5 PE: 0 of ~180): the stock stream
                    // stays on the bus and panda sends each of our frames
                    // right behind the stock one, so the same id never
                    // starts twice at once.
                    self.aprk = Aprk::Active;
                    self.aprk_frame = frame;
                    self.aprk_at = now;
                    self.aprk_acked = false;
                }
            }
            Aprk::WaitQuiet => {
                if frame.wrapping_sub(self.aprk_frame) >= 10 && now - self.stock_at >= APRK_QUIET {
                    self.aprk = Aprk::Active;
                    self.aprk_frame = frame;
                    self.aprk_at = now;
                    self.aprk_acked = true;
                } else if frame.wrapping_sub(self.aprk_frame) >= APRK_REPLY_FRAMES {
                    return Ok(give_back(self, out));
                }
            }
            Aprk::Active => {
                if self.aprk_acked && self.stock_at > self.aprk_at {
                    // The ECU came back: stop before our frames collide.
                    return Ok(give_back(self, out));
                }
                if frame.is_multiple_of(100) {
                    out.extend(uds(spas::PRESENT, &self.aprk_buses));
                }
                // Every frame (100 Hz), twice APRK's 50 Hz.
                out.extend(canfd_maintenance::spas(
                    writer,
                    self.bus,
                    [side == 3, side == 4],
                )?);
            }
            Aprk::Failed => {}
        }
        Ok(out)
    }

    /// Whether the APRK path still has to run (or give the ECU back).
    pub fn aprk_running(&self) -> bool {
        self.enabled() || self.aprk != Aprk::Idle
    }

    /// carrot `isolate`: SPAS frames go out only after modeld acknowledged
    /// the lamp ownership, so the lamp feedback is never taken as the driver.
    pub fn isolate(
        &mut self,
        frames: Vec<Frame>,
        isolation_mask: Option<u8>,
        now: f64,
    ) -> Vec<Frame> {
        if !self.configured {
            return frames;
        }
        let directions: Vec<u8> = frames
            .iter()
            .filter(|f| f.address == 0x16a && f.data.len() == 32)
            .map(|f| f.data[16] >> 5)
            .collect();
        let direction = if directions.contains(&3) {
            1
        } else if directions.contains(&4) {
            2
        } else {
            0
        };
        if direction == 0 {
            return frames;
        }
        if self.owner.held {
            if (direction != self.journal_direction || now - self.last_journal >= 0.1)
                && record_ownership(now, &json!({"time": now, "direction": direction})).is_ok()
            {
                self.journal_direction = direction;
                self.last_journal = now;
            }
            if isolation_mask.is_some_and(|mask| mask & direction != 0) {
                return frames;
            }
        }
        frames
            .into_iter()
            .filter(|f| f.address != 0x165 && f.address != 0x16a)
            .collect()
    }

    /// carrot `feedback`: acknowledge submitted SPAS frames to the worker.
    pub fn feedback(&mut self, cs: car_state::Reader<'_>, now: f64, frames: &[Frame]) {
        let sent = self.request_id.is_some()
            && frames.iter().any(|f| {
                f.address == 0x16a
                    && f.data.len() == 32
                    && (f.data[16] >> 5) & 7 == self.request_side
            });
        if sent {
            if self.first_sent.is_none() {
                self.first_sent = Some(now);
            }
            if self.tapping && self.accepted_id != self.request_id {
                self.tap_until = now + 1.;
            }
            self.accepted_id.clone_from(&self.request_id);
            self.last_sent = now;
        } else if !fresh(self.last_sent, now, 0.2) {
            self.accepted_id = None;
        }
        if self.worker.is_some() && now >= self.feedback_at {
            let driver = cs.get_blinker_lever() != 0
                || self.driver_direction.is_some_and(|d| d != 0)
                || now - self.driver_pressed_at < 0.2;
            let accepted = if self.ready {
                self.accepted_id.clone()
            } else {
                None
            };
            self.feedback = Feedback {
                stamp: now,
                state: if driver {
                    "driver_override"
                } else if self.ready {
                    "ready"
                } else {
                    "waiting"
                },
                available: self.ready,
                first_sent: if accepted.is_some() {
                    self.first_sent
                } else {
                    None
                },
                accepted_id: accepted,
                lamps: None,
            };
            self.feedback_at = now + 0.05;
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    fn bus() -> CanBus {
        CanBus::with_offset(0, false, 1)
    }

    #[test]
    fn isolate_drops_spas_until_the_mask_acknowledges() {
        let mut kick = Kick::new(true, bus());
        let frame = spas::frame(0x16a, 1, Some(spas::Direction::Left), 0);
        let other = Frame {
            address: 0x50,
            data: vec![0; 8],
            bus: 0,
        };
        let out = kick.isolate(vec![frame.clone(), other], Some(1), 10.);
        let addresses: Vec<u32> = out.iter().map(|f| f.address).collect();
        assert_eq!(addresses, vec![0x50], "no lease: never isolated");
        let unconfigured = Kick::new(false, bus()).isolate(vec![frame.clone()], None, 10.);
        assert_eq!(unconfigured.len(), 1);
    }

    #[test]
    fn a_new_request_waits_for_the_stock_contract_and_a_neutral_lever() {
        let mut kick = Kick::new(true, bus());
        let dir = std::env::temp_dir().join(format!("kick-owner-{}", std::process::id()));
        kick.owner = Owner::with_directory("CANFD Kick", dir.clone());
        kick.kick_setting = true;
        kick.onroad = true;
        // No lease, no contract and no lever: never ready.
        assert!(!kick.update(true, 10.));
        kick.driver_direction = Some(1);
        kick.driver_stamp = 10.;
        assert!(!kick.update(true, 10.01), "lever held");
        std::fs::remove_dir_all(&dir).ok();
    }

    #[test]
    fn aprk_gives_the_ecu_back_without_a_request() {
        let mut writer = CanWriter::new(
            &std::path::Path::new(env!("CARGO_MANIFEST_DIR"))
                .join("../../../opendbc_repo/opendbc/dbc/hyundai_canfd_generated.dbc"),
        )
        .unwrap();
        let mut kick = Kick::new(true, bus());
        kick.aprk = Aprk::Active;
        kick.aprk_buses = vec![0, 1];
        let mut message = capnp::message::Builder::new_default();
        let cs = message.init_root::<car_state::Builder<'_>>();
        let out = kick
            .aprk_blinkers(&mut writer, cs.into_reader(), 0)
            .unwrap();
        assert_eq!(out.len(), 2, "default session on ECAN and ACAN");
        assert!(out
            .iter()
            .all(|f| f.address == spas::APRK && f.data[..2] == [0x02, 0x10]));
        assert_eq!(kick.aprk, Aprk::Idle);
    }
}
