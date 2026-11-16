//! Opt-in Hyundai CAN-FD SPAS indicator session with acknowledged ECU handoff.
//! Port of carrot's `opendbc/car/hyundai/spas.py`.
//!
//! No vehicle-name allowlist: exact ECAN contracts and advancing stock counters
//! are required. All timing is monotonic seconds. No sockets, params or threads.
use openpilot_can::Frame;
use std::collections::BTreeMap;

pub const SPAS: [(u32, usize); 2] = [(0x165, 24), (0x16a, 32)];
pub const APRK: u32 = 0x7b1;
pub const RESPONSE: u32 = 0x7b9;
pub const EXTENDED: [u8; 8] = [0x02, 0x10, 0x03, 0, 0, 0, 0, 0];
pub const PATCH_DISABLE: [u8; 8] = [0x03, 0x28, 0x83, 0x01, 0, 0, 0, 0];
/// Communication control disable, requesting a positive acknowledgement.
pub const DISABLE: [u8; 8] = [0x03, 0x28, 0x03, 0x01, 0, 0, 0, 0];
pub const ENABLE: [u8; 8] = [0x03, 0x28, 0x00, 0x01, 0, 0, 0, 0];
pub const DEFAULT: [u8; 8] = [0x02, 0x10, 0x01, 0, 0, 0, 0, 0];
pub const PRESENT: [u8; 8] = [0x02, 0x3e, 0x80, 0, 0, 0, 0, 0];

fn size(address: u32) -> Option<usize> {
    SPAS.iter()
        .find_map(|(a, size)| (*a == address).then_some(*size))
}

fn crc16_xmodem(mut crc: u16, byte: u8) -> u16 {
    crc ^= u16::from(byte) << 8;
    for _ in 0..8 {
        crc = if crc & 0x8000 != 0 {
            (crc << 1) ^ 0x1021
        } else {
            crc << 1
        };
    }
    crc
}

pub fn checksum(address: u32, data: &[u8]) -> u16 {
    let mut crc = 0;
    for &b in data.iter().skip(2) {
        crc = crc16_xmodem(crc, b);
    }
    for b in (address as u16).to_le_bytes() {
        crc = crc16_xmodem(crc, b);
    }
    crc ^ if data.len() == 24 { 0x819d } else { 0x9f5b }
}

/// Both SPAS messages have their exact lengths, as `size` reports them.
pub fn compatible_with(size: impl Fn(u32) -> Option<usize>) -> bool {
    SPAS.iter()
        .all(|(address, expected)| size(*address) == Some(*expected))
}

/// Neither SPAS message exists: a car without APRK (HDA1). Only the parked
/// no-stock trial (CanfdBlinkerTest 19/20) can run on it; the acknowledged
/// session needs stock frames and never starts.
pub fn absent_with(size: impl Fn(u32) -> Option<usize>) -> bool {
    SPAS.iter().all(|(address, _)| size(*address).is_none())
}

/// Both SPAS messages exist on `bus` with their exact lengths.
pub fn compatible(fingerprint: &BTreeMap<u8, BTreeMap<u32, usize>>, bus: u8) -> bool {
    SPAS.iter().all(|(address, size)| {
        fingerprint
            .get(&bus)
            .and_then(|messages| messages.get(address))
            == Some(size)
    })
}

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum Direction {
    Left,
    Right,
}

pub fn frame(address: u32, counter: u8, direction: Option<Direction>, bus: u8) -> Frame {
    let mut data = vec![0; size(address).unwrap_or(0)];
    if data.len() > 16 {
        data[2] = counter;
    }
    if address == 0x16a {
        data[16] = match direction {
            None => 0,
            Some(Direction::Left) => 0x60,
            Some(Direction::Right) => 0x80,
        };
    }
    if data.len() >= 2 {
        let crc = checksum(address, &data).to_le_bytes();
        data[..2].copy_from_slice(&crc);
    }
    Frame { address, data, bus }
}

#[derive(Clone, Copy, Debug)]
struct Stock {
    stamp: f64,
    counter: i32,
    consecutive: u32,
}
impl Default for Stock {
    fn default() -> Self {
        Self {
            stamp: f64::NEG_INFINITY,
            counter: -1,
            consecutive: 0,
        }
    }
}

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum State {
    Idle,
    Failed,
    Extended,
    Disable,
    Quiet,
    Active,
    RestoreEnable,
    RestoreDefault,
    RestoreStock,
}
impl State {
    pub fn name(self) -> &'static str {
        match self {
            Self::Idle => "idle",
            Self::Failed => "failed",
            Self::Extended => "extended",
            Self::Disable => "disable",
            Self::Quiet => "quiet",
            Self::Active => "active",
            Self::RestoreEnable => "restore_enable",
            Self::RestoreDefault => "restore_default",
            Self::RestoreStock => "restore_stock",
        }
    }
    fn restoring(self) -> bool {
        matches!(
            self,
            Self::RestoreEnable | Self::RestoreDefault | Self::RestoreStock
        )
    }
}

pub struct Session {
    pub bus: u8,
    stock: [Stock; 2],
    pub state: State,
    pub reason: String,
    changed: f64,
    started: f64,
    next_send: f64,
    keepalive: f64,
    duration: f64,
    request: Option<String>,
    previous: Option<String>,
    direction: Option<Direction>,
    counter: u8,
    reply: Option<(f64, Vec<u8>)>,
    last_update: Option<f64>,
    restore_attempts: u32,
    pub restore_ack: bool,
    pub stock_resumed: bool,
    pub stock_direction: Option<u8>,
}

fn index(address: u32) -> Option<usize> {
    SPAS.iter().position(|(a, _)| *a == address)
}

pub struct Update<'a> {
    pub healthy: bool,
    pub request: Option<&'a str>,
    pub direction: Option<Direction>,
    pub duration: f64,
    pub driver_override: bool,
}

impl Session {
    pub fn new(bus: u8) -> Self {
        Self {
            bus,
            stock: [Stock::default(); 2],
            state: State::Idle,
            reason: String::new(),
            changed: 0.,
            started: 0.,
            next_send: 0.,
            keepalive: 0.,
            duration: 300.,
            request: None,
            previous: None,
            direction: None,
            counter: 0,
            reply: None,
            last_update: None,
            restore_attempts: 0,
            restore_ack: false,
            stock_resumed: false,
            stock_direction: None,
        }
    }

    pub fn observe(&mut self, address: u32, bus: u8, data: &[u8], now: f64) {
        if bus != self.bus {
            return;
        }
        if let Some(i) = index(address) {
            let valid = Some(data.len()) == size(address)
                && data.len() >= 3
                && u16::from_le_bytes([data[0], data[1]]) == checksum(address, data);
            if !valid {
                if address == 0x16a {
                    self.stock_direction = None;
                }
                self.stock[i].consecutive = 0;
                if !matches!(self.state, State::Idle | State::Failed) && !self.state.restoring() {
                    self.reason = "stock_contract".into();
                }
                return;
            }
            let st = &mut self.stock[i];
            st.consecutive = if (0. ..0.1).contains(&(now - st.stamp))
                && i32::from(data[2]) == (st.counter + 1).rem_euclid(256)
            {
                (st.consecutive + 1).min(20)
            } else {
                1
            };
            st.stamp = now;
            st.counter = i32::from(data[2]);
            if address == 0x16a {
                self.stock_direction = Some(data[16] >> 5);
            }
            if self.state == State::Active {
                self.reason = "stock_conflict".into();
            }
        } else if address == RESPONSE && data.len() == 8 {
            // Only short positive responses advance the state.
            let n = usize::from(data[0]);
            if (2..=7).contains(&n) {
                self.reply = Some((now, data[1..1 + n].to_vec()));
            }
        }
    }

    /// No stock SPAS sender: neither message valid for 1 s (HDA1 cars
    /// without APRK). Panda's `hyundai_spas_stock_absent` uses the same bound.
    pub fn stock_absent(&self, now: f64) -> bool {
        self.stock
            .iter()
            .all(|s| !(0. ..1.).contains(&(now - s.stamp)))
    }

    pub fn stock_ready(&self, now: f64) -> bool {
        self.stock
            .iter()
            .all(|s| s.consecutive >= 10 && (0. ..0.1).contains(&(now - s.stamp)))
    }

    fn acknowledged(&self, prefix: &[u8]) -> bool {
        self.reply
            .as_ref()
            .is_some_and(|(at, body)| self.changed <= *at && body.starts_with(prefix))
    }

    fn enter(&mut self, state: State, now: f64) {
        self.state = state;
        self.changed = now;
        self.reply = None;
    }

    fn uds(&self, data: [u8; 8]) -> Frame {
        Frame {
            address: APRK,
            data: data.to_vec(),
            bus: self.bus,
        }
    }

    fn restore(&mut self, now: f64, reason: &str) -> Vec<Frame> {
        self.reason = reason.into();
        self.restore_attempts = 1;
        self.restore_ack = false;
        self.stock_resumed = false;
        self.enter(State::RestoreEnable, now);
        vec![self.uds(ENABLE)]
    }

    pub fn update(&mut self, now: f64, input: Update<'_>) -> Vec<Frame> {
        let continuous = self
            .last_update
            .is_some_and(|last| (0. ..0.2).contains(&(now - last)));
        self.last_update = Some(now);
        let request = input.request.map(str::to_owned);
        let changed_request = request != self.previous;
        self.previous = request.clone();
        let allowed = input.healthy
            && continuous
            && !input.driver_override
            && input.direction.is_some()
            && request
                .as_ref()
                .is_some_and(|r| !r.is_empty() && r.len() <= 100);
        match self.state {
            State::Idle => {
                if changed_request && allowed && self.stock_ready(now) {
                    self.request = request;
                    self.direction = input.direction;
                    self.started = now;
                    self.duration = input.duration.clamp(0.1, 300.);
                    self.reason.clear();
                    self.enter(State::Extended, now);
                    return vec![self.uds(EXTENDED)];
                }
                return Vec::new();
            }
            // No automatic reacquisition after a failed restore; a held
            // request is never replayed.
            State::Failed => return Vec::new(),
            state if state.restoring() => return self.restoring(now),
            _ => {}
        }
        if !allowed
            || request != self.request
            || input.direction != self.direction
            || now - self.started >= self.duration
            || !self.reason.is_empty()
        {
            let reason = if input.driver_override {
                "driver_override".to_owned()
            } else if !self.reason.is_empty() {
                self.reason.clone()
            } else {
                "request_ended".to_owned()
            };
            return self.restore(now, &reason);
        }
        if self
            .reply
            .as_ref()
            .is_some_and(|(at, body)| *at >= self.changed && body.first() == Some(&0x7f))
        {
            return self.restore(now, "ecu_rejected");
        }
        match self.state {
            State::Extended => {
                if self.acknowledged(&[0x50, 0x03]) {
                    self.enter(State::Disable, now);
                    return vec![self.uds(DISABLE)];
                }
                if now - self.changed >= 0.4 {
                    return self.restore(now, "session_timeout");
                }
            }
            State::Disable => {
                if self.acknowledged(&[0x68, 0x03]) {
                    self.enter(State::Quiet, now);
                } else if now - self.changed >= 0.4 {
                    return self.restore(now, "disable_timeout");
                }
            }
            State::Quiet => {
                if now - self.changed >= 0.1 && self.stock.iter().all(|s| now - s.stamp >= 0.1) {
                    self.counter = ((self.stock[1].counter + 1).rem_euclid(256)) as u8;
                    self.enter(State::Active, now);
                    self.next_send = now;
                    self.keepalive = now + 1.;
                } else if now - self.changed >= 0.4 {
                    return self.restore(now, "ecu_not_quiet");
                }
            }
            _ => {}
        }
        if self.state == State::Active {
            let mut sends = Vec::new();
            if now >= self.keepalive {
                sends.push(self.uds(PRESENT));
                self.keepalive = now + 1.;
            }
            if now >= self.next_send {
                for (address, _) in SPAS {
                    sends.push(frame(address, self.counter, self.direction, self.bus));
                }
                self.counter = self.counter.wrapping_add(1);
                self.next_send = now + 0.02;
            }
            return sends;
        }
        Vec::new()
    }

    fn restoring(&mut self, now: f64) -> Vec<Frame> {
        match self.state {
            State::RestoreEnable => {
                if self.acknowledged(&[0x68, 0x00]) || now - self.changed >= 0.4 {
                    self.restore_ack = self.acknowledged(&[0x68, 0x00]);
                    self.enter(State::RestoreDefault, now);
                    return vec![self.uds(DEFAULT)];
                }
            }
            State::RestoreDefault => {
                if self.acknowledged(&[0x50, 0x01]) {
                    self.restore_ack = true;
                    self.enter(State::RestoreStock, now);
                } else if now - self.changed >= 0.4 {
                    self.enter(State::RestoreStock, now);
                }
            }
            _ => {
                if self.stock_ready(now) {
                    self.stock_resumed = true;
                    if self.restore_ack {
                        self.request = None;
                        self.direction = None;
                        self.enter(State::Idle, now);
                        return Vec::new();
                    }
                }
            }
        }
        if now - self.changed >= 1. {
            if self.restore_attempts >= 3 {
                self.enter(State::Failed, now);
                self.reason = "restore_unconfirmed".into();
            } else {
                self.restore_attempts += 1;
                self.enter(State::RestoreEnable, now);
                return vec![self.uds(ENABLE)];
            }
        }
        Vec::new()
    }

    pub fn started(&self) -> f64 {
        self.started
    }
    /// The counter that follows each stock frame's last received counter.
    pub fn next_counters(&self) -> [u8; 2] {
        self.stock.map(|s| ((s.counter + 1).rem_euclid(256)) as u8)
    }
    pub fn request(&self) -> Option<&str> {
        self.request.as_deref()
    }
    pub fn direction(&self) -> Option<Direction> {
        self.direction
    }
    #[cfg(test)]
    fn set_started(&mut self, value: f64) {
        self.started = value;
    }
}

/// Contributor wire sequence for the parked patch test; the caller owns the
/// parked guards. No response or stock-silence condition is added here.
pub struct PatchSequence {
    pub bus: u8,
    state: u8,
    started_frame: u64,
}
impl PatchSequence {
    pub fn new(bus: u8) -> Self {
        Self {
            bus,
            state: 0,
            started_frame: 0,
        }
    }
    pub fn active(&self) -> bool {
        self.state != 0
    }
    pub fn step(
        &mut self,
        controller_frame: u64,
        want: bool,
        make_spas: impl FnOnce(u64) -> Vec<Frame>,
    ) -> Vec<Frame> {
        let uds = |data: [u8; 8]| Frame {
            address: APRK,
            data: data.to_vec(),
            bus: self.bus,
        };
        if !want {
            let sends = if self.state != 0 {
                vec![uds(DEFAULT)]
            } else {
                Vec::new()
            };
            self.state = 0;
            return sends;
        }
        if self.state == 0 {
            self.state = 1;
            self.started_frame = controller_frame;
            return vec![uds(EXTENDED)];
        }
        if self.state == 1 {
            if controller_frame.saturating_sub(self.started_frame) >= 5 {
                self.state = 2;
                return vec![uds(PATCH_DISABLE)];
            }
            return Vec::new();
        }
        let mut sends = if controller_frame.is_multiple_of(100) {
            vec![uds(PRESENT)]
        } else {
            Vec::new()
        };
        if controller_frame.is_multiple_of(2) {
            sends.extend(make_spas(controller_frame));
        }
        sends
    }
}

#[cfg(test)]
mod tests;
