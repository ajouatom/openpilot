use crate::{dbc::Dbc, state::MessageState, Diagnostic, Error, Packet};
use num_traits::ToPrimitive;
use std::{
    collections::{BTreeMap, BTreeSet},
    sync::Arc,
};

pub struct Parser {
    pub dbc: Arc<Dbc>,
    pub bus: u8,
    pub states: BTreeMap<u32, MessageState>,
    order: Vec<u32>,
    successful_order: Vec<u32>,
    pub raw: BTreeMap<u32, Vec<u8>>,
    pub seen_addresses: BTreeSet<u32>,
    pub controls_ready: bool,
    pub invalid_count: u8,
    pub last_nonempty: u64,
    pub last_update: u64,
    pub diagnostics: Vec<Diagnostic>,
}

impl Parser {
    pub fn new(dbc: Arc<Dbc>, bus: u8, _now: u64) -> Self {
        Self {
            dbc,
            bus,
            states: BTreeMap::new(),
            order: Vec::new(),
            successful_order: Vec::new(),
            raw: BTreeMap::new(),
            seen_addresses: BTreeSet::new(),
            controls_ready: false,
            invalid_count: 5,
            last_nonempty: 0,
            last_update: 0,
            diagnostics: Vec::new(),
        }
    }

    pub fn add(
        &mut self,
        name: &str,
        frequency: Option<f64>,
        ignore_counter: bool,
        now: u64,
    ) -> Result<(), Error> {
        let address = self.dbc.message(name)?.address;
        self.add_address(address, frequency, ignore_counter, now)
    }

    pub fn add_address(
        &mut self,
        address: u32,
        frequency: Option<f64>,
        ignore_counter: bool,
        now: u64,
    ) -> Result<(), Error> {
        let message = self
            .dbc
            .messages
            .get(&address)
            .ok_or_else(|| Error::Message(address.to_string()))?;
        if self.states.contains_key(&address) {
            return Err(Error::Duplicate(address));
        }
        let mut state = MessageState::new(message, frequency, now);
        state.ignore_counter = ignore_counter;
        self.states.insert(address, state);
        self.order.push(address);
        if self.successful_order.capacity() < self.states.len() {
            self.successful_order
                .reserve(self.states.len() - self.successful_order.len());
        }
        Ok(())
    }

    pub fn signal(&self, name: &str, signal: &str) -> Result<f64, Error> {
        let message = self.dbc.message(name)?;
        let index = message
            .signals
            .iter()
            .position(|s| s.name == signal)
            .ok_or_else(|| Error::Signal(signal.into()))?;
        self.states
            .get(&message.address)
            .and_then(|s| s.values.get(index))
            .copied()
            .ok_or_else(|| Error::Message(name.into()))
    }

    pub fn signal_lazy(&mut self, name: &str, signal: &str, now: u64) -> Result<f64, Error> {
        let address = self.dbc.message(name)?.address;
        if !self.states.contains_key(&address) {
            self.add_address(address, None, false, now)?;
        }
        self.signal(name, signal)
    }

    pub fn bus_timeout(&self) -> bool {
        let threshold = self
            .states
            .values()
            .filter(|s| s.timeout_threshold > 0.)
            .fold(5e8f64, |threshold, s| threshold.min(s.timeout_threshold));
        let age = i128::from(self.last_update) - i128::from(self.last_nonempty);
        age.to_f64().is_some_and(|age| age > threshold)
            && !self.states.values().all(|s| s.ignore_alive)
    }

    pub fn can_valid(&mut self) -> bool {
        let mut valid = true;
        let mut counters_valid = true;
        for address in &self.order {
            let state = self
                .states
                .get_mut(address)
                .expect("registered CAN state exists");
            if state.counter_fail >= 5 {
                counters_valid = false;
                state.log(
                    self.last_update,
                    format!(
                        "counter invalid, state.counter_fail={} MAX_BAD_COUNTER=5",
                        state.counter_fail
                    ),
                    &mut self.diagnostics,
                );
            }
            if !state.valid(self.last_update) {
                valid = false;
                state.log(
                    self.last_update,
                    "not valid (timeout or missing)".into(),
                    &mut self.diagnostics,
                );
            }
        }
        self.invalid_count = if valid {
            0
        } else {
            (self.invalid_count + 1).min(5)
        };
        self.invalid_count < 5 && counters_valid
    }

    /// Timed-out messages as `NAME 0xADDR` (carrot `invalid_can_messages`,
    /// for the canError alert text2). Counter faults are not named.
    pub fn invalid_messages(&self) -> Vec<String> {
        self.order
            .iter()
            .filter_map(|address| self.states.get(address))
            .filter(|state| !state.valid(self.last_update))
            .map(|state| format!("{} 0x{:X}", state.name, state.address))
            .collect()
    }

    pub fn successful_addresses(&self) -> &[u32] {
        &self.successful_order
    }

    pub fn update(&mut self, packets: &[Packet]) -> Result<BTreeSet<u32>, Error> {
        self.successful_order.clear();
        for state in self.states.values_mut() {
            for values in &mut state.all_values {
                values.clear();
            }
        }
        let mut updated = BTreeSet::new();
        for packet in packets {
            let mut bus_empty = true;
            for frame in &packet.frames {
                if frame.bus != self.bus {
                    continue;
                }
                if self.controls_ready {
                    self.seen_addresses.insert(frame.address);
                }
                bus_empty = false;
                let Some(state) = self.states.get_mut(&frame.address) else {
                    continue;
                };
                if frame.data.len() > 64 {
                    continue;
                }
                let message = self
                    .dbc
                    .messages
                    .get(&frame.address)
                    .ok_or_else(|| Error::Message(frame.address.to_string()))?;
                if state.parse(
                    message,
                    (packet.mono_time, &frame.data),
                    &mut self.diagnostics,
                )? {
                    if updated.insert(frame.address) {
                        self.successful_order.push(frame.address);
                    }
                    self.raw.insert(frame.address, frame.data.clone());
                }
            }
            if !bus_empty {
                self.last_nonempty = packet.mono_time;
            }
            self.last_update = packet.mono_time;
        }
        Ok(updated)
    }
}
