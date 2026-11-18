use super::{config::CarConfig, flags as f, wire::Values, Error};
use openpilot_can::{dbc::Dbc, parser::Parser, Packet};
use std::{collections::BTreeMap, path::Path, sync::Arc};

#[derive(Clone, Copy, PartialEq, Eq)]
pub enum Channel {
    Pt,
    Cam,
    Alt,
}

pub struct Inputs {
    pub diagnostics: super::diagnostics::Diagnostics,
    pub pt: Parser,
    pub cam: Parser,
    pub alt: Option<Parser>,
    /// Comma Pedal sensor parser (`Bus.party`), only on configured cars.
    pub pedal: Option<Parser>,
    pub captures: BTreeMap<&'static str, (Channel, &'static str)>,
    /// Per message: the frame timestamp it was built from and its signal map.
    pub(crate) values_cache:
        std::cell::RefCell<std::collections::HashMap<(u8, u32), (u64, Values)>>,
}

impl Inputs {
    pub fn new(path: &Path, config: &CarConfig, now: u64) -> Result<Self, Error> {
        let dbc = Arc::new(Dbc::load(path)?);
        let fd = config.flags & f::CANFD != 0;
        let mut pt = Parser::new(Arc::clone(&dbc), if fd { config.bus.ecan } else { 0 }, now);
        let mut alt = if fd {
            Some(Parser::new(Arc::clone(&dbc), config.bus.acan, now))
        } else {
            None
        };
        if fd {
            if super::vehicle_gps::supported(&dbc) {
                pt.add(super::vehicle_gps::MESSAGE, Some(f64::NAN), false, now)?;
            }
            let names = if config.candidate == "KIA_PV5" {
                &["CANFD_HDA_INFO_364", "CANFD_NAVI_PROFILE_093"][..]
            } else if config.candidate == "KIA_EV4" {
                &[
                    "CANFD_HDA_INFO_364",
                    "CANFD_NAVI_SEGMENT_099",
                    "CANFD_NAVI_PROFILE_093",
                ][..]
            } else {
                &["NEW_MSG_4B4", "NEW_MSG_4B9", "NEW_MSG_4BE"][..]
            };
            for name in names {
                pt.add(name, Some(f64::NAN), false, now)?;
            }
            if config.candidate == "KIA_PV5" {
                if let Some(parser) = &mut alt {
                    parser.add("CANFD_NAVI_STATUS_380", Some(f64::NAN), false, now)?;
                }
            }
            if config.flags & f::ALT_BUTTONS == 0 {
                pt.add("CRUISE_BUTTONS", Some(50.), false, now)?;
            }
            if config.ext_flags & f::ext::EV_MODE_230 != 0 {
                pt.add("HCU_STATUS_230", Some(f64::NAN), true, now)?;
            }
        } else {
            pt.add("EMS21", Some(f64::NAN), false, now)?;
        }
        let pedal = match super::pedal::configured(&config.candidate, config.flags) {
            Some(profile) => {
                // Register the configured OEM CC input at its actual receive rate.
                pt.add(profile.cruise_ready.0, Some(10.), false, now)?;
                let dbc = Arc::new(Dbc::load(&path.with_file_name("pedal_interceptor.dbc"))?);
                let mut parser = Parser::new(dbc, super::pedal::BUS, now);
                parser.add("GAS_SENSOR", Some(super::pedal::RX_FREQUENCY), false, now)?;
                Some(parser)
            }
            None => None,
        };
        Ok(Self {
            diagnostics: super::diagnostics::Diagnostics::default(),
            pt,
            cam: Parser::new(dbc, if fd { config.bus.cam } else { 2 }, now),
            alt,
            pedal,
            captures: BTreeMap::new(),
            values_cache: Default::default(),
        })
    }

    pub fn parser(&self, channel: Channel) -> Result<&Parser, Error> {
        match channel {
            Channel::Pt => Ok(&self.pt),
            Channel::Cam => Ok(&self.cam),
            Channel::Alt => self
                .alt
                .as_ref()
                .ok_or_else(|| Error::Signal("alt CAN parser".into())),
        }
    }

    pub fn parser_mut(&mut self, channel: Channel) -> Result<&mut Parser, Error> {
        match channel {
            Channel::Pt => Ok(&mut self.pt),
            Channel::Cam => Ok(&mut self.cam),
            Channel::Alt => self
                .alt
                .as_mut()
                .ok_or_else(|| Error::Signal("alt CAN parser".into())),
        }
    }

    pub fn update(&mut self, packets: &[Packet]) -> Result<(), Error> {
        self.diagnostics.seen((&self.pt, Channel::Pt), packets)?;
        self.pt.update(packets)?;
        self.diagnostics.retain_warnings(&mut self.pt);
        self.diagnostics.seen((&self.cam, Channel::Cam), packets)?;
        self.cam.update(packets)?;
        self.diagnostics.retain_warnings(&mut self.cam);
        if let Some(alt) = &mut self.alt {
            self.diagnostics.seen((alt, Channel::Alt), packets)?;
            alt.update(packets)?;
            self.diagnostics.retain_warnings(alt);
        }
        if let Some(pedal) = &mut self.pedal {
            pedal.update(packets)?;
        }
        Ok(())
    }

    pub fn signal(
        &mut self,
        channel: Channel,
        name: &str,
        signal: &str,
        now: u64,
    ) -> Result<f64, Error> {
        Ok(self.parser_mut(channel)?.signal_lazy(name, signal, now)?)
    }

    pub fn all(
        &mut self,
        channel: Channel,
        name: &str,
        signal: &str,
        now: u64,
    ) -> Result<Vec<f64>, Error> {
        let parser = self.parser_mut(channel)?;
        parser.signal_lazy(name, signal, now)?;
        let message = parser.dbc.message(name)?;
        let index = message
            .signals
            .iter()
            .position(|s| s.name == signal)
            .ok_or_else(|| Error::Signal(signal.into()))?;
        parser
            .states
            .get(&message.address)
            .and_then(|state| state.all_values.get(index))
            .cloned()
            .ok_or(Error::Numeric)
    }

    /// Built once per received frame; later calls in the same cycle share the keys.
    pub fn values(&self, channel: Channel, name: &str) -> Result<Values, Error> {
        let parser = self.parser(channel)?;
        let message = parser.dbc.message(name)?;
        let state = parser
            .states
            .get(&message.address)
            .ok_or_else(|| Error::Signal(name.into()))?;
        let stamp = state.timestamps.back().copied().unwrap_or(0);
        let key = (channel as u8, message.address);
        let mut cache = self.values_cache.borrow_mut();
        if let Some((cached, values)) = cache.get_mut(&key) {
            if *cached == stamp {
                return Ok(values.clone());
            }
            // A new frame of the same message: refresh values, keep the shared names.
            for (signal, value) in message.signals.iter().zip(&state.values) {
                if let Some(slot) = values.get_mut(signal.name.as_str()) {
                    *slot = *value;
                }
            }
            *cached = stamp;
            return Ok(values.clone());
        }
        let values: Values = message
            .signals
            .iter()
            .zip(&state.values)
            .map(|(signal, value)| (std::sync::Arc::from(signal.name.as_str()), *value))
            .collect();
        cache.insert(key, (stamp, values.clone()));
        Ok(values)
    }

    /// Receive time (ns) of the frame a capture was last built from; 0 when
    /// unknown (carrot `lane_line_time`: all four lane fields share it).
    pub fn captured_stamp(&self, key: &str) -> u64 {
        let Some((channel, name)) = self.captures.get(key) else {
            return 0;
        };
        let Ok(parser) = self.parser(*channel) else {
            return 0;
        };
        parser
            .dbc
            .names
            .get(*name)
            .and_then(|address| parser.states.get(address))
            .and_then(|state| state.timestamps.back().copied())
            .unwrap_or(0)
    }

    /// carrot `rear_gap._fresh`: no bus timeout, a frame of the expected size
    /// received within `max_age` ns of `now`.
    pub fn message_fresh(
        &self,
        channel: Channel,
        name: &str,
        size: usize,
        now: u64,
        max_age: u64,
    ) -> bool {
        let Ok(parser) = self.parser(channel) else {
            return false;
        };
        let Some(&address) = parser.dbc.names.get(name) else {
            return false;
        };
        let stamp = parser
            .states
            .get(&address)
            .and_then(|state| state.timestamps.back().copied())
            .unwrap_or(0);
        !parser.bus_timeout()
            && stamp > 0
            && now >= stamp
            && now - stamp <= max_age
            && parser
                .raw
                .get(&address)
                .is_some_and(|data| data.len() == size)
    }

    pub fn captured(&self, key: &str) -> Result<Option<Values>, Error> {
        self.captures
            .get(key)
            .map(|(channel, name)| self.values(*channel, name))
            .transpose()
    }

    pub fn capture(
        &mut self,
        input: (Channel, &'static str, &'static str),
        ignore_counter: bool,
        now: u64,
    ) -> Result<bool, Error> {
        let (channel, name, key) = input;
        let parser = self.parser_mut(channel)?;
        let Some(address) = parser.dbc.names.get(name).copied() else {
            self.diagnostics.prints.push(format!("{name} not in DBC"));
            return Ok(false);
        };
        if parser.seen_addresses.contains(&address) && !parser.states.contains_key(&address) {
            parser.add(name, None, ignore_counter, now)?;
        }
        if parser.states.contains_key(&address) {
            self.captures.insert(key, (channel, name));
            Ok(true)
        } else {
            Ok(false)
        }
    }

    pub fn validity(&mut self) -> [bool; 2] {
        let mut valid = self.pt.can_valid();
        self.diagnostics.retain_warnings(&mut self.pt);
        if valid {
            valid = self.cam.can_valid();
            self.diagnostics.retain_warnings(&mut self.cam);
        }
        if let Some(alt) = &mut self.alt {
            if valid {
                valid = alt.can_valid();
                self.diagnostics.retain_warnings(alt);
            }
        }
        let timeout = self.pt.bus_timeout()
            || self.cam.bus_timeout()
            || self.alt.as_ref().is_some_and(Parser::bus_timeout);
        [valid, timeout]
    }
}
