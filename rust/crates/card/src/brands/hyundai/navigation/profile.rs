use super::{Event, Input, Navigation};
use crate::brands::hyundai::{wire::Values, Error};
use num_traits::ToPrimitive;

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum EventType {
    Camera,
    Bump,
    SpeedZone,
}

#[derive(Clone, Copy)]
pub struct Profile {
    pub value: u32,
    pub offset: u32,
    pub counter: u32,
    pub update: u32,
    pub path: u8,
    pub profile_type: u8,
}

fn integer(data: &Values, name: &str, default: u32) -> Result<u32, Error> {
    data.get(name)
        .map(|v| v.to_u32().ok_or(Error::Numeric))
        .unwrap_or(Ok(default))
}

impl Profile {
    pub fn decode(data: &Values) -> Result<Self, Error> {
        Ok(Self {
            value: integer(data, "PROLONG_VALUE", u32::MAX)?,
            offset: integer(data, "PROLONG_OFFSET", 8191)?,
            counter: integer(data, "PROLONG_CYCLIC_COUNTER", 0)?,
            update: integer(data, "PROLONG_UPDATE", 0)?,
            path: u8::try_from(integer(data, "PROLONG_PATH_INDEX", 0)?)
                .map_err(|_| Error::Numeric)?,
            profile_type: u8::try_from(integer(data, "PROLONG_PROFILE_TYPE", 31)?)
                .map_err(|_| Error::Numeric)?,
        })
    }

    pub fn classify(self) -> Option<(EventType, f64, u8)> {
        if self.profile_type != 16 {
            return None;
        }
        let kind = u8::try_from(self.value & 15).ok()?;
        let speed = self.value >> 4;
        if self.value > 0
            && self.value <= 511
            && kind == 7
            && self.offset == 0
            && speed > 1
            && speed <= 31
        {
            return Some((EventType::SpeedZone, f64::from((speed - 1) * 5), kind));
        }
        if self.offset == 0 || self.offset > 2500 {
            return None;
        }
        if self.value == 6 {
            return Some((EventType::Bump, 0., 6));
        }
        if self.value == 0
            || self.value > 511
            || ![0, 1, 2].contains(&kind)
            || speed <= 1
            || speed > 31
        {
            return None;
        }
        Some((EventType::Camera, f64::from((speed - 1) * 5), kind))
    }
}

pub struct Segment {
    pub route: u8,
    pub path: u8,
    pub road: u8,
}

impl Segment {
    pub fn decode(data: &Values) -> Result<Self, Error> {
        let mut raw = 0u64;
        for index in 0..8 {
            let byte = data
                .get(format!("BYTE_{}", index + 1).as_str())
                .copied()
                .unwrap_or(0.)
                .to_u64()
                .ok_or(Error::Numeric)?;
            raw = raw.wrapping_add(byte << (index * 8));
        }
        Ok(Self {
            route: u8::try_from((raw >> 22) & 3).map_err(|_| Error::Numeric)?,
            path: u8::try_from((raw >> 13) & 63).map_err(|_| Error::Numeric)?,
            road: u8::try_from((raw >> 24) & 7).map_err(|_| Error::Numeric)?,
        })
    }
}

impl Navigation {
    pub fn profile_allowed(&self, kind: EventType, profile: Profile) -> bool {
        if self.mode == 0 {
            return false;
        }
        let route = self.mode == 3 || (self.mode == 2 && kind == EventType::Bump);
        !route || (self.route_state == 1 && self.route_path == Some(profile.path))
    }

    pub fn add_event(&mut self, kind: EventType, speed: f64, subtype: u8, offset: u32) {
        let target = self.total_distance + f64::from(offset);
        if let Some(event) = self.events.iter_mut().find(|event| {
            event.kind == kind
                && event.speed == speed
                && event.subtype == subtype
                && (event.target - target).abs() < 20.
        }) {
            event.target = target;
            return;
        }
        self.events.push(Event {
            cancelled: false,
            id: self.next_event,
            kind,
            speed,
            subtype,
            target,
        });
        self.next_event += 1;
        self.events.sort_by(|a, b| a.target.total_cmp(&b.target));
        self.events.truncate(32);
    }

    pub fn update_route(&mut self, input: &Input<'_>) -> Result<(), Error> {
        if let Some(data) = input.segment {
            // EV4 wraps padding after sparse route updates. Padding supplies no
            // new route evidence and must not renew the existing route timeout.
            let padding = self.ev4
                && (1..=8).all(|index| {
                    data.get(format!("BYTE_{index}").as_str())
                        .copied()
                        .unwrap_or(0.)
                        == 255.
                });
            if input.segment_timestamp > self.segment_timestamp && !padding {
                self.segment_timestamp = input.segment_timestamp;
                let segment = Segment::decode(data)?;
                let previous = (self.route_state, self.route_path);
                self.route_state = segment.route;
                self.route_path = if [0, 1].contains(&segment.route) {
                    Some(segment.path)
                } else {
                    None
                };
                if segment.road != 7 {
                    self.road_class = segment.road;
                }
                if segment.route == 2 {
                    self.route_reset_timestamp = input.segment_timestamp;
                    self.clear_events();
                    self.clear_speed_zone();
                    self.clear_school();
                } else if self.mode >= 2
                    && (segment.route != 1 || (previous.0 == 1 && previous.1 != Some(segment.path)))
                {
                    self.clear_route_filtered();
                }
            }
        }
        let age = i128::from(input.last_update) - i128::from(self.segment_timestamp);
        if self.mode >= 2 && self.route_state == 1 && !(0..=2_000_000_000).contains(&age) {
            self.route_state = 0;
            self.route_path = None;
            self.clear_route_filtered();
        }
        Ok(())
    }
}
