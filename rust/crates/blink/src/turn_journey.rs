//! TBT turn lifetime anchored to wheel-speed distance, with driver abort
//! evidence. Port of carrot's `tools/jetlink/blink_turn_journey.py`.
use super::{turn_location::Target, Side};

#[derive(Debug, Default)]
pub struct Travel {
    previous: Option<(f64, f64)>,
    distance: f64,
    epoch: u64,
}
impl Travel {
    /// `(epoch, distance)`; a new epoch starts after any gap or invalid speed.
    pub fn update(&mut self, now: f64, speed: f64, valid: bool) -> Option<(u64, f64)> {
        self.update_stopped(now, speed, valid, false)
    }

    /// carrot `Travel.update(standstill=..., gear=...)`: the filtered
    /// estimate can cross zero while the wheels confirm a D stop; keep the
    /// distance anchor through that noise, including a long red light.
    pub fn update_stopped(
        &mut self,
        now: f64,
        speed: f64,
        valid: bool,
        stopped_in_drive: bool,
    ) -> Option<(u64, f64)> {
        let speed = if stopped_in_drive && -0.1 < speed && speed < 0. {
            0.
        } else {
            speed
        };
        let old = self.previous;
        self.previous = (valid && speed.is_finite() && speed >= 0.).then_some((now, speed));
        let Some(current) = self.previous else {
            self.epoch += 1;
            return None;
        };
        if old.is_some_and(|(at, _)| !(now - at > 0. && now - at <= 0.35)) {
            self.epoch += 1;
            return None;
        }
        if let Some((at, old_speed)) = old {
            self.distance += (current.1 + old_speed) * 0.5 * (now - at);
        }
        Some((self.epoch, self.distance))
    }
}

#[derive(Clone, Debug)]
pub struct Active {
    pub direction: Side,
    start: f64,
    end: f64,
    turn: f64,
    epoch: u64,
    heading: f64,
    rotation: f64,
    pub entered: bool,
    target: Option<Target>,
    centred: Option<f64>,
    peak_angle: f64,
}

#[derive(Debug, Default)]
pub struct TurnJourney {
    cue: Option<f64>,
    mode: Option<i32>,
    pub active: Option<Active>,
    pub pending: Option<(Side, f64, Option<Target>)>,
    pub reason: &'static str,
    pub cancelled_until: f64,
}

pub struct JourneyInput<'a> {
    pub now: f64,
    pub mode: i32,
    pub valid: bool,
    pub cue: f64,
    pub direction: Option<Side>,
    pub atc_type: &'a str,
    pub travel: Option<(u64, f64)>,
    pub distance: Option<f64>,
    pub speed: f64,
    pub angle: f64,
    pub heading: Option<f64>,
    pub pressed: bool,
    pub torque: f64,
    pub target: Option<Target>,
    pub cancelled: bool,
}

impl TurnJourney {
    pub fn new() -> Self {
        Self {
            reason: "idle",
            ..Self::default()
        }
    }

    fn start(
        &mut self,
        now: f64,
        direction: Side,
        distance: f64,
        travel: (u64, f64),
        heading: f64,
        target: Option<Target>,
    ) {
        self.active = Some(Active {
            direction,
            start: now,
            end: travel.1 + distance + 30.,
            turn: travel.1 + distance,
            epoch: travel.0,
            heading,
            rotation: 0.,
            entered: false,
            target,
            centred: None,
            peak_angle: 0.,
        });
        self.reason = "approach";
    }

    pub fn update(&mut self, i: JourneyInput<'_>) -> Option<Side> {
        let previous = self.cue.replace(i.cue);
        let changed = self.mode != Some(i.mode);
        self.mode = Some(i.mode);
        let (Some(travel), Some(heading)) = (i.travel, i.heading) else {
            self.reset("inputs");
            return None;
        };
        if !i.valid
            || i.mode == 0
            || changed
            || i.cancelled
            || ![i.now, i.cue, i.speed, i.angle, heading, i.torque, travel.1]
                .iter()
                .all(|v| v.is_finite())
        {
            self.reset("inputs");
            return None;
        }
        let fresh = previous.is_some_and(|p| p != i.cue)
            && (0. ..0.35).contains(&(i.now - i.cue))
            && i.direction.is_some_and(|d| i.atc_type == d.turn())
            && i.distance
                .is_some_and(|d| d.is_finite() && (0. ..=250.).contains(&d))
            && i.now >= self.cancelled_until;
        if fresh {
            let direction = i.direction.expect("checked");
            let distance = i.distance.expect("checked");
            if let Some(a) = &self.active {
                let next_turn = travel.1 + distance;
                let same_target = a.target.is_some() && i.target.is_some() && a.target == i.target;
                // Repeated audio for the same approach must not push the end away.
                if !same_target && next_turn > a.turn + 10. {
                    self.pending = Some((direction, next_turn, i.target.clone()));
                }
            } else {
                self.start(
                    i.now,
                    direction,
                    distance,
                    travel,
                    heading,
                    i.target.clone(),
                );
            }
        }
        let Some(a) = &mut self.active else {
            return None;
        };
        if a.epoch != travel.0
            || i.now - a.start >= 300.
            || (a.target.is_some()
                && i.target.is_some()
                && a.target.as_ref().map(|t| &t.2) != i.target.as_ref().map(|t| &t.2))
        {
            self.reset("route_or_distance");
            return None;
        }
        let sign = a.direction.sign();
        let turn = heading - a.heading;
        let delta = turn.sin().atan2(turn.cos()) * -sign;
        a.heading = heading;
        if i.speed >= 1. / 3.6 {
            a.rotation += delta;
        }
        a.peak_angle = a.peak_angle.max(i.angle * sign);
        a.entered |= i.angle * sign >= 30. && a.rotation >= 20f64.to_radians();
        // Steering through centre into the opposite direction abandons a turn.
        let opposite = a.peak_angle >= 30.
            && i.pressed
            && i.torque * sign < 0.
            && i.angle * sign < 0.
            && i.speed >= 1. / 3.6;
        if opposite {
            self.reset("driver_opposite");
            self.cancelled_until = i.now + 3.;
            return None;
        }
        let centred = i.angle.abs() <= 10. && i.speed >= 1. / 3.6;
        a.centred = if centred {
            Some(a.centred.unwrap_or(i.now))
        } else {
            None
        };
        let complete = a.centred.is_some_and(|since| i.now - since >= 0.4);
        // Only a turn with measured rotation and selected-side steering ends
        // on wheel return; approach waits must not end at a straight wheel.
        let returned =
            a.entered && a.peak_angle > 30. && i.angle.abs() <= 15. && i.speed >= 1. / 3.6;
        let expired = travel.1 >= a.end && (complete || travel.1 >= a.end + 30.);
        if returned || expired {
            self.active = None;
            self.reason = "completed";
            if let Some((next_direction, next_turn, next_target)) = self.pending.take() {
                if next_turn - travel.1 > -30. && next_turn - travel.1 <= 250. {
                    self.start(
                        i.now,
                        next_direction,
                        next_turn - travel.1,
                        travel,
                        heading,
                        next_target,
                    );
                }
            }
        }
        if let Some(a) = &self.active {
            self.reason = if a.entered { "turning" } else { "approach" };
            return Some(a.direction);
        }
        None
    }

    fn reset(&mut self, reason: &'static str) {
        self.active = None;
        self.pending = None;
        self.reason = reason;
    }
}
