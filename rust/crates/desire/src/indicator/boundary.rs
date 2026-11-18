//! Driver-initiated boundary approach indication, with lamp-feedback
//! isolation. Observes a manual maneuver; never supplies a lane-change
//! desire. Port of carrot's `desire_lib/boundary_indicator.py`.
use super::lane::{adjacent_lane, finite};
use crate::types::{Car, Model};

/// Brief confidence loss during a continuous driver crossing.
pub const TRACK_SEC: f64 = 1.0;
/// Earliest new request; extinction is required independently.
pub const QUIET_SEC: f64 = 3.0;
/// Keep delayed lamp feedback out of automatic steering.
pub const ISOLATE_SEC: f64 = 12.0;

pub type Admission = fn(&Car, &Model, u8, f64) -> bool;

#[derive(Clone, Debug)]
pub struct Episode {
    pub direction: u8,
    start: f64,
    intent: f64,
    initial: f64,
    minimum: f64,
    approach: f64,
    /// When the boundary began moving back (carrot f89f40a37).
    retreat_since: Option<f64>,
}

#[derive(Clone, Debug)]
pub struct BoundaryIndicator {
    lane_admission: Admission,
    minimum_speed: f64,
    /// `(time, direction, mask)` of the restored ownership journal.
    journal: Option<(f64, i64, Option<i64>)>,
    pub initialized: bool,
    previous: Option<(u32, f64, [f64; 2])>,
    confident: [Option<f64>; 2],
    rates: [f64; 2],
    intent: u8,
    intent_since: f64,
    pub episode: Option<Episode>,
    lane_since: [Option<f64>; 2],
    pub relabel_until: f64,
    pub blocked: u8,
    pub block_until: f64,
    isolate_until: f64,
    pub dark_since: Option<f64>,
    pub request_direction: u8,
    pub request_time: f64,
}

impl Default for BoundaryIndicator {
    fn default() -> Self {
        Self::new(None, adjacent_lane, 30. / 3.6)
    }
}

impl BoundaryIndicator {
    pub fn new(
        journal: Option<(f64, i64, Option<i64>)>,
        lane_admission: Admission,
        minimum_speed: f64,
    ) -> Self {
        Self {
            lane_admission,
            minimum_speed,
            journal,
            initialized: false,
            previous: None,
            confident: [None, None],
            rates: [0., 0.],
            intent: 0,
            intent_since: 0.,
            episode: None,
            lane_since: [None, None],
            relabel_until: 0.,
            blocked: 0,
            block_until: 0.,
            isolate_until: 0.,
            dark_since: None,
            request_direction: 0,
            request_time: 0.,
        }
    }

    pub fn reset_geometry(&mut self) {
        self.previous = None;
        self.confident = [None, None];
        self.rates = [0., 0.];
        self.intent = 0;
        self.episode = None;
        self.lane_since = [None, None];
    }

    pub fn own(&mut self, now: f64, direction: u8) {
        self.blocked |= direction;
        self.block_until = self.block_until.max(now + QUIET_SEC);
        self.isolate_until = self.isolate_until.max(now + ISOLATE_SEC);
        self.dark_since = None;
    }

    pub fn filter_lamp(&self, direction: u8) -> u8 {
        if direction & self.blocked != 0 {
            0
        } else {
            direction
        }
    }

    pub fn update(
        &mut self,
        now: f64,
        car: &Car,
        model: &Model,
        enabled: bool,
        valid: bool,
        busy: bool,
    ) {
        let admission = self.lane_admission;
        self.update_with(now, car, model, enabled, valid, busy, &admission);
    }

    /// `update` with a caller-supplied lane admission (Tesla lane signals).
    #[allow(clippy::too_many_arguments)]
    pub fn update_with(
        &mut self,
        now: f64,
        car: &Car,
        model: &Model,
        enabled: bool,
        valid: bool,
        busy: bool,
        admission: &dyn Fn(&Car, &Model, u8, f64) -> bool,
    ) {
        let lamps = u8::from(car.left_blinker) | (u8::from(car.right_blinker) << 1);
        if !self.initialized {
            self.initialized = true;
            match self.journal {
                Some((stamp, direction, mask))
                    if stamp.is_finite()
                        && (0. ..20.).contains(&(now - stamp))
                        && matches!(direction, 1 | 2) =>
                {
                    let direction = direction as u8;
                    self.blocked = match mask {
                        Some(m @ 1..=3) if (m as u8) & direction != 0 => m as u8,
                        _ => direction,
                    };
                    self.block_until = (now + 1.).max(stamp + QUIET_SEC);
                    self.isolate_until = (now + 1.).max(stamp + ISOLATE_SEC);
                }
                _ if enabled => {
                    // Suppress only new detections at startup. Without an owned
                    // journal, ordinary lamps keep their original meaning.
                    self.relabel_until = now + 1.;
                }
                _ => {}
            }
        }
        if lamps != 0 || !valid {
            self.dark_since = None;
        } else if self.dark_since.is_none() {
            self.dark_since = Some(now);
        }
        let dark = self.dark_since.is_some_and(|since| now - since >= 1.);
        if self.blocked != 0 && now >= self.isolate_until && dark {
            self.blocked = 0;
        }
        if !enabled || !valid || busy {
            self.request_direction = 0;
            self.reset_geometry();
            return;
        }
        if lamps != 0
            || car.blinker_lever != 0
            || !car.can_valid
            || !car.drive
            || car.trailer_connected
            || car.steer_fault_temporary
            || car.steer_fault_permanent
            || !car.v_ego.is_finite()
            || !car.steering_torque.is_finite()
            || car.v_ego < self.minimum_speed
        {
            self.request_direction = 0;
            self.reset_geometry();
            return;
        }
        if now < self.block_until || (self.blocked != 0 && !dark) {
            self.reset_geometry();
            // Preserve the same fresh edge after torque release, never replay a timer.
            if !(0. ..0.2).contains(&(now - self.request_time))
                || !admission(car, model, self.request_direction, now)
            {
                self.request_direction = 0;
            }
            return;
        }
        self.request_direction = 0;
        if self.geometry(now, car, model, admission).is_none() {
            self.reset_geometry();
        }
    }

    /// `None` is carrot's exception path, which resets geometry.
    fn geometry(
        &mut self,
        now: f64,
        car: &Car,
        model: &Model,
        admission: &dyn Fn(&Car, &Model, u8, f64) -> bool,
    ) -> Option<()> {
        let left = *model.lane_lines[1].first()?;
        let right = *model.lane_lines[2].first()?;
        let probs = [model.lane_line_probs[1], model.lane_line_probs[2]];
        if !finite(&[left, right, probs[0], probs[1], car.steering_torque])
            || !(left < 0. && 0. < right)
            || !(2.5 < right - left && right - left < 4.8)
        {
            self.reset_geometry();
            return Some(());
        }
        for (i, probability) in probs.iter().enumerate() {
            if *probability >= 0.10 {
                self.confident[i] = Some(now);
            }
        }
        let y = [-left, right];
        let Some((frame, stamp, prev_y)) = self.previous.replace((model.frame_id, now, y)) else {
            return Some(());
        };
        let dt = now - stamp;
        if model.frame_id <= frame || !(0.02..=0.15).contains(&dt) {
            self.reset_geometry();
            return Some(());
        }
        if (0..2).map(|i| (y[i] - prev_y[i]).abs()).fold(0., f64::max) > 0.4 {
            // During an established driver approach both boundaries can advance
            // together as markings reappear beyond an intersection.
            let paired_approach = self.episode.as_ref().is_some_and(|episode| {
                let side = usize::from(episode.direction - 1);
                now - episode.intent <= 0.5
                    && admission(car, model, episode.direction, now)
                    && 0. < prev_y[side] - y[side]
                    && prev_y[side] - y[side] <= 0.8
                    && ((y[0] + y[1]) - (prev_y[0] + prev_y[1])).abs() < 0.2
            });
            if !paired_approach {
                self.relabel_until = now + 2.;
                self.reset_geometry();
                return Some(());
            }
        }
        for i in 0..2 {
            self.rates[i] = 0.5 * self.rates[i] + 0.5 * (prev_y[i] - y[i]) / dt;
        }
        for i in 0..2 {
            self.lane_since[i] = if admission(car, model, i as u8 + 1, now) {
                Some(self.lane_since[i].unwrap_or(now))
            } else {
                None
            };
        }
        let direction = if car.steering_pressed {
            if car.steering_torque > 0. {
                1
            } else if car.steering_torque < 0. {
                2
            } else {
                0
            }
        } else {
            0
        };
        if direction != self.intent {
            self.intent = direction;
            self.intent_since = now;
        }
        let confirmed = if direction != 0 && now - self.intent_since >= 0.15 {
            direction
        } else {
            0
        };
        if now < self.relabel_until {
            return Some(());
        }
        if self.episode.is_none() && confirmed != 0 {
            let c = usize::from(confirmed - 1);
            if self.lane_since[c].is_some_and(|since| now - since >= 0.2)
                && self.confident[c].is_some_and(|seen| (0. ..=TRACK_SEC).contains(&(now - seen)))
            {
                self.episode = Some(Episode {
                    direction: confirmed,
                    start: now,
                    intent: now,
                    initial: y[c],
                    minimum: y[c],
                    approach: now,
                    retreat_since: None,
                });
            }
        }
        let Some(episode) = &mut self.episode else {
            return Some(());
        };
        let direction = episode.direction;
        let i = usize::from(direction - 1);
        if confirmed == direction {
            episode.intent = now;
        }
        if self.rates[i] > 0.1 {
            episode.approach = now;
        }
        episode.minimum = episode.minimum.min(y[i]);
        let retreating = y[i] - episode.minimum > 0.15;
        episode.retreat_since = if retreating {
            Some(episode.retreat_since.unwrap_or(now))
        } else {
            None
        };
        // A brief release or countersteer does not erase continued approach;
        // retreat held for 0.15 s, stalled approach, stale intent or a lost
        // lane does.
        if self.lane_since[i].is_none()
            || now - episode.intent > 1.5
            || now - episode.start > 4.
            || now - episode.approach > 0.3
            || episode
                .retreat_since
                .is_some_and(|since| now - since >= 0.15)
        {
            self.episode = None;
            self.relabel_until = now + 0.2;
            return Some(());
        }
        if retreating {
            return Some(());
        }
        if self.confident[i].is_none() {
            // Admission anchored confidence; geometry bounds the lifetime.
            return Some(());
        }
        // Both boundaries must move consistently; widening forks are not crossings.
        let threshold = 1.2f64.min(0.9 + 0.3 * self.rates[i].max(0.));
        if episode.initial - y[i] >= 0.15
            && y[i] < threshold
            && self.rates[i] > 0.4
            && self.rates[1 - i] < -0.4
            && (self.rates[0] + self.rates[1]).abs() < 0.35
        {
            self.request_direction = direction;
            self.request_time = now;
            self.own(now, direction);
        }
        Some(())
    }
}
