//! Driver turn observations used only by the physical-indicator path.
//! Port of carrot's `desire_lib/manual_turn_indicator.py`.
use super::lane::{finite, MANUAL_INDICATOR_MIN_SPEED};
use crate::types::Car;

#[derive(Clone, Debug)]
pub struct ManualTurnMovement {
    pub direction: u8,
    started: f64,
    previous: f64,
    settled: Option<f64>,
    pub reason: &'static str,
}

impl ManualTurnMovement {
    pub fn new(now: f64, direction: u8) -> Self {
        Self {
            direction,
            started: now,
            previous: now,
            settled: None,
            reason: "active",
        }
    }

    pub fn update(&mut self, now: f64, car: &Car, yaw_rate: Option<f64>) -> bool {
        let Some(yaw_rate) = yaw_rate.filter(|y| y.is_finite()) else {
            self.reason = "inputs";
            return false;
        };
        if !car.steering_angle_deg.is_finite() {
            self.reason = "inputs";
            return false;
        }
        let elapsed = now - self.previous;
        self.previous = now;
        if !(elapsed > 0. && elapsed <= 0.15) || now - self.started >= 30. {
            self.reason = "timeout";
            return false;
        }
        let sign = if self.direction == 1 { 1. } else { -1. };
        if car.steering_pressed
            && car.steering_torque * sign < 0.
            && car.steering_angle_deg * sign < 0.
        {
            self.reason = "driver_opposite";
            return false;
        }
        let returned = car.steering_angle_deg.abs() <= 15.
            && yaw_rate.abs() <= 3f64.to_radians()
            && car.v_ego >= 1. / 3.6;
        self.settled = if returned {
            Some(self.settled.unwrap_or(now))
        } else {
            None
        };
        if self.settled.is_some_and(|since| now - since >= 0.3) {
            self.reason = "completed";
            return false;
        }
        true
    }
}

#[derive(Clone, Debug)]
pub struct ManualTurnIndicator {
    previous: Option<(f64, u32, f64)>,
    intent: u8,
    intent_since: f64,
    last_pressed: f64,
    pressed_since: Option<f64>,
    intent_confirmed: bool,
    initial_angle: f64,
    rotation: f64,
    confirmed_since: Option<f64>,
    pub candidate: Option<(u8, &'static str)>,
    pub reason: &'static str,
    /// The last 0.25 s of steering angles (carrot 171f80bf7).
    recent_angles: std::collections::VecDeque<(f64, f64)>,
}

impl Default for ManualTurnIndicator {
    fn default() -> Self {
        Self {
            previous: None,
            intent: 0,
            intent_since: 0.,
            last_pressed: 0.,
            pressed_since: None,
            intent_confirmed: false,
            initial_angle: 0.,
            rotation: 0.,
            confirmed_since: None,
            candidate: None,
            reason: "idle",
            recent_angles: std::collections::VecDeque::with_capacity(8),
        }
    }
}

impl ManualTurnIndicator {
    pub fn reset(&mut self, reason: &'static str) {
        self.previous = None;
        self.intent = 0;
        self.pressed_since = None;
        self.intent_confirmed = false;
        self.rotation = 0.;
        self.confirmed_since = None;
        self.candidate = None;
        self.reason = reason;
        self.recent_angles.clear();
    }

    /// carrot `update(now, cs, frame, yaw_rate, *, allowed, following_curve,
    /// emit_allowed, require_growth)`: the keyword arguments stay positional here.
    #[allow(clippy::too_many_arguments)]
    pub fn update(
        &mut self,
        now: f64,
        car: &Car,
        frame: u32,
        yaw_rate: Option<f64>,
        allowed: bool,
        following_curve: bool,
        // carrot 171f80bf7: a follow-up observer may track a turn before it is
        // allowed to emit, and may require the angle still to be growing.
        emit_allowed: bool,
        require_growth: bool,
    ) -> Option<ManualTurnMovement> {
        self.candidate = None;
        let Some(yaw_rate) = yaw_rate.filter(|_| allowed) else {
            self.reset("admission");
            return None;
        };
        if !finite(&[
            now,
            yaw_rate,
            car.v_ego,
            car.steering_angle_deg,
            car.steering_torque,
        ]) || !(MANUAL_INDICATOR_MIN_SPEED..=60. / 3.6).contains(&car.v_ego)
        {
            self.reset("admission");
            return None;
        }
        let previous = self
            .previous
            .replace((now, frame, car.steering_angle_deg))?;
        let dt = now - previous.0;
        if !(0.02..=0.15).contains(&dt)
            || frame <= previous.1
            || (car.steering_angle_deg - previous.2).abs() > 100.
        {
            self.reset("inputs");
            return None;
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
        if direction != 0 && direction != self.intent {
            self.intent = direction;
            self.intent_since = now;
            self.initial_angle = previous.2;
            self.rotation = 0.;
            self.confirmed_since = None;
            self.pressed_since = Some(now);
            self.intent_confirmed = false;
        }
        if direction != 0 {
            self.last_pressed = now;
            let since = *self.pressed_since.get_or_insert(now);
            self.intent_confirmed |= now - since >= 0.15;
        } else {
            self.pressed_since = None;
        }
        if self.intent == 0 || now - self.last_pressed > 0.8 || now - self.intent_since > 4. {
            self.reset("driver");
            return None;
        }
        let sign = if self.intent == 1 { 1. } else { -1. };
        let angle = car.steering_angle_deg;
        self.recent_angles.push_back((now, angle));
        while self
            .recent_angles
            .front()
            .is_some_and(|(stamp, _)| now - stamp > 0.25)
        {
            self.recent_angles.pop_front();
        }
        if !emit_allowed {
            // Bridge only the last quarter-second of the rearm boundary; reusing
            // a whole completed sweep could indicate a later alignment motion
            // (carrot 171f80bf7).
            let lowest = self
                .recent_angles
                .iter()
                .map(|(_, a)| a * sign)
                .fold(f64::INFINITY, f64::min);
            self.initial_angle = sign * (self.initial_angle * sign).max(lowest);
        }
        // A driver may relax the wheel, then resume the same turn after takeover:
        // measure renewed growth from that admitted motion (carrot 9eb26059a).
        if direction == self.intent && car.steering_angle_deg * sign < self.initial_angle * sign {
            self.initial_angle = car.steering_angle_deg;
        }
        // Pose z is positive for right rotation; Hyundai steering is positive left.
        let signed_yaw = -yaw_rate * sign;
        self.rotation += signed_yaw * dt;
        if following_curve {
            self.confirmed_since = None;
            self.reason = "road_curve";
            return None;
        }
        self.candidate = Some((self.intent, "manual_turn"));
        self.reason = "confirming";
        let turn = self.intent_confirmed
            && car.steering_angle_deg * sign >= 42.
            && (car.steering_angle_deg - self.initial_angle) * sign >= 22.
            && signed_yaw >= 3f64.to_radians()
            && self.rotation >= 1.5f64.to_radians()
            && (!require_growth || (direction == self.intent && (angle - previous.2) * sign > 0.));
        self.confirmed_since = if turn {
            Some(self.confirmed_since.unwrap_or(now))
        } else {
            None
        };
        if emit_allowed
            && self
                .confirmed_since
                .is_some_and(|since| now - since >= 0.10)
        {
            let movement = ManualTurnMovement::new(now, self.intent);
            self.reset("manual_turn");
            return Some(movement);
        }
        None
    }
}
