use super::Error;
use openpilot_control_policy::math::{clip, interp};

#[derive(Default, Debug, serde::Serialize)]
pub struct AngleAuthority {
    pub max_torque: f64,
    pub steering_pressed_prev: bool,
    pub recovering: bool,
    pub recovery_frames: u32,
    pub override_count: u32,
    pub override_latched: bool,
    pub release_frames: u32,
    pub filtered: f64,
    pub filtered_prev: f64,
    pub pre_override_frames: u32,
    /// Frames the wheel has been held while lateral is active.
    pub pressed_frames: u32,
}

#[derive(Clone, Copy, serde::Deserialize)]
pub struct AuthorityInput {
    pub active: bool,
    pub pressed: bool,
    pub torque: f64,
    pub threshold: f64,
    pub y_std: Option<f64>,
}

impl AngleAuthority {
    pub fn update(&mut self, i: AuthorityInput) -> Result<f64, Error> {
        if i.pressed && !self.steering_pressed_prev {
            self.recovering = true;
        }
        // carrot PR580: a hand-resting torque blip shorter than 0.3 s must not
        // count as a repeated override or restart the full-recovery timer.
        self.pressed_frames = if i.pressed && i.active {
            self.pressed_frames.saturating_add(1)
        } else {
            0
        };
        if self.pressed_frames == 30 {
            if self.recovery_frames > 0 && self.recovery_frames < 500 {
                self.override_count = (self.override_count + 1).min(3);
            }
            self.recovery_frames = 0;
        }
        let threshold = i.threshold.max(1.);
        if !i.active {
            self.filtered = i.torque;
            self.filtered_prev = i.torque;
            self.pre_override_frames = 0;
        } else {
            self.filtered_prev = self.filtered;
            self.filtered += (0.01 / (0.12 + 0.01)) * (i.torque - self.filtered);
        }
        let filtered_abs = self.filtered.abs();
        let rate = ((filtered_abs - self.filtered_prev.abs()) / 0.01).max(0.);
        let ratio = filtered_abs / threshold;
        let raw_ratio = i.torque.abs() / threshold;
        let predicted = (filtered_abs + rate * 0.15) / threshold;
        let candidate = i.active
            && !i.pressed
            && raw_ratio > 0.7
            && ratio > 0.65
            && predicted > 0.9
            && rate > threshold * 0.5;
        self.pre_override_frames = if candidate {
            self.pre_override_frames + 1
        } else {
            0
        };
        let yield_ratio = if self.pre_override_frames >= 2 {
            interp(predicted, &[0.9, 1.05], &[0., 1.])?
        } else {
            0.
        };
        let mut delta = 0.;
        let mut recovery = false;
        if i.pressed {
            self.override_latched = true;
            self.release_frames = 0;
            delta = -20.;
        } else if yield_ratio > 0. {
            delta = -10. * yield_ratio;
        } else if self.max_torque >= 250. {
            delta = 0.;
        } else if self.override_latched {
            self.release_frames = if ratio < 0.6 {
                self.release_frames + 1
            } else {
                0
            };
            if self.release_frames >= 20 {
                self.override_latched = false;
                self.release_frames = 0;
                recovery = true;
            }
        } else {
            recovery = true;
        }
        if recovery {
            let y_std = i.y_std.filter(|v| v.is_finite() && *v >= 0.).unwrap_or(0.2);
            let time = interp(y_std, &[0.1, 0.2, 0.3, 0.4], &[0.5, 0.8, 1.5, 3.])?.max(interp(
                f64::from(self.override_count),
                &[0., 1., 2., 3.],
                &[0.1, 1., 2., 3.],
            )?);
            delta = (250. - 25.) * 0.01 / time * interp(ratio, &[0.6, 0.8], &[1., 0.])?;
        }
        self.max_torque = clip(self.max_torque + delta, 25., 250.);
        if !i.pressed && self.recovering && self.max_torque >= 250. {
            self.recovering = false;
            self.recovery_frames = 1;
        } else if !i.pressed && self.recovery_frames > 0 {
            self.recovery_frames += 1;
            if self.recovery_frames >= 500 {
                self.recovery_frames = 0;
                self.override_count = 0;
            }
        }
        if !i.active {
            *self = Self::default();
        }
        self.steering_pressed_prev = i.pressed && i.active;
        Ok(self.max_torque)
    }
}
