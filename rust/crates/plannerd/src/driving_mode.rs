use crate::{lead::Lead, Error};
use openpilot_control_policy::math::{maximum, minimum};
use serde::{Deserialize, Serialize};

#[derive(Clone, Copy, Debug, Deserialize, Serialize, PartialEq, Eq)]
#[serde(try_from = "i32", into = "i32")]
pub enum DrivingMode {
    Eco,
    Safe,
    Normal,
    High,
}

impl TryFrom<i32> for DrivingMode {
    type Error = Error;
    fn try_from(value: i32) -> Result<Self, Error> {
        match value {
            1 => Ok(Self::Eco),
            2 => Ok(Self::Safe),
            3 => Ok(Self::Normal),
            4 => Ok(Self::High),
            _ => Err(Error::Contract("unsupported driving mode")),
        }
    }
}
impl From<DrivingMode> for i32 {
    fn from(value: DrivingMode) -> Self {
        match value {
            DrivingMode::Eco => 1,
            DrivingMode::Safe => 2,
            DrivingMode::Normal => 3,
            DrivingMode::High => 4,
        }
    }
}

impl DrivingMode {
    /// `MyDrivingMode` as stored: outside 1..=4 (a missing or corrupt setting)
    /// reads as the settings default, Normal. carrot's `DrivingMode(...)`
    /// raised there and plannerd failed every cycle.
    pub fn from_setting(value: i32) -> Self {
        Self::try_from(value).unwrap_or(Self::Normal)
    }
    pub fn lead_response(self, requested: i32) -> i32 {
        let ceiling = match self {
            Self::Eco => 2,
            Self::Safe => 3,
            Self::Normal | Self::High => 5,
        };
        requested.clamp(0, ceiling)
    }
    pub const fn comfort_brake_factor(self) -> f64 {
        match self {
            Self::Safe => 0.9,
            Self::Eco | Self::Normal | Self::High => 1.,
        }
    }
    pub const fn factors(self) -> [f64; 2] {
        match self {
            Self::Eco => [0.9, 2. - 0.9],
            Self::Safe => [0.8, 2. - 0.8],
            Self::Normal | Self::High => [1., 1.],
        }
    }
}

#[derive(Clone, Copy, Debug, Deserialize)]
#[serde(deny_unknown_fields)]
pub struct TrafficSample {
    pub valid: bool,
    pub dt: f64,
    pub ego_speed: f64,
    pub lead: Lead,
}

#[derive(Debug, Default, Serialize)]
pub struct DrivingModeDetector {
    congested: bool,
    stop_time: f64,
    slow_time: f64,
    recovery_time: f64,
    clear_time: f64,
    accel_time: f64,
    lead_key: Option<(bool, i32)>,
}

impl DrivingModeDetector {
    fn reset_evidence(&mut self) {
        self.stop_time = 0.;
        self.slow_time = 0.;
        self.recovery_time = 0.;
        self.clear_time = 0.;
        self.accel_time = 0.;
    }

    pub fn update(&mut self, input: &TrafficSample) {
        let lead = &input.lead;
        let dt = input.dt;
        if !input.valid || !dt.is_finite() || dt <= 0. || dt > 0.2 || !input.ego_speed.is_finite() {
            self.reset_evidence();
            self.lead_key = None;
            return;
        }
        let ego = maximum(0., input.ego_speed);
        if !lead.status {
            self.stop_time = 0.;
            self.slow_time = 0.;
            self.recovery_time = 0.;
            self.accel_time = 0.;
            self.lead_key = None;
            self.clear_time = if ego >= 15. * (1. / 3.6) {
                self.clear_time + dt
            } else {
                0.
            };
            if self.clear_time >= 4. {
                self.congested = false;
            }
            return;
        }
        if ![lead.d_rel, lead.v_lead, lead.v_rel, lead.a_lead_k]
            .iter()
            .all(|value| value.is_finite())
            || lead.d_rel <= 0.
        {
            self.reset_evidence();
            self.lead_key = None;
            return;
        }
        self.clear_time = 0.;
        let speed = maximum(0., lead.v_lead);
        let key = (lead.radar, lead.radar_track_id);
        if self.lead_key != Some(key) {
            self.recovery_time = 0.;
            self.accel_time = 0.;
        }
        self.lead_key = Some(key);
        let approach = minimum(200., maximum(12., ego * ego / (2. * 2.4) + 2. * ego));
        let stopping = speed <= 5. * (1. / 3.6) && lead.d_rel <= approach;
        let following = lead.d_rel <= minimum(80., maximum(30., 12. + 3. * ego));
        let slow = following && ego <= 35. * (1. / 3.6) && speed <= 30. * (1. / 3.6);
        self.stop_time = if stopping {
            minimum(0.30, self.stop_time + dt)
        } else {
            0.
        };
        self.slow_time = if slow {
            minimum(8., self.slow_time + dt)
        } else {
            0.
        };
        let accelerating = !stopping && lead.a_lead_k > 1.5;
        self.accel_time = if accelerating {
            minimum(0.5, self.accel_time + dt)
        } else {
            0.
        };
        let flowing = ego >= 35. * (1. / 3.6) && speed >= 35. * (1. / 3.6);
        let opening = speed >= 15. * (1. / 3.6) && lead.v_rel >= 1. && lead.d_rel >= 8. + 1.8 * ego;
        let recovering = !stopping && lead.a_lead_k >= -0.2 && (flowing || opening);
        self.recovery_time = if recovering {
            minimum(6., self.recovery_time + dt)
        } else {
            0.
        };
        if self.accel_time >= 0.5 || self.recovery_time >= 6. {
            self.congested = false;
            self.stop_time = 0.;
            self.slow_time = 0.;
        } else if self.stop_time >= 0.30 || self.slow_time >= 8. {
            self.congested = true;
        }
    }

    pub const fn mode(&self, automatic: i32) -> DrivingMode {
        if self.congested {
            DrivingMode::Safe
        } else if automatic == 2 {
            DrivingMode::Eco
        } else {
            DrivingMode::Normal
        }
    }
}

#[cfg(test)]
mod setting_tests {
    use super::DrivingMode;

    #[test]
    fn an_out_of_range_driving_mode_reads_as_normal() {
        assert_eq!(DrivingMode::from_setting(1), DrivingMode::Eco);
        assert_eq!(DrivingMode::from_setting(4), DrivingMode::High);
        for value in [0, 5, -1] {
            assert_eq!(DrivingMode::from_setting(value), DrivingMode::Normal);
        }
    }
}
