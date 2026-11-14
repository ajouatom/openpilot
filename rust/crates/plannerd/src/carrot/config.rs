use super::{CarrotPlanner, DrivingMode, GapParameters};
use crate::{parameters::Parameters, Error};
use openpilot_control_policy::math::interp;

#[derive(Debug)]
pub struct Config {
    pub automatic_mode: i32,
    pub traffic_light_mode: i32,
    pub gaps: GapParameters,
    pub response_base: i32,
    pub response_overrides: [i32; 4],
    pub lane_change_ratio: f64,
    pub cruise_maximum: [f64; 7],
    pub stop_distance: f64,
    pub eco_over_speed: i32,
    pub navigation_decel: f64,
    pub change_cost_starting: f64,
    pub traffic_stop_adjust: f64,
}

impl Default for Config {
    fn default() -> Self {
        Self {
            automatic_mode: 0,
            traffic_light_mode: 2,
            gaps: GapParameters::default(),
            response_base: 0,
            response_overrides: [-1; 4],
            lane_change_ratio: 1.,
            cruise_maximum: [1.6, 1.6, 1.2, 1., 0.8, 0.7, 0.6],
            stop_distance: 6.,
            eco_over_speed: 2,
            navigation_decel: 1.5,
            change_cost_starting: 10.,
            traffic_stop_adjust: 2.5,
        }
    }
}

impl CarrotPlanner {
    pub(super) fn parameters_update(
        &mut self,
        parameters: &mut impl Parameters,
    ) -> Result<(), Error> {
        self.frame += 1;
        self.params_count += 1;
        if self.params_count.is_multiple_of(10) {
            let selected = DrivingMode::from_setting(parameters.integer("MyDrivingMode")?);
            if selected != self.driving_mode_last {
                self.disable_auto = true;
            }
            self.driving_mode_last = selected;
            self.config.automatic_mode = parameters.integer("MyDrivingModeAuto")?;
            self.driving_mode = if self.config.automatic_mode > 0 && !self.disable_auto {
                self.detector.mode(self.config.automatic_mode)
            } else {
                selected
            };
        }
        match self.params_count {
            10 => self.config.traffic_light_mode = parameters.integer("TrafficLightDetectMode")?,
            20 => {
                for (index, key) in ["TFollowGap1", "TFollowGap2", "TFollowGap3", "TFollowGap4"]
                    .into_iter()
                    .enumerate()
                {
                    self.config.gaps.gaps[index] = parameters.float(key)? / 100.;
                }
                self.config.response_base = parameters.integer("LeadAccelResponse")?.clamp(0, 5);
                for (index, key) in [
                    "LeadAccelResponseTF1",
                    "LeadAccelResponseTF2",
                    "LeadAccelResponseTF3",
                    "LeadAccelResponseTF4",
                ]
                .into_iter()
                .enumerate()
                {
                    self.config.response_overrides[index] = parameters.integer(key)?.clamp(-1, 5);
                }
                self.config.lane_change_ratio = parameters.float("DynamicTFollowLC")? / 100.;
                self.config.gaps.speed_factor = parameters.integer("SpeedTFFactor")?.clamp(10, 30);
                self.config.gaps.decel_boost = parameters.float("TFollowDecelBoost")? / 100.;
            }
            30 => {
                for (index, key) in [
                    "CruiseMaxVals0",
                    "CruiseMaxVals1",
                    "CruiseMaxVals2",
                    "CruiseMaxVals3",
                    "CruiseMaxVals4",
                    "CruiseMaxVals5",
                    "CruiseMaxVals6",
                ]
                .into_iter()
                .enumerate()
                {
                    self.config.cruise_maximum[index] = parameters.float(key)? / 100.;
                }
            }
            40 => {
                self.config.stop_distance = parameters.float("StopDistanceCarrot")? / 100.;
                self.config.eco_over_speed = parameters.integer("CruiseEcoControl")?;
                self.config.navigation_decel =
                    f64::from(parameters.integer("AutoNaviSpeedDecelRate")?) * 0.01;
                self.config.change_cost_starting = parameters.float("AChangeCostStarting")?;
                self.config.traffic_stop_adjust =
                    parameters.float("TrafficStopDistanceAdjust")? / 100.;
            }
            100.. => self.params_count = 0,
            _ => {}
        }
        Ok(())
    }

    pub fn acceleration(&self, speed: f64) -> Result<f64, Error> {
        let factor = if self.driving_mode == DrivingMode::High {
            1.2
        } else {
            self.safe_factor
        };
        Ok(interp(
            speed,
            &[
                0.,
                10. * (1. / 3.6),
                40. * (1. / 3.6),
                60. * (1. / 3.6),
                80. * (1. / 3.6),
                110. * (1. / 3.6),
                140. * (1. / 3.6),
            ],
            &self.config.cruise_maximum,
        )? * factor)
    }
}
