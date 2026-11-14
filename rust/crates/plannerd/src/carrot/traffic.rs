use super::{CarrotPlanner, EventName, Input, TrafficState, XState};
use crate::{traffic_stop::entry_allowed, Error};
use openpilot_control_policy::math::{interp, minimum};

/// Frames (20 Hz) a model start sign must last before it releases a stop
/// that is being approached. carrot released on its first green (0.25 s):
/// approaching a red light at 40-70 km/h the model's end speed hovered at the
/// start/stop thresholds, red and green alternated every ~0.5 s, every green
/// dropped the stop and the car accelerated toward the light until the driver
/// braked (route 00000cb1, three times). A real green still releases, 1 s later.
pub(super) const GREEN_RELEASE_FRAMES: u64 = 20;

impl CarrotPlanner {
    pub(super) fn check_stopping(&mut self, input: &Input<'_>, cruise: f64) -> Result<(), Error> {
        let velocities = &input.model.velocity.x;
        let last_v = *velocities
            .last()
            .ok_or(Error::Contract("missing traffic model velocity"))?;
        let first_v = velocities[0];
        let model_x = *input
            .model
            .position
            .x
            .last()
            .ok_or(Error::Contract("missing traffic model position"))?;
        self.velocity_filter.push(last_v);
        let model_v = self.velocity_filter.mean()?;
        let start = model_v > 5. || model_v > first_v + 2.;
        let speed_kph = input.car.v_ego * 3.6;
        let stop = if speed_kph < 1. {
            model_x < 20. && model_v < 10.
        } else if speed_kph < 82. {
            let distance = if input.radar.lead_one.status {
                input.radar.lead_one.d_rel
            } else {
                1000.
            };
            let y = *input
                .model
                .position
                .y
                .last()
                .ok_or(Error::Contract("missing traffic model lateral position"))?;
            model_x < distance - 3.
                && model_x < interp(first_v * 3.6, &[60., 80.], &[120., 150.])?
                && (model_v < 3. || model_v < first_v * 0.7)
                && y.abs() < 5.
                && !(cruise != 0. && self.x_state == XState::E2eCruise && input.car.a_ego < -1.)
        } else {
            false
        };
        self.stop_sign_count = if stop { self.stop_sign_count + 1 } else { 0 };
        self.start_sign_count = if start && !stop {
            self.start_sign_count + 1
        } else {
            0
        };
        self.traffic_state = if self.stop_sign_count > 0 {
            TrafficState::Red
        } else if self.start_sign_count > 4 {
            TrafficState::Green
        } else {
            TrafficState::Off
        };
        Ok(())
    }

    pub(super) fn transition(
        &mut self,
        input: &Input<'_>,
        cruise: &mut f64,
        stop: [f64; 2],
        stop_model: &mut f64,
        previous_traffic: TrafficState,
        clock: &mut impl FnMut() -> f64,
    ) -> Result<(), Error> {
        let [stop_raw, stop_rl] = stop;
        let car = input.car;
        let lead = input.radar.lead_one;
        if car.gas_pressed || car.brake_pressed {
            self.user_stop_distance = -1.;
        }
        if self.soft_hold_active > 0 {
            self.signal_stop_owned = false;
            self.x_state = XState::E2eStopped;
            if matches!(previous_traffic, TrafficState::Off | TrafficState::Red)
                && self.traffic_state == TrafficState::Green
            {
                self.add_signal_event(EventName::TrafficSignChanged, clock);
            }
        } else {
            match self.x_state {
                XState::E2eStopped => {
                    if car.gas_pressed {
                        self.x_state = XState::E2eCruise;
                    } else if super::signal::lead_in_stop_range(lead.status, lead.d_rel, stop_raw) {
                        self.x_state = XState::Lead;
                    } else if self.stopping_count == 0
                        && self.traffic_state == TrafficState::Green
                        && !self.carrot_stay_stop
                        && !car.left_blinker
                        && self.config.traffic_light_mode != 1
                    {
                        self.x_state = XState::E2eCruise;
                        self.add_signal_event(EventName::TrafficSignGreen, clock);
                    }
                    self.stopping_count = self.stopping_count.saturating_sub(1);
                    *cruise = 0.;
                }
                XState::E2eStop => {
                    self.stopping_count = 0;
                    if car.gas_pressed {
                        self.x_state = XState::E2eCruise;
                        self.traffic_starting_count = 200;
                    } else if super::signal::lead_in_stop_range(lead.status, lead.d_rel, stop_raw) {
                        self.x_state = XState::Lead;
                    } else if self.traffic_state == TrafficState::Green
                        && self.start_sign_count >= GREEN_RELEASE_FRAMES
                    {
                        self.add_signal_event(EventName::TrafficSignGreen, clock);
                        self.x_state = XState::E2eCruise;
                    } else {
                        self.comfort_brake = minimum(self.comfort_brake, 2.4 * 0.9);
                        let ratio = interp(car.v_ego * 3.6, &[0., 100.], &[1., 0.7])?;
                        let stop = stop_rl * interp(stop_rl, &[0., 50.], &[1., ratio])?;
                        if stop > 10. {
                            self.actual_stop_distance = stop;
                        }
                        *stop_model = 0.;
                        self.fake_cruise_distance = if self.actual_stop_distance > 10. {
                            0.
                        } else {
                            10.
                        };
                        if car.v_ego < 0.3 {
                            self.stopping_count = 10;
                            self.x_state = XState::E2eStopped;
                        }
                    }
                }
                XState::E2ePrepare => {
                    if lead.status {
                        self.x_state = XState::Lead;
                    } else if self.atc_active {
                        if car.gas_pressed {
                            self.x_state = XState::E2eCruise;
                        }
                    } else if !self.signal_detection_suspended
                        && car.v_ego * 3.6 < 5.
                        && self.traffic_state != TrafficState::Green
                    {
                        self.x_state = XState::E2eStop;
                        self.actual_stop_distance = 5.;
                    } else if car.v_ego * 3.6 > 5. {
                        self.x_state = XState::E2eCruise;
                    }
                }
                XState::Lead | XState::Cruise | XState::E2eCruise | XState::E2eStopPrepare => {
                    self.traffic_starting_count = self.traffic_starting_count.saturating_sub(1);
                    if lead.status {
                        self.x_state = XState::Lead;
                    } else if self.traffic_state == TrafficState::Red
                        && entry_allowed(car.steering_angle_deg)
                        && self.traffic_starting_count == 0
                    {
                        self.add_signal_event(EventName::TrafficStopping, clock);
                        self.x_state = XState::E2eStop;
                        self.actual_stop_distance = stop_rl;
                    } else {
                        self.x_state = XState::E2eCruise;
                    }
                }
            }
        }
        Ok(())
    }
}
