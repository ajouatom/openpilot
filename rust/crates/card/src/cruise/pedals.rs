use super::{CruiseCarrot, Number};
use crate::core::Error;
use openpilot_cereal::car_capnp::car_state;

#[derive(Clone, Copy)]
pub(super) enum CancelTimer {
    Bypass,
    Immediate,
    Traffic,
    Braking,
}
impl CancelTimer {
    fn frames(self) -> i64 {
        match self {
            Self::Bypass => -100,
            Self::Immediate => 0,
            Self::Traffic => 300,
            Self::Braking => 500,
        }
    }
}

impl CruiseCarrot {
    pub(super) fn control(
        &mut self,
        enable: i16,
        timer: CancelTimer,
        reason: &str,
        allow_cancel: bool,
        manual: bool,
    ) {
        if enable > 0 && !self.s._cruise_available {
            self.s._activate_cruise = 0;
            self.add_log(&format!("{reason} > Cruise unavailable"));
            return;
        }
        if enable > 0 && self.s._steering_interlock_active {
            self.s._activate_cruise = 0;
            self.add_log(&format!("{reason} > Steering angle interlock active"));
            return;
        }
        if enable > 0 && self.s._hold_interlock_active {
            self.s._activate_cruise = 0;
            self.add_log(&format!("{reason} > Brake hold interlock active"));
            return;
        }
        if self.s._cruise_cancel_state && !allow_cancel {
            self.add_log(&format!("{reason} > Cancel state"));
        } else if enable > 0 && self.s._cancel_timer > 0 && timer.frames() >= 0 {
            self.add_log(&format!("{reason} > Canceled"));
        } else {
            if !manual && self.s.auto_cruise_control == 0. && enable != 0 {
                self.s._soft_hold_active = 0;
                return;
            }
            if !manual && self.s.auto_cruise_control_cancel_timer > 0 && enable != 0 {
                self.add_log(&format!("{reason} > timer Canceled"));
                self.s._soft_hold_active = 0;
                return;
            }
            self.s._activate_cruise = enable;
            self.s._cancel_timer = timer.frames();
            self.add_log(reason);
        }
    }
    pub(super) fn prepare_brake_gas(&mut self, cs: car_state::Reader<'_>) -> Result<(), Error> {
        if cs.get_gas_pressed() {
            let start = self.s._gas_pressed_count <= 0;
            let cancel = start && self.s._soft_hold_active > 0 && self.s._cruise_cancel_state;
            self.s._paddle_decel_active = false;
            self.s._paddle_resume_kph = 0.;
            self.s._gas_pressed_count = (self.s._gas_pressed_count + 1).max(1);
            self.s._gas_pressed_count_last = self.s._gas_pressed_count;
            self.s._gas_pressed_value = if self.s._gas_pressed_count > 1 {
                f64::from(cs.get_gas()).max(self.s._gas_pressed_value)
            } else {
                f64::from(cs.get_gas())
            };
            self.s._gas_tok = false;
            self.s._soft_hold_active = 0;
            if cancel {
                self.s._cruise_ready = false;
                self.s.carrot_cruise_active = false;
                self.control(
                    -1,
                    CancelTimer::Bypass,
                    "Cruise off (cancel soft hold released)",
                    true,
                    false,
                );
            } else if start && self.s.disengage_on_accelerator {
                self.s._cruise_ready = false;
                self.s.carrot_cruise_active = false;
                self.control(
                    -1,
                    CancelTimer::Immediate,
                    "Cruise off (gas pressed)",
                    false,
                    false,
                );
            }
        } else {
            self.s._gas_tok = 0 < self.s._gas_pressed_count
                && self.s._gas_pressed_count < self.s._gas_tok_timer
                && !self.s.speedcam_gas_tok_consumed;
            self.s._gas_pressed_count = (self.s._gas_pressed_count - 1).min(-1);
            if self.s._gas_pressed_count < -1 {
                self.s._gas_pressed_count_last = 0;
                self.s._gas_tok = false;
            }
        }
        if cs.get_brake_pressed() {
            self.s._cruise_ready = false;
            self.s._paddle_decel_active = false;
            self.s._paddle_resume_kph = 0.;
            self.s._brake_pressed_count = (self.s._brake_pressed_count + 1).max(1);
            if self.s._brake_pressed_count == 1 && self.s.enabled_last {
                self.s._v_cruise_kph_at_brake = self.s.v_cruise_kph;
                self.add_log(&format!("{} Cruise speed at brake", self.s.v_cruise_kph));
            }
            let available = cs.get_cruise_state()?.get_available()
                && self.s.auto_cruise_control != 0.
                && !self.pcm_cruise
                && self.s.auto_cruise_control_cancel_timer == 0
                && (!self.s._cruise_cancel_state || self.s.soft_hold_on_cancel);
            self.s._soft_hold_count = if available
                && cs.get_v_ego() < 0.1
                && cs.get_gear_shifter()? == car_state::GearShifter::Drive
            {
                self.s._soft_hold_count + 1
            } else {
                0
            };
            self.s._soft_hold_active = if available && self.s._soft_hold_count > 60 {
                1
            } else {
                0
            };
        } else {
            self.s._soft_hold_count = 0;
            self.s._brake_pressed_count = (self.s._brake_pressed_count - 1).min(-1);
        }
        Ok(())
    }
    pub(super) fn update_cruise_state(
        &mut self,
        cs: car_state::Reader<'_>,
        enabled: bool,
        mut speed: Number,
    ) -> Result<Number, Error> {
        let ego = Number::int(self.s.v_ego_kph_set);
        let v = f64::from(cs.get_v_ego());
        if !enabled {
            self.s._paddle_resume_kph = 0.;
        }
        if !enabled {
            if self.s._brake_pressed_count == -1 && self.s._soft_hold_active > 0 {
                self.s._soft_hold_active = 2;
                self.control(
                    1,
                    CancelTimer::Bypass,
                    "Cruise on (soft hold)",
                    self.s.soft_hold_on_cancel,
                    false,
                );
            } else if self.params.get_bool("ActivateCruiseAfterBrake")? {
                self.put("ActivateCruiseAfterBrake", "0".to_owned());
                self.control(1, CancelTimer::Bypass, "Cruise on (brake)", false, false);
            } else if self.s.v_cruise_kph.value < ego.value {
                self.s.v_cruise_kph = ego;
            }
        }
        if !self.s.disengage_on_accelerator
            && self.s._gas_tok
            && ego.value >= self.s.auto_gas_tok_speed
        {
            if !enabled {
                self.control(1, CancelTimer::Bypass, "Cruise on (gas tok)", false, false);
                if ego.value > speed.value {
                    speed = ego;
                }
            } else {
                speed = self.desired(speed)?;
            }
        } else if self.s._gas_pressed_count == -1 {
            if 0. < self.s.d_rel && self.s.d_rel < v * 0.8 {
                if v < 1. {
                    self.control(
                        1,
                        if self.s.a_target > 0. {
                            CancelTimer::Bypass
                        } else {
                            CancelTimer::Immediate
                        },
                        "Cruise on (safe speed)",
                        false,
                        false,
                    );
                } else {
                    self.control(
                        -1,
                        CancelTimer::Immediate,
                        "Cruise off (lead car too close)",
                        false,
                        false,
                    );
                }
            } else if ego.value < self.s.auto_gas_cancel_speed {
                self.control(
                    -1,
                    CancelTimer::Immediate,
                    "Cruise off (gas speed)",
                    false,
                    false,
                );
            } else if self.s.x_state == 3 {
                speed = ego.min(speed);
                self.control(
                    -1,
                    CancelTimer::Traffic,
                    "Cruise off (traffic sign)",
                    false,
                    false,
                );
            } else if !cs.get_left_blinker()
                && !cs.get_right_blinker()
                && !self.s.disengage_on_accelerator
                && ego.value >= self.s.auto_gas_tok_speed
                && !enabled
            {
                speed = ego.min(speed);
                self.control(
                    1,
                    if self.s.a_target > 0. {
                        CancelTimer::Bypass
                    } else {
                        CancelTimer::Immediate
                    },
                    "Cruise on (gas pressed)",
                    false,
                    false,
                );
            }
        } else if self.s._brake_pressed_count == -1 && self.s._soft_hold_active == 0 {
            if !cs.get_left_blinker() && !cs.get_right_blinker() {
                if ego.value > self.s.auto_gas_tok_speed {
                    speed = ego;
                    self.control(
                        1,
                        if self.s.a_target > 0. {
                            CancelTimer::Bypass
                        } else {
                            CancelTimer::Immediate
                        },
                        "Cruise on (speed)",
                        false,
                        false,
                    );
                } else if cs.get_steering_angle_deg().abs() < 20. {
                    if [3, 5].contains(&self.s.x_state) {
                        if self.s.x_state == 3 {
                            speed = ego;
                        }
                        self.control(
                            1,
                            CancelTimer::Immediate,
                            "Cruise on (traffic sign)",
                            false,
                            false,
                        );
                    } else if 0. < self.s.d_rel && self.s.d_rel < 20. {
                        self.control(
                            1,
                            if ego.value < 1. {
                                CancelTimer::Bypass
                            } else {
                                CancelTimer::Immediate
                            },
                            "Cruise on (lead car)",
                            false,
                            false,
                        );
                    }
                }
            }
        } else if self.s._brake_pressed_count < 0 && self.s._gas_pressed_count < 0 {
            if !enabled {
                if self.s.d_rel > 0. && v > 0.02 {
                    let safe = self.s.d_rel - v.powi(2) / 3. - self.s.v_rel.powi(2) / 3.;
                    if cs.get_steering_angle_deg().abs() <= 70. {
                        if safe < 4. {
                            self.control(1, CancelTimer::Bypass, "Cruise on (fcw)", false, false);
                        } else if self.s.d_rel < self.s.cruise_on_dist {
                            self.control(
                                1,
                                CancelTimer::Immediate,
                                "Cruise on (fcw dist)",
                                false,
                                false,
                            );
                        } else {
                            self.add_log(&format!(
                                "leadCar d={:.1},v={:.1},{v:.1}, {safe:.1}",
                                self.s.d_rel, self.s.v_rel
                            ));
                        }
                    }
                }
                if !cs.get_left_blinker() && !cs.get_right_blinker() {
                    if self.s.desired_speed < ego.value {
                        self.control(
                            1,
                            CancelTimer::Bypass,
                            "Cruise on (desired speed)",
                            false,
                            false,
                        );
                    }
                    if self.s._cruise_ready {
                        if self.s.x_state == 3 {
                            self.control(
                                1,
                                CancelTimer::Immediate,
                                "Cruise on (traffic sign)",
                                false,
                                false,
                            );
                        } else if self.s.d_rel > 0. {
                            self.control(
                                1,
                                CancelTimer::Immediate,
                                "Cruise on (lead car)",
                                false,
                                false,
                            );
                        }
                    }
                }
            } else if self.s._paddle_decel_active && self.s._paddle_mode == 2 {
                // PaddleMode 2: hand the paddle stop to the traffic-light
                // stop (xState 3 slowing, 5 stopped) or to a (nearly) stopped
                // lead and keep the driver's set speed; a hand-off at a stop
                // otherwise left the 5 km/h floor, and any moving lead
                // cancelled the stop request.
                if matches!(self.s.x_state, 3 | 5) || (self.s.d_rel > 0. && v + self.s.v_rel < 1.) {
                    self.s._paddle_decel_active = false;
                    if v > 0.5 {
                        // Still rolling: hold the current speed into the
                        // stop; the set speed returns once stopped.
                        self.s._paddle_resume_kph = speed.value;
                        self.s._paddle_resume_cnt = 0;
                        speed = ego;
                    }
                }
            } else if self.s._paddle_decel_active && (self.s.x_state == 3 || self.s.d_rel > 0.) {
                self.s._paddle_decel_active = false;
                speed = ego;
            } else if self.s._paddle_resume_kph > 0. {
                // Give the set speed back once stopped, or when the light has
                // not asked for a stop for 1 s (green before we stopped).
                self.s._paddle_resume_cnt = if matches!(self.s.x_state, 3 | 5) {
                    0
                } else {
                    self.s._paddle_resume_cnt + 1
                };
                if v < 0.1 || self.s._paddle_resume_cnt > 100 {
                    if self.s._paddle_resume_kph > speed.value {
                        speed = Number::int(self.s._paddle_resume_kph);
                    }
                    self.s._paddle_resume_kph = 0.;
                }
            }
        }
        if self.s._gas_pressed_count > self.s._gas_tok_timer {
            if cs.get_a_ego() < -0.5 {
                self.control(
                    -1,
                    CancelTimer::Braking,
                    "Cruise off (gas pressed while braking)",
                    false,
                    false,
                );
            }
            if ego.value > speed.value && self.s.auto_gas_sync_speed != 0 {
                speed = ego;
            }
        }
        if self.s._gas_pressed_count == 1 || v < 0.1 {
            self.s._pause_auto_speed_up = false;
            if self.s._gas_pressed_count == 1 && v < 0.1 {
                self.control(
                    -1,
                    CancelTimer::Bypass,
                    "Cruise off (gasPressed)",
                    false,
                    false,
                );
            }
        } else if self.s._brake_pressed_count > 0 {
            self.s._pause_auto_speed_up = true;
        }
        Ok(self.auto_speed_up(speed))
    }
    pub(super) fn auto_speed_up(&mut self, mut speed: Number) -> Number {
        if !self.s._pause_auto_speed_up && self.s.apply_model_speed != 0. {
            let model = self.s.model_v_kph * self.s.apply_model_speed.abs();
            // carrot capped with `nRoadLimitSpeed * 1.1` unconditionally; with no
            // road limit (0) a negative ApplyModelSpeed set the cruise speed to 0,
            // i.e. the minimum. Without a limit only the model speed applies.
            let apply = if self.s.n_road_limit_speed > 0. {
                model.min(self.s.n_road_limit_speed * 1.1)
            } else if self.s.apply_model_speed < 0. {
                model
            } else {
                0.
            };
            if self.s.apply_model_speed < 0. {
                speed = Number::float(model.min(apply));
            } else if speed.value < apply {
                speed = Number::float(apply);
            }
        }
        let limit = self.s.n_road_limit_speed * self.s.auto_speed_upto_road_speed_limit;
        if limit < 1. {
            return speed;
        }
        if self.s.auto_road_speed_limit_offset > 0 {
            self.s._v_cruise_kph_at_brake = Number::int(
                self.s.n_road_limit_speed + f64::from(self.s.auto_road_speed_limit_offset),
            );
        }
        if !self.s._pause_auto_speed_up
            && self.s.v_lead_kph + 5. > speed.value
            && speed.value < limit
            && self.s.d_rel < 60.
        {
            speed = Number::float((speed.value + 5.).min(limit));
        } else if self.s.auto_road_speed_adjust < 0.
            && self.s.n_road_limit_speed != self.s.n_road_limit_speed_last
        {
            speed = if self.s.auto_road_speed_limit_offset < 0 {
                Number::float(self.s.n_road_limit_speed * self.s.auto_navi_speed_safety_factor)
            } else {
                Number::int(
                    self.s.n_road_limit_speed + f64::from(self.s.auto_road_speed_limit_offset),
                )
            };
        } else if self.s.n_road_limit_speed < self.s.n_road_limit_speed_last
            && self.s.auto_road_speed_adjust > 0.
        {
            let limit = self.s.n_road_limit_speed * self.s.auto_road_speed_adjust
                + speed.value * (1. - self.s.auto_road_speed_adjust);
            self.add_log(&format!("AutoSpeed change {speed} -> {limit:.1}"));
            speed = speed.min(Number::float(limit));
        }
        self.s.road_limit_kph = limit;
        self.s.n_road_limit_speed_last = self.s.n_road_limit_speed;
        speed
    }
    pub(super) fn desired(&self, mut speed: Number) -> Result<Number, Error> {
        if self.s._cruise_button_mode == 3 {
            if let Some(next) = self
                .s
                ._cruise_speed_table
                .iter()
                .find(|&&v| speed.value < v)
            {
                speed = Number::float(*next);
            } else {
                let unit = f64::from(self.s._cruise_speed_unit);
                if unit == 0. {
                    return Err(Error::Numeric);
                }
                speed = Number::float(((speed.value / unit).floor() + 1.) * unit);
            }
        } else if speed.value < 30. {
            speed = Number::int(30.);
        } else {
            if self.s._cruise_speed_unit == 0 {
                return Err(Error::Numeric);
            }
            if self.s._cruise_speed_unit > 0 {
                let mut next = 40;
                while next < 160 {
                    if speed.value < f64::from(next) {
                        speed = Number::int(f64::from(next));
                        break;
                    }
                    next += self.s._cruise_speed_unit;
                }
            }
        }
        Ok(speed)
    }
}

#[cfg(test)]
pub(super) mod tests {
    use super::super::{
        assistant_speed::AssistantSpeed, number::Number, state::CruiseState, CruiseCarrot,
    };
    use openpilot_desire::command::CommandReader;

    pub(in crate::cruise) fn cruise() -> CruiseCarrot {
        let root = std::env::temp_dir().join(format!("card-cruise-{}", std::process::id()));
        std::fs::create_dir_all(&root).unwrap();
        CruiseCarrot {
            s: CruiseState::new(0, 0, false, false),
            params: openpilot_params::Params::open(&root, "d").unwrap(),
            pcm_cruise: false,
            brand: "hyundai".into(),
            fingerprint: String::new(),
            openpilot_longitudinal: true,
            commands: CommandReader::new(&root, "cruise", 0.),
            assistant_speed: AssistantSpeed::new(0.),
            assistant_speed_valid: false,
            long_active: false,
            writes: Vec::new(),
            prints: Vec::new(),
        }
    }

    #[test]
    fn a_negative_model_speed_without_a_road_limit_follows_the_model() {
        let mut c = cruise();
        c.s.apply_model_speed = -1.;
        c.s.model_v_kph = 72.;
        c.s.n_road_limit_speed = 0.;
        // carrot computed min(model, 0) here: the set speed collapsed to 0.
        assert_eq!(c.auto_speed_up(Number::float(100.)).value, 72.);
        c.s.n_road_limit_speed = 50.;
        assert!((c.auto_speed_up(Number::float(100.)).value - 55.).abs() < 1e-9);
        // A positive ratio still never raises the speed without a road limit.
        c.s.apply_model_speed = 1.;
        c.s.n_road_limit_speed = 0.;
        assert_eq!(c.auto_speed_up(Number::float(40.)).value, 40.);
    }
}
