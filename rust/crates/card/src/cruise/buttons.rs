use super::pedals::CancelTimer;
use super::{CruiseCarrot, Number};
use crate::core::Error;
use openpilot_cereal::car_capnp::car_state::{self, button_event::Type as Button};

const REMOTE_BUTTONS: [&str; 5] = [
    "accelCruise",
    "decelCruise",
    "gapAdjustCruise",
    "lfaButton",
    "cancel",
];
fn remote_button(remote: &str) -> Option<Button> {
    match remote {
        "accelCruise" => Some(Button::AccelCruise),
        "decelCruise" => Some(Button::DecelCruise),
        "gapAdjustCruise" => Some(Button::GapAdjustCruise),
        "lfaButton" => Some(Button::LfaButton),
        "cancel" => Some(Button::Cancel),
        _ => None,
    }
}
fn name(button: Button) -> &'static str {
    match button {
        Button::Unknown => "unknown",
        Button::LeftBlinker => "leftBlinker",
        Button::RightBlinker => "rightBlinker",
        Button::AccelCruise => "accelCruise",
        Button::DecelCruise => "decelCruise",
        Button::Cancel => "cancel",
        Button::Lkas => "lkas",
        Button::AltButton2 => "altButton2",
        Button::MainCruise => "mainCruise",
        Button::SetCruise => "setCruise",
        Button::ResumeCruise => "resumeCruise",
        Button::GapAdjustCruise => "gapAdjustCruise",
        Button::LfaButton => "lfaButton",
        Button::PaddleLeft => "paddleLeft",
        Button::PaddleRight => "paddleRight",
    }
}
fn long_speed(speed: Number, button: Button) -> Number {
    let remainder = speed.value.rem_euclid(10.);
    speed.with_value(if button == Button::AccelCruise {
        speed.value + 10. - remainder
    } else {
        speed.value - 10. + (-remainder).rem_euclid(10.)
    })
}
impl CruiseCarrot {
    fn prepare_buttons(
        &mut self,
        cs: car_state::Reader<'_>,
        speed: Number,
        remote: Option<&str>,
    ) -> Result<(Number, Button, bool), Error> {
        let mut speed = speed;
        let mut button = Button::Unknown;
        if let Some(base) = remote.and_then(|r| r.strip_suffix("Long")) {
            if REMOTE_BUTTONS.contains(&base) {
                let bt = remote_button(base).ok_or(Error::Numeric)?;
                if [Button::AccelCruise, Button::DecelCruise].contains(&bt) {
                    speed = long_speed(speed, bt);
                }
                return Ok((speed, bt, true));
            }
        }
        let events: Vec<(Button, bool)> = if let Some(bt) = remote.and_then(remote_button) {
            vec![(bt, true), (bt, false)]
        } else {
            cs.get_button_events()?
                .iter()
                .map(|b| Ok((b.get_type()?, b.get_pressed())))
                .collect::<Result<_, Error>>()?
        };
        if self.s.button_cnt > 0 {
            self.s.button_cnt += 1;
            if [
                u16::from(Button::AccelCruise),
                u16::from(Button::DecelCruise),
            ]
            .contains(&self.s.button_prev)
                && cs.get_cruise_speed_big_step()
            {
                self.s.button_big_step = true;
            }
        }
        for &(bt, pressed) in &events {
            if [Button::PaddleLeft, Button::PaddleRight].contains(&bt) && pressed {
                button = bt;
                self.s.long_pressed = false;
                self.s.button_cnt = 0;
                continue;
            }
            if pressed
                && self.s.button_cnt == 0
                && [
                    Button::AccelCruise,
                    Button::DecelCruise,
                    Button::GapAdjustCruise,
                    Button::Cancel,
                    Button::LfaButton,
                ]
                .contains(&bt)
            {
                self.s.button_cnt = 1;
                self.s.button_prev = u16::from(bt);
                self.s.button_big_step = remote.is_none()
                    && [Button::AccelCruise, Button::DecelCruise].contains(&bt)
                    && cs.get_cruise_speed_big_step();
                self.s.button_long_time = i64::from(self.s._cruise_button_long_delay)
                    + if [Button::AccelCruise, Button::DecelCruise].contains(&bt) {
                        0
                    } else {
                        30
                    };
            } else if !pressed && self.s.button_cnt > 0 && u16::from(bt) == self.s.button_prev {
                if bt == Button::Cancel {
                    button = bt;
                } else if self.s.button_big_step
                    && !self.s.long_pressed
                    && [Button::AccelCruise, Button::DecelCruise].contains(&bt)
                {
                    speed = long_speed(speed, bt);
                    button = bt;
                } else if !self.s.long_pressed {
                    if bt == Button::AccelCruise {
                        let unit = f64::from(self.s._cruise_speed_unit_basic)
                            * if self.s.is_metric { 1. } else { 1.609344 };
                        if unit == 0. {
                            return Err(Error::Numeric);
                        }
                        speed = if self.s.is_metric {
                            Number::int(((speed.value + 0.01) / unit).ceil() * unit)
                        } else {
                            Number::float(((speed.value + 0.01) / unit).ceil() * unit)
                        };
                    } else if bt == Button::DecelCruise {
                        let raw = if [1, 2, 3].contains(&self.s._cruise_button_mode) {
                            self.s._cruise_speed_unit
                        } else {
                            self.s._cruise_speed_unit_basic
                        };
                        let unit = f64::from(raw) * if self.s.is_metric { 1. } else { 1.609344 };
                        if unit == 0. {
                            return Err(Error::Numeric);
                        }
                        speed = if self.s.is_metric {
                            Number::int(((speed.value - 0.01) / unit).floor() * unit)
                        } else {
                            Number::float(((speed.value - 0.01) / unit).floor() * unit)
                        };
                    }
                    button = bt;
                }
                self.s.long_pressed = false;
                self.s.button_cnt = 0;
                self.s.button_big_step = false;
            }
        }
        if self.s.button_cnt > self.s.button_long_time {
            self.s.long_pressed = true;
            let bt = Button::try_from(self.s.button_prev)?;
            if [Button::AccelCruise, Button::DecelCruise].contains(&bt) {
                speed = long_speed(speed, bt);
                button = bt;
                if self.s.button_long_time == 0 {
                    return Err(Error::Numeric);
                }
                self.s.button_cnt = self.s.button_cnt.rem_euclid(self.s.button_long_time);
            } else if self.s.button_cnt < self.s.button_long_time + 2 {
                button = bt;
            }
        }
        if button != Button::Cancel {
            for &(bt, pressed) in &events {
                if !pressed && [Button::SetCruise, Button::ResumeCruise].contains(&bt) {
                    return Ok((speed, bt, false));
                }
            }
        }
        Ok((speed, button, self.s.long_pressed))
    }
    fn carrot_command(
        &mut self,
        mut speed: Number,
        mut button: Button,
        mut long: bool,
    ) -> Result<(Number, Button, bool), Error> {
        if self.s.carrot_cmd_index_last != self.s.carrot_cmd_index {
            self.s.carrot_cmd_index_last = self.s.carrot_cmd_index;
            self.prints.push(format!(
                "Carrot command(cruise.py): {} {}",
                self.s.carrot_cmd, self.s.carrot_arg
            ));
            match (self.s.carrot_cmd.as_str(), self.s.carrot_arg.as_str()) {
                ("CRUISE", "OFF") => self.control(
                    -2,
                    CancelTimer::Bypass,
                    "Cruise off (carrot command)",
                    false,
                    false,
                ),
                ("CRUISE", "ON") => self.control(
                    1,
                    CancelTimer::Bypass,
                    "Cruise on (carrot command)",
                    false,
                    false,
                ),
                ("CRUISE", "GO") => {
                    if button == Button::Unknown {
                        button = Button::AccelCruise;
                        long = false;
                        self.add_log("Cruise accelCruise (carrot command)");
                    }
                }
                ("CRUISE", "STOP") => {
                    speed = Number::int(5.);
                    self.s._cruise_speed_initialized = false;
                    self.add_log("Cruise stop (carrot command)");
                }
                ("SPEED", "UP") => {
                    speed = self.auto_speed_up(speed);
                    self.add_log("Cruise speed up (carrot command)");
                }
                ("SPEED", "DOWN") => {
                    if speed.value > 20. {
                        speed = speed.sub_int(10.);
                        self.add_log("Cruise speed downup (carrot command)");
                    }
                }
                ("SPEED", arg) => {
                    // carrot's `int(self.carrot_arg)` raised on a non-numeric
                    // argument and took card (and vehicle control) down; a
                    // malformed remote command is ignored instead.
                    if let Some(value) = super::integer::command_speed(arg).ok().flatten() {
                        speed = Number::int(value);
                        self.add_log(&format!("Cruise speed set to {speed} (carrot command)"));
                    }
                }
                _ => (),
            }
        }
        Ok((speed, button, long))
    }
    pub(super) fn update_buttons(
        &mut self,
        cs: car_state::Reader<'_>,
        enabled: bool,
        speed: Number,
        now: f64,
    ) -> Result<Number, Error> {
        let allowed = cs.get_can_valid()
            && cs.get_cruise_state()?.get_available()
            && cs.get_gear_shifter()? == car_state::GearShifter::Drive
            && cs.get_button_events()?.is_empty()
            && self.s.button_cnt == 0;
        let mut remote = self.commands.read(allowed, now);
        if self.commands.is_repeat && !enabled {
            remote = None;
        }
        let voice = {
            let minimum = self.s._cruise_speed_min.max(if self.pcm_cruise {
                30.
            } else {
                self.s._cruise_speed_min
            });
            let current = speed.value;
            let allowed = self.assistant_speed_valid
                && cs.get_can_valid()
                && enabled
                && self.long_active
                && self.openpilot_longitudinal
                && cs.get_cruise_state()?.get_available()
                && cs.get_gear_shifter()? == car_state::GearShifter::Drive
                && !(cs.get_brake_pressed()
                    || cs.get_gas_pressed()
                    || !cs.get_button_events()?.is_empty()
                    || remote.is_some()
                    || self.s.button_cnt != 0
                    || self.s._hold_interlock_active)
                && self.s._soft_hold_active == 0
                && minimum <= current
                && current <= self.s._cruise_speed_max
                && !(self.pcm_cruise
                    && (self.s.speed_from_pcm == 1
                        || (self.s.speed_from_pcm == 3 && self.brand == "toyota")))
                && self.s.carrot_cmd_index_last == self.s.carrot_cmd_index;
            self.assistant_speed
                .read(now, allowed, current, minimum, self.s._cruise_speed_max)
        };
        if voice.as_ref().is_some_and(|v| v.action == "speed_up") {
            // Same RES/+ handling, without driver-attention input.
            remote = Some("accelCruise".to_owned());
        }
        let (button_speed, button, long) = self.prepare_buttons(cs, speed, remote.as_deref())?;
        let remote_enable = remote.as_deref().is_some_and(|r| {
            [
                "accelCruise",
                "decelCruise",
                "accelCruiseLong",
                "decelCruiseLong",
            ]
            .contains(&r)
        }) && !enabled
            && !(remote.as_deref() == Some("decelCruise") && self.s._soft_hold_active > 0);
        let (mut speed, button, long) = self.carrot_command(speed, button, long)?;
        if let Some(target) = voice.as_ref().and_then(|v| v.target) {
            speed = Number::float(target);
            self.s._pause_auto_speed_up = true;
            self.s._v_cruise_kph_at_brake = Number::int(0.);
        }
        if [
            Button::AccelCruise,
            Button::DecelCruise,
            Button::SetCruise,
            Button::ResumeCruise,
        ]
        .contains(&button)
        {
            self.s._paddle_decel_active = false;
            self.s._paddle_resume_kph = 0.;
            if self.s.auto_cruise_control_cancel_timer > 0 {
                self.add_log(&format!(
                    "AutoCruiseControl cancel timer RESET {}",
                    name(button)
                ));
                self.s.auto_cruise_control_cancel_timer = 0;
            }
            if self.s._cruise_cancel_state {
                self.add_log(&format!("Cruise Cancel state RESET {}", name(button)));
                self.s._cruise_cancel_state = false;
            }
        }
        if !long {
            match button {
                Button::SetCruise | Button::ResumeCruise => {
                    self.s._lat_enabled = true;
                    self.s._soft_hold_active = 0;
                    self.s._cruise_ready = false;
                    self.s.carrot_cruise_active = false;
                    self.s._pause_auto_speed_up = button == Button::SetCruise;
                    if button == Button::SetCruise {
                        speed = Number::int(self.s.v_ego_kph_set.max(self.s._cruise_speed_min));
                    } else if self.s._v_cruise_kph_at_brake.value > 0. {
                        speed = self.s._v_cruise_kph_at_brake;
                    } else if !self.s._cruise_speed_initialized {
                        speed = Number::int(self.s.v_ego_kph_set.max(self.s._cruise_speed_min));
                    }
                    self.s._v_cruise_kph_at_brake = Number::int(0.);
                    self.s._cruise_speed_initialized = true;
                }
                Button::AccelCruise => {
                    self.s._lat_enabled = true;
                    self.s._pause_auto_speed_up = false;
                    if self.s._soft_hold_active > 0 {
                        self.s._soft_hold_active = 0;
                    } else if self.s.carrot_cruise_active {
                        self.s._v_cruise_kph_at_brake = Number::int(0.);
                    } else if self.s._cruise_ready
                        || !enabled
                        || cs.get_cruise_state()?.get_standstill()
                    {
                        if self.s._v_cruise_kph_at_brake.value > 0. {
                            speed = speed.max(self.s._v_cruise_kph_at_brake);
                            self.s._v_cruise_kph_at_brake = Number::int(0.);
                            self.s._cruise_speed_initialized = true;
                        } else if !self.s._cruise_speed_initialized {
                            speed = Number::int(self.s.v_ego_kph_set.max(self.s._cruise_speed_min));
                            self.s._cruise_speed_initialized = true;
                            self.add_log(&format!("{speed} Cruise resume from current speed"));
                        }
                    } else {
                        self.s._v_cruise_kph_at_brake = Number::int(0.);
                        speed = if self.s._cruise_button_mode == 0 {
                            button_speed
                        } else {
                            self.desired(speed)?
                        };
                    }
                    self.s._cruise_speed_initialized = true;
                    self.s.carrot_cruise_active = false;
                }
                Button::DecelCruise => {
                    self.s._lat_enabled = true;
                    self.s._pause_auto_speed_up = true;
                    if self.s._soft_hold_active > 0 {
                        self.control(
                            -1,
                            CancelTimer::Bypass,
                            "Cruise off,softhold mode (decelCruise)",
                            false,
                            false,
                        );
                    } else if self.s._cruise_ready {
                        self.s._paddle_decel_active = true;
                    } else if !enabled
                        || (self.s.v_ego_kph_set > speed.value + 2.
                            && [2, 3].contains(&self.s._cruise_button_mode))
                    {
                        speed = Number::int(self.s.v_ego_kph_set.max(self.s._cruise_speed_min));
                    } else if [0, 1].contains(&self.s._cruise_button_mode) {
                        speed = button_speed;
                    } else if self.s.v_ego_kph_set < 1. {
                        self.s.carrot_cruise_active = true;
                    } else if self.s.v_ego_kph_set > self.s._cruise_speed_min
                        && speed.value > self.s.v_ego_kph_set
                    {
                        speed = Number::int(self.s.v_ego_kph_set);
                    } else {
                        self.s.carrot_cruise_active = true;
                    }
                    self.s._v_cruise_kph_at_brake = Number::int(0.);
                    self.s._cruise_speed_initialized = true;
                }
                Button::GapAdjustCruise => {
                    let maximum = match self.int("LongitudinalPersonalityMax")? {
                        3 => 3,
                        4 => 4,
                        _ => 3,
                    };
                    let requested = self.int("CruiseGapLevels")?;
                    let mut levels = if requested > 0 {
                        requested.max(2).min(maximum)
                    } else {
                        maximum
                    };
                    if !self.openpilot_longitudinal {
                        levels = maximum;
                    }
                    let personality = if remote.as_deref() == Some("gapAdjustCruise")
                        || cs.get_pcm_cruise_gap() == 0
                        || levels < maximum
                    {
                        let current = self.int("LongitudinalPersonality")?;
                        if 0 < current && current < levels {
                            current - 1
                        } else {
                            levels - 1
                        }
                    } else {
                        (i32::from(cs.get_pcm_cruise_gap()) - 1).clamp(0, maximum - 1)
                    };
                    self.put("LongitudinalPersonality", personality.to_string());
                }
                Button::LfaButton => {
                    match self.s._lfa_button_mode {
                        0 => {
                            self.s._lat_enabled = !self.s._lat_enabled;
                            // carrot's `"Lateral " + "enabled" if ... else "disabled"`
                            // dropped the prefix by precedence.
                            self.add_log(if self.s._lat_enabled {
                                "Lateral enabled"
                            } else {
                                "Lateral disabled"
                            });
                        }
                        2 => self.s.carrot_cruise_active = true,
                        _ => self.s._paddle_decel_active = true,
                    };
                    self.prints.push("lfaButton".to_owned());
                }
                Button::Cancel => {
                    self.s._paddle_decel_active = false;
                    if self.s._cancel_button_mode == 1 {
                        self.s._lat_enabled = false;
                        self.add_log("Lateral disabled");
                    }
                    self.s._cruise_cancel_state = true;
                }
                Button::Unknown
                | Button::LeftBlinker
                | Button::RightBlinker
                | Button::Lkas
                | Button::AltButton2
                | Button::MainCruise
                | Button::PaddleLeft
                | Button::PaddleRight => (),
            }
        } else {
            match button {
                Button::AccelCruise => {
                    speed = button_speed;
                    self.s._v_cruise_kph_at_brake = Number::int(0.);
                }
                Button::DecelCruise => {
                    self.s._pause_auto_speed_up = true;
                    speed = button_speed;
                    self.s._v_cruise_kph_at_brake = Number::int(0.);
                }
                Button::GapAdjustCruise => {
                    let value = self.int("MyDrivingMode")?.rem_euclid(4) + 1;
                    self.put("MyDrivingMode", value.to_string());
                }
                Button::LfaButton => {
                    let lane = self.s.use_lane_line_speed.max(1.);
                    self.s.use_lane_line_speed_apply = if self.s.use_lane_line_speed_apply == 0. {
                        lane
                    } else {
                        0.
                    };
                }
                Button::Cancel => {
                    self.s._cruise_cancel_state = true;
                    self.s._lat_enabled = false;
                    self.s._paddle_decel_active = false;
                    self.add_log("Lateral disabled");
                }
                Button::Unknown
                | Button::LeftBlinker
                | Button::RightBlinker
                | Button::Lkas
                | Button::AltButton2
                | Button::MainCruise
                | Button::SetCruise
                | Button::ResumeCruise
                | Button::PaddleLeft
                | Button::PaddleRight => (),
            }
        }
        if remote.as_deref() == Some("carrotCruise") {
            self.s.carrot_cruise_active = true;
        } else if remote.as_deref() == Some("paddleDecel") {
            self.control(
                -2,
                CancelTimer::Bypass,
                "Cruise off & Ready (Bluetooth paddle)",
                false,
                false,
            );
            self.s._paddle_decel_active = true;
        } else if self.s._paddle_mode > 0
            && [Button::PaddleLeft, Button::PaddleRight].contains(&button)
        {
            if self.s._paddle_mode == 3 {
                self.s.carrot_cruise_active = true;
            } else {
                self.control(
                    -2,
                    CancelTimer::Bypass,
                    "Cruise off & Ready (paddle)",
                    false,
                    false,
                );
                if self.s._paddle_mode == 2 {
                    self.s._paddle_decel_active = true;
                }
            }
        } else if self.s._paddle_decel_active && !enabled {
            self.control(
                1,
                CancelTimer::Bypass,
                "Cruise on (paddle decel)",
                false,
                false,
            );
        }
        let updated = self.update_cruise_state(cs, enabled, speed)?;
        if ![Button::SetCruise, Button::ResumeCruise].contains(&button) {
            speed = updated;
        }
        if remote
            .as_deref()
            .is_some_and(|r| ["cancel", "cancelLong"].contains(&r))
        {
            self.control(
                -3,
                CancelTimer::Bypass,
                "Cruise off (Bluetooth cancel)",
                true,
                true,
            );
        } else if remote_enable
            && !cs.get_brake_pressed()
            && !cs.get_gas_pressed()
            && self.s._activate_cruise >= 0
        {
            self.control(
                1,
                CancelTimer::Bypass,
                "Cruise on (Bluetooth button)",
                false,
                true,
            );
            if self.s._activate_cruise > 0 {
                self.s._lat_enabled = true;
            }
        }
        Ok(speed)
    }
}

#[cfg(test)]
mod tests {
    use super::{Button, Number};

    #[test]
    fn a_malformed_speed_command_is_ignored_instead_of_failing_the_frame() {
        let mut c = super::super::pedals::tests::cruise();
        for (arg, expected) in [("RESET", 80.), ("", 80.), ("120", 120.), ("250", 80.)] {
            c.s.carrot_cmd = "SPEED".into();
            c.s.carrot_arg = arg.into();
            c.s.carrot_cmd_index += 1;
            let (speed, _, _) = c
                .carrot_command(Number::int(80.), Button::Unknown, false)
                .unwrap_or_else(|e| panic!("{arg:?}: {e:?}"));
            assert_eq!(speed.value, expected, "{arg:?}");
        }
    }
}
