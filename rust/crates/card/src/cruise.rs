//! Source-equivalent VCruiseCarrot state tail from selfdrive/car/cruise.py.
mod assistant_speed;
mod buttons;
mod integer;
mod number;
mod pedals;
mod state;
pub mod toyota_pcm;
use crate::{
    brands::hyundai::{parameters::setting_int, settings_float},
    core::{Error, StateTail},
};
use num_traits::ToPrimitive;
use number::Number;
use openpilot_cereal::{
    car_capnp::{car_params, car_state},
    log_capnp::event,
};
use openpilot_desire::command::CommandReader;
use openpilot_messaging::state::State;
use openpilot_params::Params;
use std::path::Path;

pub struct CruiseCarrot {
    s: state::CruiseState,
    params: Params,
    pcm_cruise: bool,
    brand: String,
    fingerprint: String,
    openpilot_longitudinal: bool,
    commands: CommandReader,
    assistant_speed: assistant_speed::AssistantSpeed,
    assistant_speed_valid: bool,
    long_active: bool,
    writes: Vec<(String, Vec<u8>)>,
    prints: Vec<String>,
}
impl CruiseCarrot {
    pub fn new(
        cp: car_params::Reader<'_>,
        params: Params,
        root: &Path,
        started: f64,
    ) -> Result<Self, Error> {
        let s = state::CruiseState::new(
            setting_int(&params, "AutoEngage")?,
            setting_int(&params, "UseLaneLineSpeed")?,
            params.get_bool("DisengageOnAccelerator")?,
            params.get_bool("SoftHoldOnCancel")?,
        );
        Ok(Self {
            s,
            params,
            pcm_cruise: cp.get_pcm_cruise(),
            brand: cp.get_brand()?.to_str()?.to_owned(),
            fingerprint: cp.get_car_fingerprint()?.to_str()?.to_owned(),
            openpilot_longitudinal: cp.get_openpilot_longitudinal_control(),
            commands: CommandReader::new(root, "cruise", started),
            assistant_speed: assistant_speed::AssistantSpeed::new(started),
            assistant_speed_valid: false,
            long_active: false,
            writes: Vec::new(),
            prints: Vec::new(),
        })
    }
    pub fn set_speedcam_gas_tok_consumed(&mut self, consumed: bool) {
        self.s.speedcam_gas_tok_consumed = consumed;
    }
    pub fn snapshot(&self) -> impl serde::Serialize + '_ {
        &self.s
    }
    pub fn take_param_writes(&mut self) -> Vec<(String, Vec<u8>)> {
        std::mem::take(&mut self.writes)
    }
    pub fn take_prints(&mut self) -> Vec<String> {
        std::mem::take(&mut self.prints)
    }
    fn put(&mut self, key: &str, value: String) {
        self.writes.push((key.to_owned(), value.into_bytes()));
    }
    fn int(&self, key: &str) -> Result<i32, Error> {
        Ok(setting_int(&self.params, key)?)
    }
    fn float(&self, key: &str) -> Result<f64, Error> {
        Ok(settings_float::read(&self.params, key)?)
    }
    fn add_log(&mut self, log: &str) {
        if log.is_empty() {
            self.s._log_timer = (self.s._log_timer - 1).max(0);
            if self.s._log_timer <= 0 {
                self.s.log.clear();
            }
        } else {
            self.s.log = log.to_owned();
            self.s._log_timer = self.s._log_timeout;
        }
    }
    fn update_params(&mut self, is_metric: bool) -> Result<(), Error> {
        if !self.s.frame.is_multiple_of(10) {
            return Ok(());
        }
        let unit = if is_metric { 1. } else { 1.609344 };
        self.s.auto_cruise_control = f64::from(self.int("AutoCruiseControl")?) * unit;
        self.s.soft_hold_on_cancel = self.params.get_bool("SoftHoldOnCancel")?;
        self.s.auto_gas_tok_speed = f64::from(self.int("AutoGasTokSpeed")?) * unit;
        self.s.auto_gas_cancel_speed = f64::from(self.int("AutoGasCancelSpeed")?) * unit;
        self.s.auto_gas_sync_speed = self.int("AutoGasSyncSpeed")?;
        self.s.apply_model_speed = self.float("ApplyModelSpeed")? * 0.01;
        self.s.auto_speed_upto_road_speed_limit = self.float("AutoSpeedUptoRoadSpeedLimit")? * 0.01;
        self.s.auto_road_speed_adjust = self.float("AutoRoadSpeedAdjust")? * 0.01;
        let lane = f64::from(self.int("UseLaneLineSpeed")?) * unit;
        if self.s.use_lane_line_speed != lane {
            self.s.use_lane_line_speed_apply = lane;
        }
        self.s.use_lane_line_speed = lane;
        self.s.speed_from_pcm = self.int("SpeedFromPCM")?;
        self.s._cruise_speed_unit = self.int("CruiseSpeedUnit")?;
        self.s._cruise_button_long_delay = self.int("CruiseButtonLongDelay")?;
        self.s._cruise_speed_unit_basic = self.int("CruiseSpeedUnitBasic")?;
        self.s._paddle_mode = self.int("PaddleMode")?;
        self.s._cruise_button_mode = self.int("CruiseButtonMode")?;
        self.s._cancel_button_mode = self.int("CancelButtonMode")?;
        self.s._lfa_button_mode = self.int("LfaButtonMode")?;
        self.s.disengage_on_accelerator = self.params.get_bool("DisengageOnAccelerator")?;
        self.s.auto_road_speed_limit_offset = self.int("AutoRoadSpeedLimitOffset")?;
        self.s.auto_navi_speed_safety_factor = self.float("AutoNaviSpeedSafetyFactor")? * 0.01;
        self.s.cruise_on_dist = self.float("CruiseOnDist")? * 0.01;
        let mut first = self.float("CruiseSpeed1")? * unit;
        if first <= 0. {
            first = if self.s.auto_road_speed_limit_offset < 0 {
                self.s.n_road_limit_speed * self.s.auto_navi_speed_safety_factor
            } else {
                self.s.n_road_limit_speed + f64::from(self.s.auto_road_speed_limit_offset)
            };
        }
        self.s._cruise_speed_table = [
            first,
            self.float("CruiseSpeed2")? * unit,
            self.float("CruiseSpeed3")? * unit,
            self.float("CruiseSpeed4")? * unit,
            self.float("CruiseSpeed5")? * unit,
        ];
        Ok(())
    }
    pub fn update_at(
        &mut self,
        cs: car_state::Reader<'_>,
        sm: &State,
        is_metric: bool,
        now: f64,
    ) -> Result<(), Error> {
        let prior_speed = self.s.v_cruise_kph.value;
        self.add_log("");
        self.update_params(is_metric)?;
        self.s.frame += 1;
        if cs.get_gear_shifter()? != car_state::GearShifter::Drive {
            self.s.auto_cruise_control_cancel_timer = 2000;
        } else {
            self.s.auto_cruise_control_cancel_timer =
                (self.s.auto_cruise_control_cancel_timer - 1).max(0);
        }
        let event::Which::CarControl(cc) = sm.topic("carControl")?.event()?.which()? else {
            return Err(Error::Event("carControl"));
        };
        let cc = cc?;
        let control = sm.topic("carControl")?;
        self.assistant_speed_valid =
            control.valid && control.alive && (0. ..0.35).contains(&(now - control.receive_time));
        self.long_active = cc.get_long_active();
        self.update_inputs(sm, now)?;
        self.s.v_cruise_kph_last = self.s.v_cruise_kph;
        self.s.is_metric = is_metric;
        self.s._cancel_timer = (self.s._cancel_timer - 1).max(0);
        self.s.v_ego_kph_set = (f64::from(cs.get_v_ego_cluster()) * 3.6 + 0.5).trunc();
        self.s._activate_cruise = 0;
        let cruise = cs.get_cruise_state()?;
        self.s._cruise_available = cruise.get_available();
        if !self.s._cruise_available {
            self.s._cruise_ready = false;
            self.s._paddle_decel_active = false;
            self.s._soft_hold_count = 0;
            self.s._soft_hold_active = 0;
        }
        self.s._hold_interlock_active = cs.get_brake_hold_active() || cs.get_parking_brake();
        self.s._steering_interlock_active = cs.get_steering_angle_deg().abs() >= 70.;
        if self.s._hold_interlock_active {
            self.s._cruise_ready = false;
            self.s._paddle_decel_active = false;
            self.s._soft_hold_active = 0;
        }
        self.prepare_brake_gas(cs)?;
        if cc.get_enabled() {
            self.s._cruise_ready = false;
        }
        let previous_lat_enabled = self.s._lat_enabled;
        let mut speed = self.update_buttons(cs, cc.get_enabled(), self.s.v_cruise_kph, now)?;
        if self.assistant_speed.active() {
            // A speed request must not enable lateral control.
            self.s._lat_enabled = previous_lat_enabled;
        }
        let eligible = self.fingerprint == "TOYOTA_RAV4_TSS2_2022"
            && self.brand == "toyota"
            && self.pcm_cruise
            && self.openpilot_longitudinal
            && self.s.speed_from_pcm == 0;
        let pcm_speed = self.s.toyota_pcm_speed_sync.update(
            eligible,
            cs,
            (speed.value - prior_speed).abs() > 1e-3,
        )?;
        if self.s._activate_cruise > 0 {
            self.s._cruise_ready = false;
        } else if self.s._activate_cruise < 0 {
            self.s._cruise_ready = self.s._activate_cruise == -2;
        }
        if cruise.get_available() {
            if !self.s.cruise_state_available_last {
                self.s._lat_enabled = true;
                speed = Number::int(self.s.v_ego_kph_set);
            }
            // carrot PR580 `_adopt_stock_cruise`: the device restarted while
            // the camera's stock SCC cruised; take it over at the stock set
            // speed. Interlocks and the cancel latch still apply; re-request
            // every 0.5 s since engage needs a rising edge.
            if self.s.stock_adopt_kph > 0. && !cc.get_enabled() {
                if self.s.frame.is_multiple_of(50) {
                    self.control(
                        1,
                        pedals::CancelTimer::Bypass,
                        "Cruise on (stock SCC)",
                        false,
                        true,
                    );
                }
                speed = Number::float(self.s.stock_adopt_kph);
            }
            if !self.pcm_cruise {
                self.s.v_cruise_kph =
                    speed.clip(self.s._cruise_speed_min, self.s._cruise_speed_max);
                self.s.v_cruise_cluster_kph = self.s.v_cruise_kph;
            } else if self.s.speed_from_pcm == 1
                || (self.s.speed_from_pcm == 3 && self.brand == "toyota")
            {
                self.s.v_cruise_kph = Number::float(f64::from(cruise.get_speed()) * 3.6);
                self.s.v_cruise_cluster_kph =
                    Number::float(f64::from(cruise.get_speed_cluster()) * 3.6);
            } else {
                if let Some([value, _]) = pcm_speed {
                    speed = Number::float(value);
                }
                self.s.v_cruise_kph = speed.clip(30., self.s._cruise_speed_max);
                self.s.v_cruise_cluster_kph = pcm_speed
                    .map_or(self.s.v_cruise_kph, |[_, cluster]| {
                        Number::float(cluster).clip(30., self.s._cruise_speed_max)
                    });
            }
        } else {
            self.s.v_cruise_kph = speed.clip(self.s._cruise_speed_min, self.s._cruise_speed_max);
            self.s.v_cruise_cluster_kph = self.s.v_cruise_kph;
        }
        self.s.cruise_state_available_last = cruise.get_available();
        self.s.enabled_last = cc.get_enabled();
        self.assistant_speed.finish(now, self.s.v_cruise_kph.value);
        Ok(())
    }
    fn update_inputs(&mut self, sm: &State, now: f64) -> Result<(), Error> {
        let topic = sm.topic("carrotMan")?;
        let age = now - topic.receive_time;
        let event::Which::CarrotMan(cm) = topic.event()?.which()? else {
            return Err(Error::Event("carrotMan"));
        };
        let cm = cm?;
        if topic.seen
            && topic.alive
            && topic.valid
            && (0.0..=1.).contains(&age)
            && cm.get_desired_speed() > 0
            && cm.get_desired_speed() <= 250
        {
            self.s.n_road_limit_speed = f64::from(cm.get_n_road_limit_speed());
            self.s.desired_speed = f64::from(cm.get_desired_speed());
            self.s.carrot_cmd_index = cm.get_carrot_cmd_index();
            self.s.carrot_cmd = cm
                .get_carrot_cmd()?
                .to_str()
                .map_err(|_| Error::Numeric)?
                .to_owned();
            self.s.carrot_arg = cm
                .get_carrot_arg()?
                .to_str()
                .map_err(|_| Error::Numeric)?
                .to_owned();
        } else {
            self.s.n_road_limit_speed = 0.;
            self.s.desired_speed = 250.;
            self.s.carrot_cmd.clear();
            self.s.carrot_arg.clear();
        }
        if sm.topic("longitudinalPlan")?.alive {
            let event::Which::LongitudinalPlan(lp) =
                sm.topic("longitudinalPlan")?.event()?.which()?
            else {
                return Err(Error::Event("longitudinalPlan"));
            };
            let lp = lp?;
            self.s.x_state = lp.get_x_state();
            self.s.traffic_state = lp.get_traffic_state();
            self.s.a_target = f64::from(lp.get_a_target());
        }
        if sm.topic("radarState")?.alive {
            let event::Which::RadarState(r) = sm.topic("radarState")?.event()?.which()? else {
                return Err(Error::Event("radarState"));
            };
            let l = r?.get_lead_one()?;
            self.s.d_rel = if l.get_status() {
                f64::from(l.get_d_rel())
            } else {
                0.
            };
            self.s.v_rel = if l.get_status() {
                f64::from(l.get_v_rel())
            } else {
                0.
            };
            self.s.v_lead_kph = if l.get_status() {
                f64::from(l.get_v_lead_k()) * 3.6
            } else {
                0.
            };
        }
        if sm.topic("drivingModelData")?.alive {
            let event::Which::DrivingModelData(m) =
                sm.topic("drivingModelData")?.event()?.which()?
            else {
                return Err(Error::Event("drivingModelData"));
            };
            self.s.model_v_kph = f64::from(m?.get_action()?.get_desired_velocity()) * 3.6;
        }
        Ok(())
    }
    pub fn project(&self, mut cs: car_state::Builder<'_>) -> Result<(), Error> {
        cs.set_log_carrot(self.s.log.as_str());
        cs.set_v_cruise(if self.s._paddle_decel_active {
            0.
        } else {
            self.s.v_cruise_kph.value.to_f32().ok_or(Error::Numeric)?
        });
        cs.set_v_cruise_cluster(if self.s._paddle_decel_active {
            0.
        } else {
            self.s
                .v_cruise_cluster_kph
                .value
                .to_f32()
                .ok_or(Error::Numeric)?
        });
        cs.set_soft_hold_active(self.s._soft_hold_active);
        cs.set_cruise_cancel_state(self.s._cruise_cancel_state);
        cs.set_activate_cruise(self.s._activate_cruise);
        cs.set_lat_enabled(self.s._lat_enabled);
        cs.set_use_lane_line_speed(
            self.s
                .use_lane_line_speed_apply
                .to_f32()
                .ok_or(Error::Numeric)?,
        );
        cs.set_carrot_cruise(i16::from(self.s.carrot_cruise_active));
        Ok(())
    }
}
impl StateTail for CruiseCarrot {
    fn cruise_speed_kph(&self) -> f64 {
        self.s.v_cruise_kph.value
    }
    fn set_speedcam_gas_tok_consumed(&mut self, consumed: bool) {
        self.s.speedcam_gas_tok_consumed = consumed;
    }
    fn set_stock_adopt_kph(&mut self, kph: f64) {
        self.s.stock_adopt_kph = kph;
    }
    fn update(
        &mut self,
        cs: car_state::Builder<'_>,
        sm: &State,
        is_metric: bool,
        now: f64,
    ) -> Result<(), Error> {
        self.update_at(cs.reborrow_as_reader(), sm, is_metric, now)
    }
    fn initialize(
        &mut self,
        _previous: car_state::Reader<'_>,
        _experimental: bool,
    ) -> Result<(), Error> {
        Ok(())
    }
    fn project(&self, cs: car_state::Builder<'_>) -> Result<(), Error> {
        self.project(cs)
    }
    fn take_prints(&mut self) -> Vec<String> {
        self.take_prints()
    }
    fn take_param_writes(&mut self) -> Vec<(String, Vec<u8>)> {
        self.take_param_writes()
    }
}
