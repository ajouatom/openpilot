use crate::{
    centering::{Centering, Sample},
    config::Config,
    inputs::{Inputs, Pose},
    interface::ControlInterface,
    lateral::{Lateral, Log},
    longitudinal::Longitudinal,
    parameters::Parameters,
    suspend::Suspend,
    Error,
};
use openpilot_control_policy::{
    drive::{self, Plan},
    math::{interp, maximum, smooth},
    pid::{Gain, Gains, Multiplicative, Pid, Step},
    vehicle::VehicleModel,
};

pub struct Command {
    pub enabled: bool,
    pub lateral: bool,
    pub longitudinal: bool,
    pub left: bool,
    pub right: bool,
    /// `carControl.assistantLaneSignalId` for a native voice indicator.
    pub assistant_id: String,
    pub accel: f32,
    pub target: f32,
    pub jerk: f32,
    pub curvature: f32,
    pub torque: f32,
    pub angle: f32,
    pub log: Log,
    pub errors: Vec<String>,
}
pub struct Controls {
    pub config: Config,
    pub interface: ControlInterface,
    pub vm: VehicleModel,
    pub longitudinal: Longitudinal,
    pub lateral: Lateral,
    pub suspend: Suspend,
    pub meb: Option<Pid>,
    pub curvature: f64,
    pub desired: f64,
    pub safety_limited: bool,
    pub lateral_startup: openpilot_control_policy::lateral_readiness::Gate,
    pub lateral_started: bool,
    pub lane_lines: bool,
    /// LanelessCentering's state (see `centering.rs`).
    pub centering: Centering,
    pub turn_speed: i32,
    pub calibration: [f64; 3],
    pub pose: Option<Pose>,
    pub physical_hold: crate::physical_hold::HoldReader,
}
impl Controls {
    pub fn new(
        config: Config,
        interface: ControlInterface,
        params: &mut impl Parameters,
    ) -> Result<Self, Error> {
        let mut meb = config.meb().then(|| {
            Pid::new(
                Gains {
                    p: Gain {
                        x: vec![10., 40.],
                        y: vec![0., 1.45],
                    },
                    i: Gain {
                        x: vec![10., 40.],
                        y: vec![0., 0.12],
                    },
                    d: Gain::constant(0.),
                    f: 1.,
                },
                [-0.2, 0.2],
            )
        });
        if let Some(pid) = &mut meb {
            pid.multiplicative = Some(Multiplicative {
                previous_override: false,
                factor: 1.,
                min_command: 1e-10,
                reduction_time: 1.,
            });
        }
        let turn_speed = params.integer("AutoTurnControlSpeedTurn")?;
        let longitudinal = Longitudinal::new(&config, params)?;
        let lateral = Lateral::new(&config, &interface, params)?;
        let vm = VehicleModel::new(config.physical.clone());
        let physical_hold = crate::physical_hold::HoldReader::new(&config.fingerprint);
        Ok(Self {
            config,
            interface,
            vm,
            longitudinal,
            lateral,
            suspend: Suspend::default(),
            meb,
            curvature: 0.,
            desired: 0.,
            safety_limited: false,
            lateral_startup: Default::default(),
            lateral_started: false,
            lane_lines: false,
            centering: Centering::default(),
            turn_speed,
            calibration: [0.; 3],
            pose: None,
            physical_hold,
        })
    }
    pub fn control(
        &mut self,
        input: &Inputs,
        params: &mut impl Parameters,
    ) -> Result<Command, Error> {
        self.update_pose(input);
        let cs = &input.car;
        let lateral_ready = self.lateral_startup.update(input.startup_inputs());
        let live = if lateral_ready {
            input.live
        } else {
            input.live.for_inactive(
                input.startup.services.live,
                self.config.physical.steer_ratio,
            )
        };
        let ratio = drive::steer_ratio(
            live.ratio,
            params.float("SteerRatioRate")?,
            params.float("CustomSR")?,
        );
        self.vm.update_geometry(input.center_to_front_ratio);
        self.vm.update(maximum(live.stiffness, 0.1), ratio);
        let angle = (cs.steer_angle - live.offset).to_radians();
        self.curvature = -self.vm.curvature(angle, cs.speed, live.roll)?;
        if self.config.angle
            && matches!(self.config.tuning, crate::config::Tuning::Torque(_))
            && input.torque_checks
            && input.torque.use_params
        {
            return Err(Error::Contract(
                "angle controller cannot update torque tuning",
            ));
        }
        self.lateral.live(input);
        let jetlink_lost =
            input.model.jetlink_latched || (input.model.jetlink && !input.model_checks);
        let always = cs.driving_gear && params.boolean("AlwaysLateral")? && !jetlink_lost;
        let below = cs.speed.abs() <= maximum(self.config.min_steer_speed, 0.3);
        let standstill = self.config.brand == "tesla" && self.config.standstill_steering;
        let lateral = input.selfdrive.current
            && (input.selfdrive.active || always)
            && cs.lat_enabled
            && !cs.fault_temporary
            && !cs.fault_permanent
            && ((cs.standstill && standstill) || (!cs.standstill && !below));
        // AlwaysLateral must also stop while manager drains workers for an impact reboot.
        let impact_reboot = params.boolean("ImpactDashcamReboot")?;
        let lateral = self.suspend.update(cs, lateral, params)?
            && lateral_ready
            && !input.vehicle_fault
            && !impact_reboot;
        let longitudinal = input.selfdrive.enabled
            && !input.override_long
            && !input.vehicle_fault
            && !impact_reboot
            && self.config.openpilot_long;
        if !lateral {
            self.lateral.reset();
        }
        if lateral && !self.lateral_started {
            self.desired = self.curvature;
            self.lateral_started = true;
        }
        if !longitudinal {
            self.longitudinal.reset();
        }
        let limits = self.interface.identity.accel_limits(
            self.config.flags,
            cs.speed,
            cs.cruise * (1. / 3.6),
        )?;
        let [accel, target, jerk] =
            self.longitudinal
                .update(&self.config, params, longitudinal, input, limits)?;
        self.lane_lines = input.lateral.lane_lines
            && input.carrot.turn_speed.abs() > f64::from(params.integer("UseLaneLineCurveSpeed")?);
        let tau = params.float("LatSmoothSec")? * 0.01;
        let delay = params.float("SteerActuatorDelay")? * 0.01;
        let delay = if delay == 0. { input.delay } else { delay };
        let lag = || {
            drive::lag_adjusted(
                cs.speed,
                delay + tau,
                Plan {
                    psis: &input.lateral.psis,
                    curvatures: &input.lateral.curvatures,
                    distances: &input.lateral.distances,
                },
            )
        };
        let new = if !lateral {
            self.curvature
        } else if self.config.meb() {
            if input.lateral.lane_lines && !input.lateral.curvatures.is_empty() {
                smooth(lag()?, self.desired, tau)
            } else {
                input.model.desired_curvature
            }
        } else if self.lane_lines {
            if input.lateral.curvatures.is_empty() {
                self.curvature
            } else {
                smooth(lag()?, self.desired, tau)
            }
        } else {
            let model = &input.model;
            let turn_or_lane = model.desire.iter().skip(1).take(4).sum::<f64>() >= 0.2;
            let atc = input.carrot.turn_type.as_str();
            let atc_idle = matches!(atc, "" | "none" | "straight") || atc.ends_with(" prepare");
            let correction = self.centering.update(&Sample {
                strength: f64::from(params.integer("LanelessCentering")?) * 0.01,
                allowed: !model.lane_change && !turn_or_lane && atc_idle && !cs.steering_pressed,
                speed: cs.speed,
                model_curvature: model.desired_curvature,
                lane_x: &model.lane_x,
                left_y: &model.left_y,
                right_y: &model.right_y,
                left_prob: model.left_prob,
                right_prob: model.right_prob,
                path_x: &model.path_x,
                path_y: &model.path_y,
            });
            smooth(model.desired_curvature + correction, self.desired, 0.1)
        };
        if !lateral || self.lane_lines || self.config.meb() {
            self.centering = Centering::default();
        }
        let (desired, curvature_limited) =
            drive::clip_curvature(cs.speed, self.desired, new, live.roll)?;
        self.desired = desired;
        let mut curvature = desired;
        if let Some(pid) = &mut self.meb {
            if !lateral {
                pid.reset();
            } else {
                let compensation = -self.vm.roll(live.roll, cs.speed)?;
                let no_roll = -self.vm.curvature(angle, cs.speed, 0.)?;
                let mut actual = no_roll - compensation;
                if let Some(pose) = &self.pose {
                    if cs.speed > 5. {
                        actual = interp(
                            cs.speed,
                            &[2., 5.],
                            &[actual, pose.angular[2] / maximum(cs.speed, 0.1)],
                        )?;
                    }
                }
                let mut step = Step::new(desired - actual, cs.speed, desired - compensation);
                step.freeze = self.safety_limited || cs.speed < 5. || cs.steering_pressed;
                step.driver_override = cs.steering_pressed;
                curvature = pid.update(step)? + (cs.steer_curvature - no_roll);
            }
        }
        let (torque, angle, log) = self.lateral.update(
            &self.interface,
            params,
            crate::lateral::Request {
                config: &self.config,
                vm: &self.vm,
                input,
                live: &live,
                active: lateral,
                curvature: desired,
                limited: self.safety_limited,
                curvature_limited,
            },
        )?;
        let route = input.model.assistant_route.as_str();
        let native_voice = route == "native" && !input.model.assistant_id.is_empty() && lateral;
        let lane_lamps = input.selfdrive.current
            && input.model.lane_change
            && (matches!(route, "" | "ordinary") || native_voice);
        let mut command = Command {
            enabled: input.selfdrive.enabled && !impact_reboot,
            lateral,
            longitudinal,
            // carrot voice_indicator.set_lane_change_blinkers: only ordinary or
            // admitted native voice maneuvers drive the lane-change lamps.
            left: lane_lamps && input.model.lane_left,
            right: lane_lamps && input.model.lane_right,
            assistant_id: if native_voice {
                input.model.assistant_id.clone()
            } else {
                String::new()
            },
            accel: accel as f32,
            target: target as f32,
            jerk: jerk as f32,
            curvature: curvature as f32,
            torque: torque as f32,
            angle: angle as f32,
            log,
            errors: Vec::new(),
        };
        crate::sanitize::apply(&mut command, self.longitudinal.state)?;
        Ok(command)
    }
}
