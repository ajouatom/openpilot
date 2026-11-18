use super::{
    controller_buttons::Buttons,
    controller_control::Steering,
    controller_model::Model,
    controller_settings::Settings,
    flags as f,
    jerk::{CruiseInput, HyundaiJerk, JerkInput},
    lead::LeadLateralFilter,
    limits::TorqueLimits,
    state::State,
    state_fields::float32,
    stopping::CanfdStopping,
    wire::{CanWriter, Values},
    Error,
};
use crate::core::{ApplyInput, ApplyOutput};
use capnp::message::Builder;
use openpilot_can::Frame;
use openpilot_cereal::car_capnp::{car_control, car_state};
use std::collections::BTreeMap;

pub struct Controller {
    pub frame: u32,
    pub writer: CanWriter,
    /// carrot `hudControl.physicalBlinkerHold`: 0 none, 1 release, 2 left, 3 right.
    pub external_hold: u8,
    pub settings: Settings,
    pub steering: Steering,
    pub buttons: Buttons,
    pub jerk: HyundaiJerk,
    pub lateral: LeadLateralFilter,
    pub blink_hold: super::blink_hold::BlinkHold,
    /// carrot `create_ccnc_messages._camera_sync` (PR580).
    pub camera_sync: super::camera_sync::CameraSync,
    pub stopping: Option<CanfdStopping>,
    pub accel_last: f64,
    pub value_last: f64,
    pub lkas_active: bool,
    pub lane_check: i32,
    pub steering_template_ready: bool,
    /// Comma Pedal command source on configured cars (carrot `PedalInterceptor`).
    pub pedal: Option<super::pedal::Interceptor>,
}

impl Controller {
    pub fn new(state: &State) -> Result<Self, Error> {
        Ok(Self {
            frame: 0,
            external_hold: 0,
            writer: CanWriter::from_dbc(std::sync::Arc::clone(&state.inputs.pt.dbc)),
            pedal: super::pedal::configured(&state.config.candidate, state.config.flags)
                .zip(state.inputs.pedal.as_ref())
                .map(|(profile, parser)| {
                    super::pedal::Interceptor::new(std::sync::Arc::clone(&parser.dbc), profile)
                }),
            settings: Settings::new(
                TorqueLimits::new(&state.config.candidate, state.config.flags),
                &state.settings,
            )?,
            steering: Steering::default(),
            buttons: Buttons::default(),
            jerk: HyundaiJerk::default(),
            lateral: LeadLateralFilter::default(),
            blink_hold: super::blink_hold::BlinkHold::default(),
            camera_sync: super::camera_sync::CameraSync::default(),
            stopping: if state.config.flags & f::CANFD != 0 && state.config.longitudinal {
                Some(CanfdStopping::default())
            } else {
                None
            },
            accel_last: 0.,
            value_last: 0.,
            lkas_active: false,
            lane_check: 0,
            steering_template_ready: false,
        })
    }

    pub fn jerk(
        &mut self,
        input: (car_control::Reader<'_>, &State),
        command: &super::controller_control::Command,
    ) -> Result<(), Error> {
        let (cc, state) = input;
        let cs = state.out.get_root_as_reader::<car_state::Reader<'_>>()?;
        let actuator = cc.get_actuators()?;
        let hud = cc.get_hud_control()?;
        self.jerk.make(JerkInput {
            canfd: state.config.flags & f::CANFD != 0,
            state: actuator.get_long_control_state()?,
            accel: command.accel,
            measured_accel: f64::from(cs.get_a_ego()),
            actuator_jerk: f64::from(actuator.get_jerk()),
            brake: cs.get_brake_pressed(),
            gas: cs.get_gas_pressed(),
        })?;
        self.jerk.check_cruise(CruiseInput {
            decel: super::settings_float::read(&state.settings, "CarrotCruiseDecel")?,
            atc_decel: super::settings_float::read(&state.settings, "CarrotCruiseAtcDecel")?,
            atc_distance: f64::from(hud.get_atc_distance()),
            carrot_cruise: i32::from(cs.get_carrot_cruise()),
            override_active: cc.get_cruise_control()?.get_override(),
            soft_hold: state.soft_hold != 0,
            stopping: command.stopping,
            speed: f64::from(cs.get_v_ego()),
            target_accel: f64::from(actuator.get_a_target()),
            accel: command.accel,
            measured_accel: f64::from(cs.get_a_ego()),
        });
        Ok(())
    }

    pub fn apply(
        &mut self,
        state: &mut State,
        input: ApplyInput<'_>,
    ) -> Result<ApplyOutput, Error> {
        let mut guarded_control = Builder::new_default();
        let mut cc = input.control;
        if !self.steering_template_ready
            && state.config.flags & f::CANFD != 0
            && state.config.flags & f::CAMERA_SCC != 0
        {
            let key = if state.config.flags & f::ANGLE_CONTROL != 0 {
                "lfa_alt"
            } else {
                "lfa"
            };
            self.steering_template_ready = state.inputs.captured(key)?.is_some();
            if !self.steering_template_ready && cc.get_lat_active() {
                guarded_control.set_root(cc)?;
                guarded_control
                    .get_root::<car_control::Builder<'_>>()?
                    .set_lat_active(false);
                cc = guarded_control.get_root_as_reader::<car_control::Reader<'_>>()?;
            }
        }
        if self.frame.is_multiple_of(50) {
            self.settings.refresh(&state.settings)?;
        }
        self.settings
            .lane_change(cc.get_hud_control()?.get_model_desire());
        let model = Model::read(input.model, input.radar)?;
        let command = self.steering.update(
            (cc, state, &model, input.now_ns),
            &self.settings,
            self.frame,
        )?;
        let captures: BTreeMap<&str, Values> = state
            .inputs
            .captures
            .keys()
            .map(|key| {
                Ok((
                    *key,
                    state
                        .inputs
                        .captured(key)?
                        .ok_or_else(|| Error::Signal((*key).into()))?,
                ))
            })
            .collect::<Result<_, Error>>()?;
        let mut can = Vec::new();
        if self.frame.is_multiple_of(100)
            && state.config.flags & f::CAMERA_SCC == 0
            && state.config.longitudinal
            && self.pedal.is_none()
        {
            let (address, bus) = if state.config.flags & f::HDA2 != 0 {
                (0x730, state.config.bus.ecan)
            } else {
                (
                    0x7d0,
                    if state.config.flags & f::CANFD != 0 {
                        state.config.bus.ecan
                    } else {
                        0
                    },
                )
            };
            can.push(Frame {
                address,
                bus,
                data: vec![2, 0x3e, 0x80, 0, 0, 0, 0, 0],
            });
            if state.config.flags & f::ENABLE_BLINKERS != 0 {
                can.push(Frame {
                    address: 0x7b1,
                    bus: state.config.bus.ecan,
                    data: vec![2, 0x3e, 0x80, 0, 0, 0, 0, 0],
                });
            }
        }
        if state.config.flags & f::CANFD != 0 {
            can.extend(super::controller_fd::messages(
                self,
                (cc, state),
                (&command, &model, &captures),
            )?);
        } else {
            can.extend(super::controller_legacy::messages(
                self,
                (cc, state),
                (&command, &captures),
            )?);
        }
        let mut actuators = Builder::new_default();
        actuators.set_root(cc.get_actuators()?)?;
        let mut output = actuators.get_root::<car_control::actuators::Builder<'_>>()?;
        output.set_torque(float32(command.torque / self.settings.limits.max)?);
        output.set_torque_output_can(float32(if state.config.flags & f::ANGLE_CONTROL != 0 {
            command.authority
        } else {
            command.torque
        })?);
        output.set_steering_angle_deg(float32(command.angle)?);
        output.set_stop_object_active(
            self.stopping
                .as_ref()
                .is_some_and(|stopping| stopping.stop_object_active),
        );
        output.set_accel(float32(command.accel)?);
        self.frame += 1;
        Ok(ApplyOutput { actuators, can })
    }
}
