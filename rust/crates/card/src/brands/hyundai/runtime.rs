use super::{
    bus::Fingerprints,
    config::CarConfig,
    controller::Controller,
    flags as f,
    state::{CanSetup, State},
    Error,
};
use capnp::message::{Builder, HeapAllocator};
use openpilot_can::Packet;
use openpilot_cereal::car_capnp::car_state;
use openpilot_params::Params;
use std::path::Path;

pub struct Setup<'a> {
    pub params_bytes: &'a [u8],
    pub dbc_root: &'a Path,
    pub settings: Params,
    pub fingerprints: &'a Fingerprints,
    pub now_ns: u64,
}

pub struct Hyundai {
    pub state: State,
    pub controller: Controller,
    pub spas: super::spas_signals::SpasSignals,
    spas_logged: (String, f64),
    pub kick: super::kick::Kick,
}

impl Hyundai {
    pub fn new(setup: Setup<'_>) -> Result<Self, Error> {
        let config = CarConfig::decode(setup.params_bytes, &setup.settings)?;
        let path = setup.dbc_root.join(if config.flags & f::CANFD != 0 {
            "hyundai_canfd_generated.dbc"
        } else {
            "hyundai_kia_generic.dbc"
        });
        let state = State::new(
            config,
            setup.settings,
            CanSetup {
                path: &path,
                fingerprints: setup.fingerprints,
                now_ns: setup.now_ns,
            },
        )?;
        let controller = Controller::new(&state)?;
        let spas = super::spas_signals::SpasSignals::new(
            state.config.flags & f::CANFD_SPAS_BLINKERS != 0,
            state.config.bus.ecan,
            &state.config.candidate,
        );
        let mut kick = super::kick::Kick::new(
            state.config.kick && state.config.flags & f::CANFD != 0,
            state.config.bus,
        );
        kick.no_lever = state.config.kick_no_lever;
        Ok(Self {
            state,
            controller,
            spas,
            spas_logged: (String::new(), 0.),
            kick,
        })
    }

    pub fn update(
        &mut self,
        packets: &[Packet],
        now_ns: u64,
    ) -> Result<Builder<HeapAllocator>, Error> {
        let mut message = self.state.update(packets, now_ns)?;
        let now = now_ns as f64 * 1e-9;
        self.spas.observe(packets, now);
        if self.kick.configured {
            self.kick.refresh(&self.state.settings, now);
            self.kick
                .observe(packets, message.get_root::<car_state::Builder<'_>>()?, now);
        }
        Ok(message)
    }
    pub fn apply(&mut self, input: super::ApplyInput<'_>) -> Result<super::ApplyOutput, Error> {
        let mut spas_sends = Vec::new();
        if self.spas.enabled {
            if let Some(subscribers) = input.subscribers {
                let now = input.now_ns as f64 * 1e-9;
                let cs = self
                    .state
                    .out
                    .get_root_as_reader::<car_state::Reader<'_>>()?;
                spas_sends = self.spas.update(super::spas_signals::Inputs {
                    cs,
                    subscribers,
                    settings: &self.state.settings,
                    now,
                    initialized: input.initialized,
                    controller_frame: u64::from(self.controller.frame),
                    writer: &mut self.controller.writer,
                    bus: self.state.config.bus,
                })?;
                // carrot card.py: CC.hudControl.physicalBlinkerHold = hold_value.
                self.controller.external_hold = self.spas.hold_value;
                self.log_spas(cs_blinkers(&self.state)?, now);
            }
        }
        if !self.spas.enabled {
            // controlsd's physical-hold.json lease (B1 path, carrot physical.py).
            self.controller.external_hold =
                input.control.get_hud_control()?.get_physical_blinker_hold();
        }
        let now = input.now_ns as f64 * 1e-9;
        if self.kick.configured {
            // carrot card.py `kick_valid`.
            let fresh = |name: &str| {
                input.subscribers.is_some_and(|subs| {
                    subs.topic(name).is_ok_and(|t| {
                        t.valid && t.alive && (0. ..0.2).contains(&(now - t.receive_time))
                    })
                })
            };
            let cs = self
                .state
                .out
                .get_root_as_reader::<car_state::Reader<'_>>()?;
            let valid = cs.get_can_valid()
                && !input.impact_reboot
                && input.model.is_some()
                && fresh("carControl")
                && fresh("modelV2");
            self.kick.update(valid, now);
            match input.subscribers {
                Some(subs) => {
                    self.kick
                        .request(cs, now, input.initialized, subs, &self.state.settings)?;
                }
                None => self.kick.direction = 0,
            }
            // The SPAS lease path owns APRK when it is on; an idle state
            // still gives the ECU back after the Kick was switched off.
            if self.state.config.flags & f::CANFD_SPAS_BLINKERS == 0 && self.kick.aprk_running() {
                spas_sends.extend(self.kick.aprk_blinkers(
                    &mut self.controller.writer,
                    cs,
                    self.controller.frame,
                )?);
            }
        }
        let isolation_mask = input
            .model
            .and_then(|model| model.get_meta().ok())
            .map(|meta| meta.get_indicator_isolation_mask());
        let mut output = self.controller.apply(&mut self.state, input)?;
        if !spas_sends.is_empty() {
            spas_sends.append(&mut output.can);
            output.can = spas_sends;
        }
        if self.kick.configured {
            output.can = self
                .kick
                .isolate(std::mem::take(&mut output.can), isolation_mask, now);
            let cs = self
                .state
                .out
                .get_root_as_reader::<car_state::Reader<'_>>()?;
            self.kick.feedback(cs, now, &output.can);
        }
        Ok(output)
    }
    /// carrot card.py `cloudlog.event('hyundai_spas', ...)`: on every status
    /// change, and once a second while a session or a parked trial runs.
    fn log_spas(&mut self, blinkers: (bool, bool), now: f64) {
        let s = &self.spas;
        let running = s.session.state != super::spas::State::Idle
            || (s.test_direct && s.test_request.is_some());
        if s.status != self.spas_logged.0 || (running && now >= self.spas_logged.1) {
            self.state.inputs.diagnostics.logs.push(crate::core::VehicleLog {
                level: crate::query::DiagnosticLevel::Info,
                message: format!(
                    "hyundai_spas state={} reason={} request={:?} direction={:?} left={} right={} hold={} trial={} trial_status={} restored_ack={} stock_resumed={}",
                    s.session.state.name(),
                    s.session.reason,
                    s.session.request(),
                    s.session.direction(),
                    blinkers.0,
                    blinkers.1,
                    s.hold_value,
                    s.test_setting,
                    s.status,
                    s.session.restore_ack,
                    s.session.stock_resumed,
                ),
            });
            self.spas_logged = (s.status.clone(), now + 1.);
        }
    }
    pub fn init(&mut self, io: &mut impl crate::firmware_query::StartupIo) -> Result<(), Error> {
        super::startup::init(&mut self.state, io)
    }
    pub fn deinit(&mut self, _io: &mut impl crate::firmware_query::StartupIo) -> Result<(), Error> {
        Ok(())
    }
    pub fn set_soft_hold(&mut self, active: i16) {
        self.state.soft_hold = active;
    }
    pub fn commit_state(&mut self, state: car_state::Reader<'_>) -> Result<(), Error> {
        self.state.commit(state)
    }
    pub fn take_diagnostics(&mut self) -> Vec<String> {
        let mut lines = std::mem::take(&mut self.state.inputs.diagnostics.prints);
        lines.append(&mut self.controller.buttons.diagnostics);
        lines
    }
    pub fn take_logs(&mut self) -> Vec<crate::core::VehicleLog> {
        let mut logs = std::mem::take(&mut self.state.inputs.diagnostics.logs);
        logs.append(&mut self.controller.steering.logs);
        logs
    }
    pub fn take_warnings(&mut self) -> Vec<String> {
        let mut warnings = std::mem::take(&mut self.state.inputs.diagnostics.warnings);
        if let Some(stopping) = &mut self.controller.stopping {
            warnings.append(&mut stopping.warnings);
        }
        warnings
    }
}

fn cs_blinkers(state: &State) -> Result<(bool, bool), Error> {
    let cs = state.out.get_root_as_reader::<car_state::Reader<'_>>()?;
    Ok((cs.get_left_blinker(), cs.get_right_blinker()))
}
