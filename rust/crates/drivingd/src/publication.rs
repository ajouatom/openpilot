use crate::{state::State, wire, Error};
use openpilot_cereal::{
    car_capnp::car_state,
    custom_capnp::carrot_man,
    log_capnp::{event, radar_state},
};
use openpilot_desire::{
    helper::DesireHelper,
    types::{Config, Indicator},
};
use openpilot_modeld::{
    action::{self, ActionInputs, PlanActionInput},
    derived_wire::{self, PoseTiming},
    model_wire::{self, ModelFrame, ModelTiming},
    prediction::DrivingPrediction,
    publication::PublishState,
};

#[derive(Default)]
pub struct Publication {
    pub desire: DesireHelper,
    publish: PublishState,
}

impl Publication {
    pub fn new(desire: DesireHelper) -> Self {
        Self {
            desire,
            publish: PublishState::default(),
        }
    }
}

pub struct Sources<'a> {
    pub car: car_state::Reader<'a>,
    pub navigation: carrot_man::Reader<'a>,
    pub radar: radar_state::Reader<'a>,
    pub lateral_active: bool,
    pub live_lateral_delay: f64,
    /// Freshness-only observer inputs; `valid` is joined with modelV2 here.
    pub indicator: Indicator,
}

pub struct Output<'a> {
    pub prediction: &'a DrivingPrediction,
    pub timing: ModelTiming,
    pub simulation: bool,
    pub dropped: u32,
    pub raw_predictions: Option<&'a [u8]>,
}

pub struct Messages {
    pub model: capnp::message::Builder<capnp::message::HeapAllocator>,
    pub driving: capnp::message::Builder<capnp::message::HeapAllocator>,
    pub pose: capnp::message::Builder<capnp::message::HeapAllocator>,
}

impl Publication {
    pub fn build(
        &mut self,
        state: &mut State,
        output: Output<'_>,
        sources: Sources<'_>,
        config: impl FnMut() -> Config,
        command: impl FnMut(bool) -> Option<String>,
    ) -> Result<Messages, Error> {
        let (lat_action_t, long_action_t) = state.action_times();
        let velocity = f64::from(sources.car.get_v_ego());
        let action = action::from_plan(
            PlanActionInput {
                plan: &output.prediction.plan,
                direct_action: output.prediction.direct_action,
            },
            state.previous_action,
            ActionInputs {
                lat_action_t,
                long_action_t,
                v_ego: if velocity < 0.0 { 0.0 } else { velocity },
                lat_smooth_seconds: action::dynamic_lat_smooth_seconds(
                    state.settings.lateral_smooth,
                ),
                v_ego_stopping: state.settings.v_ego_stopping,
            },
        );
        state.complete_action(action, sources.live_lateral_delay);
        let mut model = model_wire::build(
            ModelFrame {
                prediction: output.prediction,
                timing: output.timing,
                action,
                raw_predictions: output.raw_predictions,
            },
            &mut self.publish,
        )?;
        let event = model.get_root_as_reader::<event::Reader>()?;
        let event::ModelV2(data) = event.which()? else {
            return Err(Error::Contract("model output event"));
        };
        let mut indicator = sources.indicator;
        let model_valid = event.get_valid();
        indicator.valid &= model_valid;
        indicator.assistant_valid &= model_valid;
        let input = wire::desire_input(
            sources.car,
            data?,
            sources.navigation,
            sources.radar,
            sources.lateral_active,
            indicator,
        )?;
        self.desire.update(&input, config, command)?;
        let event::ModelV2(data) = model.get_root::<event::Builder>()?.which()? else {
            return Err(Error::Contract("model output event"));
        };
        wire::apply_desire(data?, &self.desire)?;
        let driving = derived_wire::driving(
            model.get_root_as_reader::<event::Reader>()?,
            output.timing.log_mono_time,
        )?;
        let pose = derived_wire::pose(
            output.prediction,
            PoseTiming {
                log_mono_time: output.timing.log_mono_time,
                frame_id: output.timing.frame_id,
                dropped_frames: output.dropped,
                timestamp_eof: if output.simulation {
                    crate::clock::timestamp()?
                } else {
                    output.timing.timestamp_eof
                },
                live_calibration_seen: output.timing.valid,
            },
        )?;
        Ok(Messages {
            model,
            driving,
            pose,
        })
    }
}
