use crate::{publication::Messages, Error};
use openpilot_cereal::log_capnp::{event, jetlink_frame_status};
use openpilot_jetlink::{
    runtime::Status,
    transition::{Phase, Source},
};
use openpilot_params::Params;
pub fn raw_predictions(
    requested: bool,
    source: Source,
    native: &[u8],
) -> Result<Option<&[u8]>, Error> {
    if !requested {
        return Ok(None);
    }
    match source {
        Source::Native => Ok(Some(native)),
        Source::Jetlink => Err(Error::JetlinkRawPredictionsUnavailable),
    }
}
pub fn persist(
    params: &Params,
    stored: &mut Option<(bool, bool)>,
    status: &Status,
) -> Result<(), Error> {
    let state = (
        status.decision.source == Source::Jetlink,
        status.decision.loss_latched,
    );
    if *stored != Some(state) {
        params.put_bool("JetlinkActive", state.0)?;
        params.put_bool("JetlinkLossLatched", state.1)?;
        *stored = Some(state);
    }
    Ok(())
}
fn fill(mut builder: jetlink_frame_status::Builder<'_>, status: &Status) {
    builder.set_source(match status.decision.source {
        Source::Native => jetlink_frame_status::Source::Native,
        Source::Jetlink => jetlink_frame_status::Source::Jetlink,
    });
    builder.set_phase(match status.decision.phase {
        Phase::Off => "OFF",
        Phase::Preparing => "PREPARING",
        Phase::Ready => "READY",
        Phase::Active => "ACTIVE",
        Phase::Lost => "LOST",
    });
    builder.set_loss_latched(status.decision.loss_latched);
    builder.set_generation(&status.generation);
    builder.set_frame_id(status.frame);
    builder.set_execution_ms(status.execution_ms as f32);
    builder.set_validated(status.validated);
}
pub fn publication(messages: &mut Messages, status: &Status, fresh: bool) -> Result<(), Error> {
    for message in [
        &mut messages.model,
        &mut messages.driving,
        &mut messages.pose,
    ] {
        let mut root = message.get_root::<event::Builder>()?;
        if !fresh {
            root.set_valid(false);
        }
        match root.which()? {
            event::ModelV2(value) => fill(value?.init_jetlink(), status),
            event::DrivingModelData(value) => fill(value?.init_jetlink(), status),
            event::CameraOdometry(_) => {}
            _ => return Err(Error::Contract("Jetlink publication topic")),
        }
    }
    Ok(())
}

/// Keep display freshness independent of Params' change-only restart flags.
pub struct ModelStatusWriter {
    path: std::path::PathBuf,
    next: f64,
    previous: Option<(bool, String)>,
}
impl ModelStatusWriter {
    pub fn new(path: &std::path::Path) -> Self {
        Self {
            path: path.to_owned(),
            next: 0.0,
            previous: None,
        }
    }
    pub fn update(&mut self, status: &Status, detail: &str, now: f64) -> Result<(), Error> {
        self.update_selected(status, detail, now, openpilot_jetlink::contract::SHA256)
    }
    pub fn update_selected(
        &mut self,
        status: &Status,
        detail: &str,
        now: f64,
        sha256: &str,
    ) -> Result<(), Error> {
        let active = status.decision.source == Source::Jetlink;
        let error = if active {
            String::new()
        } else {
            detail.chars().take(240).collect()
        };
        let current = (active, error);
        if self.previous.as_ref() == Some(&current) && now < self.next {
            return Ok(());
        }
        openpilot_jetlink::status::write(
            &self.path,
            &serde_json::json!({
                "active":active,"updated":now,"error":current.1,"generation":status.generation,
            "ready":matches!(status.decision.phase,Phase::Ready|Phase::Active),"sha256":sha256,
            "model":openpilot_jetlink::status::model_name(sha256),
                "frame_id":status.frame,"validated":status.validated,
            }),
        )?;
        self.previous = Some(current);
        self.next = now + 1.0;
        Ok(())
    }
}
