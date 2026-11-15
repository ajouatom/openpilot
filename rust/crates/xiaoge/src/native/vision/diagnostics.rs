//! Bounded structured records on the existing route log transport.
use super::super::{platform, shared::Shared};
use crate::service::Stream;
use openpilot_logging::{
    log_site,
    producer::{Factory, Logger},
    record::Record,
    Fields, Value as LogValue,
};
use serde_json::{json, Value};
use std::{
    fs,
    sync::{atomic::Ordering, Arc},
    thread,
    time::Duration,
};

pub fn log_fields(value: Value) -> Option<Fields> {
    match serde_json::from_value::<LogValue>(value).ok()? {
        LogValue::Object(fields) => Some(fields),
        _ => None,
    }
}
fn emit(logger: &mut Option<Logger>, event: &str, mut fields: Value) {
    if fields
        .get("error")
        .is_some_and(|error| error.as_str() == Some(""))
    {
        if let Some(fields) = fields.as_object_mut() {
            fields.remove("error");
        }
    }
    if let (Some(logger), Some(fields)) = (logger, log_fields(fields)) {
        if let Ok(record) = Record::event(event, Vec::new(), fields) {
            let _ = logger.emit(log_site!(), record);
        }
    }
}
fn logger() -> Option<Logger> {
    Factory::for_runtime().ok().map(|factory| factory.logger())
}

pub struct Diagnostics {
    logger: Option<Logger>,
    last: f64,
    count: u64,
}
impl Diagnostics {
    pub fn new() -> Self {
        Self {
            logger: logger(),
            last: f64::NEG_INFINITY,
            count: 0,
        }
    }
    pub fn backend(&mut self, stream: Stream, generation: u64, status: Value) {
        let now = platform::timestamp().unwrap_or(0);
        let mut fields = status;
        if let Some(fields) = fields.as_object_mut() {
            fields.insert("schema".into(), json!(1));
            fields.insert("mono_ns".into(), json!(now));
            fields.insert("stream".into(), json!(stream.name()));
            fields.insert("generation".into(), json!(generation));
        }
        emit(&mut self.logger, "xiaoge_backend", fields);
    }
    #[allow(clippy::too_many_arguments)]
    pub fn inference(
        &mut self,
        stream: Stream,
        frame_id: u32,
        sof: u64,
        generation: u64,
        latency_ms: f64,
        thread_cpu_ms: f64,
        status: Value,
        timing: Value,
        detection: Value,
    ) {
        self.count = self.count.saturating_add(1);
        let now = platform::monotonic().unwrap_or(0.0);
        let compared = timing
            .get("comparison")
            .is_some_and(|value| !value.is_null());
        if !compared && now - self.last < 1.0 {
            return;
        }
        self.last = now;
        let mut fields = timing;
        if let Some(fields) = fields.as_object_mut() {
            fields.insert("schema".into(), json!(1));
            fields.insert("mono_ns".into(), json!(platform::timestamp().unwrap_or(0)));
            fields.insert("stream".into(), json!(stream.name()));
            fields.insert("frame_id".into(), json!(frame_id));
            fields.insert("timestamp_sof".into(), json!(sof));
            fields.insert("generation".into(), json!(generation));
            fields.insert("mode".into(), status["mode"].clone());
            fields.insert(
                "effective_backend".into(),
                status["effective_backend"].clone(),
            );
            fields.insert("fallback_reason".into(), status["fallback_reason"].clone());
            fields.insert("backend_report".into(), status["backend_report"].clone());
            fields.insert("latency_ms".into(), json!(latency_ms));
            fields.insert("thread_cpu_ms".into(), json!(thread_cpu_ms));
            fields.insert("inference_count".into(), json!(self.count));
            fields.insert("sampled".into(), json!(true));
            fields.insert("detection".into(), detection);
        }
        emit(&mut self.logger, "xiaoge_inference", fields);
    }
}

// GPU activity/temperature already arrive in deviceState. Do not add another
// reader of KGSL gpubusy: its sysfs read can clear idle statistics used by hardwared.
fn rss_bytes() -> Option<u64> {
    let status = fs::read_to_string("/proc/self/status").ok()?;
    let mut fields = status
        .lines()
        .find(|line| line.starts_with("VmRSS:"))?
        .split_whitespace();
    fields.next()?;
    let kilobytes = fields.next()?.parse::<u64>().ok()?;
    if fields.next()? != "kB" {
        return None;
    }
    kilobytes.checked_mul(1024)
}
pub fn resources(shared: Arc<Shared>) {
    let mut logger = logger();
    let mut last: Option<(f64, f64)> = None;
    while shared.running.load(Ordering::Acquire) {
        let sample = (|| {
            let now = platform::monotonic().ok()?;
            let cpu = platform::process_cpu().ok()?;
            let (mode, generation, since, effective) = {
                let state = shared.state().ok()?;
                let backends: Vec<_> = state
                    .compute_status
                    .iter()
                    .filter_map(|status| match status.as_ref()?.get("effective_backend")? {
                        LogValue::Text(text) => Some(text.as_str()),
                        _ => None,
                    })
                    .collect();
                let backend = if state.compute_mode == crate::compute::ComputeMode::Cpu {
                    "cpu"
                } else if backends.len() == 2 && backends[0] == backends[1] {
                    backends[0]
                } else if backends.contains(&"opencl-fp32")
                    || backends.iter().any(|backend| backend.starts_with("tablet"))
                {
                    "mixed"
                } else {
                    "pending"
                };
                (
                    state.compute_mode.value(),
                    state.compute_generation,
                    state.compute_since_ns,
                    backend.to_owned(),
                )
            };
            let percent = last
                .filter(|(prior, _)| now > *prior)
                .map(|(prior, prior_cpu)| 100.0 * (cpu - prior_cpu) / (now - prior));
            last = Some((now, cpu));
            let usb = shared
                .params
                .as_ref()
                .and_then(|params| params.get("UsbGpuActive").ok().flatten())
                .map(|bytes| bytes == b"1");
            Some(json!({"schema":1,"mono_ns":platform::timestamp().ok()?,
                "mode":mode,"generation":generation,"mode_since_ns":since,"effective_backend":effective,
                "process_cpu_percent":percent,"cpu_percent_unit":"one_logical_core=100",
                "process_rss_bytes":rss_bytes(),
                "gpu_usage_source":"deviceState",
                "usb_gpu_active":usb,"sample_interval_seconds":1.0}))
        })();
        if let Some(sample) = sample {
            emit(&mut logger, "xiaoge_resources", sample);
        }
        thread::sleep(Duration::from_secs(1));
    }
}
