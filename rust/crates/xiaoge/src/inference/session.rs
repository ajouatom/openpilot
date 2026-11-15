//! Owner-thread DNN selection. CPU mode has no GPU initialization or shadow work.
use super::Error;
use crate::compute::{ComparisonCadence, ComputeMode, Difference, Selection};
use openpilot_opencv_runtime::{DnnNet, DnnReport, DnnTarget, Tensor, TensorView};
use serde::Serialize;
use serde_json::{json, Value};
use std::{
    path::{Path, PathBuf},
    sync::Mutex,
    time::Instant,
};

static GPU_WORK: Mutex<()> = Mutex::new(());

pub(super) fn cpu_seconds() -> f64 {
    let t = rustix::time::clock_gettime(rustix::time::ClockId::ThreadCPUTime);
    t.tv_sec as f64 + t.tv_nsec as f64 * 1e-9
}
fn target(target: DnnTarget) -> &'static str {
    match target {
        DnnTarget::Cpu => "cpu",
        DnnTarget::OpenClFp32 => "opencl-fp32",
    }
}
fn report_value(report: DnnReport) -> Value {
    json!({"requested_target":target(report.requested_target),
        "effective_target":report.effective_target.map(target), "device":report.device,
        "vendor":report.vendor,"driver":report.driver,"priority_status":report.priority_status,
        "graph_gpu_layers":report.graph_gpu_layers,"graph_cpu_layers":report.graph_cpu_layers,
        "execution_status":report.execution_status})
}

#[derive(Clone, Default, Serialize)]
pub struct Comparison {
    pub shape_equal: bool,
    pub elements: u64,
    pub unequal_bits: u64,
    pub nonfinite: u64,
    pub max_abs: f64,
    pub mean_abs: f64,
    pub max_rel: f64,
    pub decoded_equal: Option<bool>,
    pub decision_equal: Option<bool>,
    pub decoded_cpu: Option<Value>,
    pub decoded_gpu: Option<Value>,
    pub error: Option<String>,
    pub outputs: Vec<Value>,
}
impl Comparison {
    fn tensors(cpu: &[Tensor], gpu: &[Tensor], names: &[String]) -> Self {
        let mut result = Self {
            shape_equal: cpu.len() == gpu.len(),
            ..Self::default()
        };
        let mut total = 0.0;
        let mut finite = 0;
        for (index, (cpu, gpu)) in cpu.iter().zip(gpu).enumerate() {
            let cpu = cpu.view();
            let gpu = gpu.view();
            if cpu.shape() != gpu.shape() {
                result.shape_equal = false;
                continue;
            }
            let Ok(diff) = Difference::between(cpu.values(), gpu.values()) else {
                result.shape_equal = false;
                continue;
            };
            result.outputs.push(
                json!({"name":names.get(index).map_or("default",String::as_str),
                "shape":cpu.shape(),"difference":diff}),
            );
            result.elements += diff.elements;
            result.unequal_bits += diff.unequal_bits;
            result.nonfinite += diff.nonfinite;
            result.max_abs = result.max_abs.max(diff.max_abs);
            result.max_rel = result.max_rel.max(diff.max_rel);
            let count = diff.elements - diff.nonfinite;
            total += diff.mean_abs * count as f64;
            finite += count;
        }
        if finite != 0 {
            result.mean_abs = total / finite as f64;
        }
        result
    }
}

#[derive(Default, Serialize)]
pub struct ForwardTiming {
    pub queue_wait_ms: f64,
    pub primary_ms: f64,
    pub primary_thread_cpu_ms: f64,
    pub compare_wall_ms: f64,
    pub compare_thread_cpu_ms: f64,
    pub comparison: Option<Comparison>,
}

pub struct Session {
    path: PathBuf,
    net: Option<DnnNet>,
    shadow: Option<DnnNet>,
    selection: Selection,
    comparison_cadence: ComparisonCadence,
    started: Instant,
    report_at: Option<Instant>,
    backend_report: Value,
    pub timing: ForwardTiming,
    shadow_outputs: Option<Vec<Tensor>>,
    error: String,
    fallback_reason: String,
    initialization_ms: f64,
    /// `OnnxComputeMode` 3: the Jetlink App runs the model; `net` stays on
    /// the CPU for every frame the App cannot take.
    remote: super::remote::Remote,
}
impl Session {
    pub fn load(path: &Path) -> Result<Self, Error> {
        let net = DnnNet::load(path)?;
        Ok(Self {
            path: path.to_owned(),
            net: Some(net),
            shadow: None,
            selection: Selection::default(),
            comparison_cadence: ComparisonCadence::default(),
            started: Instant::now(),
            report_at: None,
            backend_report: json!({"execution_status":"not_run"}),
            timing: ForwardTiming::default(),
            shadow_outputs: None,
            error: String::new(),
            fallback_reason: String::new(),
            initialization_ms: 0.0,
            remote: super::remote::Remote::default(),
        })
    }
    pub fn loaded(&self) -> bool {
        self.net.is_some()
    }
    pub fn error(&self) -> &str {
        &self.error
    }
    pub fn set_mode(&mut self, mode: ComputeMode, generation: u64) -> bool {
        let had_gpu = self.selection.wants_gpu();
        if !self.selection.select(mode, generation) {
            return false;
        }
        let started = Instant::now();
        // Loading/freeing a GPU net also belongs to the single Xiaoge GPU owner gate.
        let needs_gate = matches!(mode, ComputeMode::Gpu | ComputeMode::Compare) || had_gpu;
        let lock = needs_gate.then(|| GPU_WORK.lock());
        self.shadow = None;
        self.shadow_outputs = None;
        self.timing = ForwardTiming::default();
        self.comparison_cadence = ComparisonCadence::default();
        self.report_at = None;
        self.backend_report = json!({"execution_status":"not_run"});
        self.fallback_reason.clear();
        self.error.clear();
        self.remote = super::remote::Remote::default();
        let requested = if self.selection.wants_gpu() {
            DnnTarget::OpenClFp32
        } else {
            DnnTarget::Cpu
        };
        let loaded = if lock.as_ref().is_some_and(|lock| lock.is_err()) {
            Err(openpilot_opencv_runtime::Error::Contract(
                "GPU owner gate poisoned",
            ))
        } else {
            DnnNet::load_with_target(&self.path, requested)
        };
        match loaded {
            Ok(net) => {
                self.net = Some(net);
                self.refresh_report();
            }
            Err(error) if self.selection.wants_gpu() => self.fallback(error.to_string()),
            Err(error) => {
                self.net = None;
                self.error = error.to_string();
            }
        }
        self.initialization_ms = started.elapsed().as_secs_f64() * 1000.0;
        true
    }
    fn fallback(&mut self, reason: String) {
        self.selection.fail_gpu();
        self.fallback_reason = reason;
        self.shadow = None;
        self.shadow_outputs = None;
        match DnnNet::load(&self.path) {
            Ok(net) => {
                self.net = Some(net);
                self.error.clear();
                self.refresh_report();
            }
            Err(error) => {
                self.net = None;
                self.error = error.to_string();
            }
        }
    }
    fn refresh_report(&mut self) {
        if let Some(net) = &self.net {
            self.backend_report = match net.report() {
                Ok(report) => report_value(report),
                Err(error) => json!({"execution_status":"report_failed","error":error.to_string()}),
            };
            self.report_at = Some(Instant::now());
        }
    }
    pub fn status(&self) -> Value {
        let effective = if self.selection.wants_gpu() {
            self.backend_report["effective_target"]
                .as_str()
                .unwrap_or("pending")
                .to_owned()
        } else if self.selection.wants_tablet() && self.remote.last_error.is_empty() {
            self.remote
                .device()
                .map_or_else(|| "tablet".to_owned(), |device| format!("tablet {device}"))
        } else {
            "cpu".to_owned()
        };
        let mut status = json!({"mode":self.selection.requested.value(),
            "effective_backend":effective,
            "fallback_reason":self.fallback_reason,"loaded":self.loaded(),"error":self.error,
            "initialization_ms":self.initialization_ms,"backend_report":self.backend_report});
        if self.selection.wants_tablet() {
            status["tablet"] = self.remote.status();
        }
        status
    }
    pub fn begin_frame(&mut self) {
        self.timing = ForwardTiming::default();
        self.shadow_outputs = None;
    }
    pub fn output_names(&self) -> Result<Vec<String>, Error> {
        Ok(self
            .net
            .as_ref()
            .ok_or(Error::Contract("model not loaded"))?
            .output_names()?)
    }
    pub fn forward(
        &mut self,
        input: TensorView<'_>,
        names: &[String],
    ) -> Result<Vec<Tensor>, Error> {
        self.timing = ForwardTiming::default();
        self.shadow_outputs = None;
        let queue_started = Instant::now();
        let wants_gpu = self.selection.wants_gpu();
        let lock = wants_gpu.then(|| GPU_WORK.lock());
        self.timing.queue_wait_ms = if wants_gpu {
            queue_started.elapsed().as_secs_f64() * 1000.0
        } else {
            0.0
        };
        let started = Instant::now();
        let cpu = cpu_seconds();
        if lock.as_ref().is_some_and(|lock| lock.is_err()) {
            self.fallback("GPU owner gate poisoned".into());
        }
        let result = self.forward_primary(input, names);
        self.timing.primary_ms = started.elapsed().as_secs_f64() * 1000.0;
        self.timing.primary_thread_cpu_ms = (cpu_seconds() - cpu) * 1000.0;
        let outputs = match result {
            Ok(outputs) => outputs,
            Err(error) => {
                self.refresh_report();
                return Err(error);
            }
        };
        // Release the GPU gate before optional CPU diagnostic work.
        drop(lock);
        if self.selection.wants_gpu()
            && self.comparison_cadence.due(
                self.selection.requested,
                self.started.elapsed().as_secs_f64(),
            )
        {
            let started = Instant::now();
            let cpu = cpu_seconds();
            let shadow = (|| -> Result<Vec<Tensor>, Error> {
                if self.shadow.is_none() {
                    self.shadow = Some(DnnNet::load(&self.path)?);
                }
                Ok(self
                    .shadow
                    .as_mut()
                    .ok_or(Error::Contract("comparison model missing"))?
                    .forward(input, names)?)
            })();
            match shadow {
                Ok(shadow) => {
                    self.timing.comparison = Some(Comparison::tensors(&shadow, &outputs, names));
                    self.shadow_outputs = Some(shadow);
                }
                Err(error) => {
                    self.timing.comparison = Some(Comparison {
                        error: Some(error.to_string()),
                        ..Comparison::default()
                    })
                }
            }
            self.timing.compare_wall_ms = started.elapsed().as_secs_f64() * 1000.0;
            self.timing.compare_thread_cpu_ms = (cpu_seconds() - cpu) * 1000.0;
        }
        Ok(outputs)
    }
    fn forward_primary(
        &mut self,
        input: TensorView<'_>,
        names: &[String],
    ) -> Result<Vec<Tensor>, Error> {
        if self.selection.wants_tablet() {
            match self.remote.run(&self.path, input, names) {
                Ok(outputs) => {
                    self.fallback_reason.clear();
                    return Ok(outputs);
                }
                // This frame runs on the comma's CPU, as before the tablet.
                Err(reason) => self.fallback_reason = reason,
            }
        }
        let first = self
            .net
            .as_mut()
            .ok_or(Error::Contract("model not loaded"))?
            .forward(input, names);
        let mut outputs = match first {
            Ok(outputs) => outputs,
            Err(error) if self.selection.wants_gpu() => {
                self.fallback(format!("GPU forward failed: {error}"));
                self.net
                    .as_mut()
                    .ok_or(Error::Contract("CPU fallback model not loaded"))?
                    .forward(input, names)?
            }
            Err(error) => return Err(error.into()),
        };
        if self
            .report_at
            .is_none_or(|at| at.elapsed().as_secs_f64() >= 1.0)
            || self.backend_report["effective_target"].is_null()
        {
            self.refresh_report();
        }
        if self.selection.wants_gpu() && self.backend_report["effective_target"] != "opencl-fp32" {
            self.fallback("OpenCV selected CPU after the GPU request".into());
            outputs = self
                .net
                .as_mut()
                .ok_or(Error::Contract("CPU fallback model not loaded"))?
                .forward(input, names)?;
            self.refresh_report();
        }
        Ok(outputs)
    }
    pub fn take_shadow(&mut self) -> Option<Vec<Tensor>> {
        self.shadow_outputs.take()
    }
    pub fn decoded(&mut self, cpu: Value, gpu: Value, equal: bool) {
        if let Some(comparison) = &mut self.timing.comparison {
            comparison.decoded_equal = Some(cpu == gpu);
            comparison.decision_equal = Some(equal);
            comparison.decoded_cpu = Some(cpu);
            comparison.decoded_gpu = Some(gpu);
        }
    }
    pub fn add_comparison_cost(&mut self, wall_ms: f64, thread_ms: f64) {
        self.timing.compare_wall_ms += wall_ms;
        self.timing.compare_thread_cpu_ms += thread_ms;
    }
    pub fn comparison_error(&mut self, error: String) {
        if let Some(comparison) = &mut self.timing.comparison {
            comparison.error = Some(error);
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    #[test]
    fn failed_forward_clears_previous_success_evidence_and_keeps_its_elapsed_cost() {
        openpilot_opencv_runtime::initialize(2).unwrap();
        let path = std::env::var("OPENCV_TEST_LANE_MODEL").expect("pinned model path");
        let mut net = Session::load(Path::new(&path)).unwrap();
        let names = net.output_names().unwrap();
        let values = vec![0.5; 3 * 416 * 416];
        let shape = [1, 3, 416, 416];
        let input = TensorView::new(&shape, &values).unwrap();
        net.forward(input, &names).unwrap();
        assert_eq!(net.status()["backend_report"]["execution_status"], "cpu");
        assert!(net.forward(input, &["not-an-output".into()]).is_err());
        assert_eq!(
            net.status()["backend_report"]["execution_status"],
            "forward_failed"
        );
        assert!(net.status()["backend_report"]["effective_target"].is_null());
        assert!(net.timing.primary_ms > 0.0);
        assert!(net.timing.comparison.is_none());
        net.forward(input, &names).unwrap();
        assert_eq!(net.status()["backend_report"]["execution_status"], "cpu");
        assert_eq!(net.status()["backend_report"]["effective_target"], "cpu");
    }
}
