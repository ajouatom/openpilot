//! CPU-default opt-in policy and bounded, same-input diagnostic summaries.
use crate::Error;
use openpilot_logmessaged::{JsonValue, JsonView};
use serde::Serialize;

#[derive(Clone, Copy, Debug, Default, PartialEq, Eq, Serialize)]
#[repr(u8)]
pub enum ComputeMode {
    #[default]
    Cpu = 0,
    Gpu = 1,
    Compare = 2,
    /// The Jetlink App's device (`openpilot_jetlink::aux`): the tablet's GPU
    /// runs the same ONNX model while the comma runs only pre/post-processing,
    /// and any frame the App cannot take runs on the comma's CPU.
    Tablet = 3,
}

impl ComputeMode {
    pub fn from_parameter(value: Option<&[u8]>) -> Self {
        match value
            .and_then(|value| std::str::from_utf8(value).ok())
            .map(str::trim)
        {
            Some("1") => Self::Gpu,
            Some("2") => Self::Compare,
            Some("3") => Self::Tablet,
            _ => Self::Cpu,
        }
    }
    pub fn from_json(value: &JsonValue) -> Result<Self, Error> {
        let number = match value.view() {
            JsonView::Integer(value) => value.parse::<f64>().ok(),
            JsonView::Float(value) => Some(value),
            _ => None,
        };
        match number {
            Some(0.0) => Ok(Self::Cpu),
            Some(1.0) => Ok(Self::Gpu),
            Some(2.0) => Ok(Self::Compare),
            Some(3.0) => Ok(Self::Tablet),
            _ => Err(Error::Invalid(
                "computeMode must be 0 (CPU), 1 (GPU), 2 (GPU comparison) or 3 (Jetlink tablet)",
            )),
        }
    }
    pub const fn value(self) -> u8 {
        self as u8
    }
}

#[derive(Default)]
pub struct Selection {
    pub requested: ComputeMode,
    gpu_failed: bool,
    generation: u64,
}
impl Selection {
    pub fn select(&mut self, mode: ComputeMode, generation: u64) -> bool {
        if mode == self.requested && generation == self.generation {
            return false;
        }
        self.requested = mode;
        self.generation = generation;
        self.gpu_failed = false;
        true
    }
    pub fn fail_gpu(&mut self) {
        self.gpu_failed = true;
    }
    /// The comma's own OpenCL GPU; the tablet mode keeps a CPU net locally.
    pub fn wants_gpu(&self) -> bool {
        matches!(self.requested, ComputeMode::Gpu | ComputeMode::Compare) && !self.gpu_failed
    }
    pub fn wants_tablet(&self) -> bool {
        self.requested == ComputeMode::Tablet
    }
}

#[derive(Default)]
pub struct ComparisonCadence {
    next: Option<f64>,
}
impl ComparisonCadence {
    pub fn due(&mut self, mode: ComputeMode, now: f64) -> bool {
        if mode != ComputeMode::Compare
            || !now.is_finite()
            || now < 0.0
            || self.next.is_some_and(|next| now < next)
        {
            return false;
        }
        self.next = Some(now + 10.0);
        true
    }
}

#[derive(Debug, Clone, Serialize)]
pub struct Difference {
    pub elements: u64,
    pub unequal_bits: u64,
    pub nonfinite: u64,
    pub max_abs: f64,
    pub mean_abs: f64,
    pub max_rel: f64,
}
impl Difference {
    pub fn between(cpu: &[f32], gpu: &[f32]) -> Result<Self, Error> {
        if cpu.len() != gpu.len() {
            return Err(Error::Invalid("comparison tensor lengths differ"));
        }
        let mut result = Self {
            elements: cpu.len() as u64,
            unequal_bits: 0,
            nonfinite: 0,
            max_abs: 0.0,
            mean_abs: 0.0,
            max_rel: 0.0,
        };
        let mut sum = 0.0;
        let mut finite = 0_u64;
        for (&cpu, &gpu) in cpu.iter().zip(gpu) {
            result.unequal_bits += u64::from(cpu.to_bits() != gpu.to_bits());
            if !cpu.is_finite() || !gpu.is_finite() {
                result.nonfinite += 1;
                continue;
            }
            let cpu = f64::from(cpu);
            let delta = (f64::from(gpu) - cpu).abs();
            result.max_abs = result.max_abs.max(delta);
            result.max_rel = result.max_rel.max(delta / cpu.abs().max(1e-12));
            sum += delta;
            finite += 1;
        }
        if finite != 0 {
            result.mean_abs = sum / finite as f64;
        }
        Ok(result)
    }
}
