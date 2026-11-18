use crate::{driver::driver_transform, Error};
use openpilot_model_runtime::{
    catalog::{Bundle, Kind},
    qcom::QcomModel,
    CpuModel,
};
use openpilot_modeld::{
    driver_wire::{self, DriverTiming},
    parse::RawOutputs,
    prediction::DriverPrediction,
};
use openpilot_msgq::{VisionFrame, VisionMetadata};
use std::time::Instant;

enum Backend {
    Cpu(CpuModel),
    Qcom(QcomModel),
}

impl Backend {
    fn write(&mut self, name: &str, bytes: &[u8]) -> Result<(), Error> {
        match self {
            Self::Cpu(model) => model.write_input(name, bytes)?,
            Self::Qcom(model) => model.write_input(name, bytes)?,
        }
        Ok(())
    }

    fn execute(&mut self, name: &str) -> Result<(), Error> {
        match self {
            Self::Cpu(model) => model.run_entry(name)?,
            Self::Qcom(model) => model.run_entry(name)?,
        }
        Ok(())
    }

    fn read(&self, destination: &mut [u8]) -> Result<(), Error> {
        match self {
            Self::Cpu(model) => model.read_output("model", destination)?,
            Self::Qcom(model) => {
                let source = model.read_output("model")?;
                if source.len() != destination.len() {
                    return Err(Error::Contract("driver output size changed"));
                }
                destination.copy_from_slice(source);
            }
        }
        Ok(())
    }
}

pub struct DriverRuntime<'a> {
    bundle: &'a Bundle,
    model: Backend,
    frame_bytes: Vec<u8>,
    bind_frame: bool,
    output_bytes: Vec<u8>,
    output_values: Vec<f32>,
    transform: Vec<u8>,
}

impl<'a> DriverRuntime<'a> {
    /// # Safety
    /// The catalog must contain trusted, immutable native kernels satisfying
    /// the CPU/QCOM model load contracts for the entire runtime lifetime.
    pub unsafe fn load(bundle: &'a Bundle, priority: u8) -> Result<Self, Error> {
        if bundle.descriptor.kind != Kind::Driver {
            return Err(Error::Contract("driver daemon requires a driver bundle"));
        }
        let transform = driver_transform(bundle.descriptor.camera)?
            .into_iter()
            .flat_map(f32::to_ne_bytes)
            .collect();
        let output = bundle
            .descriptor
            .outputs
            .iter()
            .find(|output| output.name == "model")
            .ok_or(Error::Contract("missing driver model output"))?
            .bytes()?;
        if output % 4 != 0 {
            return Err(Error::Contract("driver output must be float32"));
        }
        let model = match bundle.backend.as_str() {
            // SAFETY: the caller guarantees immutable trusted kernels and buffers.
            "cpu-clang" | "cpu-llvm" => Backend::Cpu(unsafe { CpuModel::load(&bundle.directory) }?),
            // SAFETY: the caller guarantees the QCOM kernel memory contract.
            "qcom-cl" => Backend::Qcom(unsafe { QcomModel::load(&bundle.directory, priority) }?),
            _ => return Err(Error::Contract("unsupported driver backend")),
        };
        Ok(Self {
            bundle,
            model,
            frame_bytes: Vec::new(),
            bind_frame: bundle.backend == "qcom-cl",
            output_bytes: vec![0; output],
            output_values: vec![0.0; output / 4],
            transform,
        })
    }

    pub fn validate_frame(&self, metadata: &VisionMetadata) -> Result<(), Error> {
        let descriptor = &self.bundle.descriptor;
        if [metadata.width, metadata.height] != descriptor.camera.map(|v| v as usize)
            || metadata.stride != descriptor.nv12.stride
            || metadata.uv_offset != descriptor.nv12.stride * descriptor.nv12.y_height
            || metadata.len != descriptor.nv12.bytes
        {
            return Err(Error::Contract(
                "camera layout does not match driver artifact",
            ));
        }
        Ok(())
    }

    /// # Safety
    /// The frame's VisionIPC generation must stay mapped until this runtime is dropped.
    pub unsafe fn infer(
        &mut self,
        frame: &VisionFrame<'_>,
        calibration: [f32; 3],
    ) -> Result<f32, Error> {
        self.validate_frame(frame.metadata())?;
        let calibration: Vec<u8> = calibration.into_iter().flat_map(f32::to_ne_bytes).collect();
        self.model.write("calib", &calibration)?;
        let gpu_start = Instant::now();
        let len = frame.metadata().len;
        let bound = match &mut self.model {
            // SAFETY: this VisionIPC generation stays mapped through inference.
            // A lost generation terminates the daemon before another inference.
            Backend::Qcom(model) if self.bind_frame => {
                self.bind_frame =
                    unsafe { model.bind_input("frame", frame.data_address(), len) }.is_ok();
                self.bind_frame
            }
            _ => false,
        };
        if !bound {
            self.frame_bytes.resize(len, 0);
            frame.copy_into(&mut self.frame_bytes)?;
            self.model.write("frame", &self.frame_bytes)?;
        }
        self.model.write("transform", &self.transform)?;
        self.model.execute("prepare")?;
        self.model.execute("model")?;
        self.model.read(&mut self.output_bytes)?;
        for (value, bytes) in self
            .output_values
            .iter_mut()
            .zip(self.output_bytes.chunks_exact(4))
        {
            *value = f32::from_ne_bytes([bytes[0], bytes[1], bytes[2], bytes[3]]);
        }
        Ok(gpu_start.elapsed().as_secs_f32())
    }

    pub fn message(&self, timing: DriverTiming, raw: bool) -> Result<Vec<u8>, Error> {
        let outputs = RawOutputs::new(
            &self.output_values,
            &self.bundle.descriptor.metadata.output_slices,
        )?;
        let prediction = DriverPrediction::parse(&outputs)?;
        Ok(driver_wire::encode(
            &prediction,
            timing,
            if raw { &self.output_bytes } else { &[] },
        ))
    }
}
