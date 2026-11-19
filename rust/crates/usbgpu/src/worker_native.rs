use crate::{
    hcq_gpu::{HcqBus, HcqGpu},
    hcq_model::Model,
    worker::{Info, Runtime},
    Error,
};

pub struct NativeRuntime<B: HcqBus> {
    pub model: Model<HcqGpu<B>>,
    last_map: Vec<u8>,
    #[cfg(feature = "native-skip-miri")]
    local: Option<crate::qcom_warp::LocalWarp>,
}
impl<B: HcqBus> NativeRuntime<B> {
    pub fn new(model: Model<HcqGpu<B>>) -> Self {
        Self {
            model,
            last_map: Vec::new(),
            #[cfg(feature = "native-skip-miri")]
            local: None,
        }
    }
    pub fn load_warp(&mut self, descriptor: &[u8]) -> Result<(), Error> {
        self.model
            .prepare_input("new_img", |gpu, output| gpu.load_warp(descriptor, output))
    }
    #[cfg(feature = "native-skip-miri")]
    pub fn enable_local(
        &mut self,
        mut local: crate::qcom_warp::LocalWarp,
        camera: [u32; 2],
    ) -> Result<(), Error> {
        use crate::{
            hcq_model::Device,
            warp_validation::{sampling_boundary_only, IMAGE_BYTES},
        };
        let [width, height] = camera.map(|v| v as usize);
        let frame_size = width.next_multiple_of(128)
            * (height.next_multiple_of(32) + (height / 2).next_multiple_of(16));
        let mut random = 0x12345678u32;
        let frames = (0..2 * frame_size)
            .map(|_| {
                random ^= random << 13;
                random ^= random >> 17;
                random ^= random << 5;
                random as u8
            })
            .collect::<Vec<_>>();
        let result = (|| {
            for (name, matrix) in [
                ("identity", [1., 0., 0., 0., 1., 0., 0., 0., 1.]),
                (
                    "projective",
                    [2.3, 0.01, 20.2, -0.02, 2.1, 40.3, 0.0001, -0.0002, 1.],
                ),
                ("border", [1., 0., -200., 0., 1., -100., 0., 0., 1.]),
            ] {
                let transforms = matrix
                    .into_iter()
                    .chain(matrix)
                    .flat_map(f32::to_le_bytes)
                    .collect::<Vec<_>>();
                let mut expected = vec![0; IMAGE_BYTES];
                self.model.prepare_input("new_img", |gpu, output| {
                    gpu.warp(&frames, &transforms)?;
                    gpu.read(output, &mut expected)
                })?;
                let actual = local.prepare(&frames, &transforms)?;
                if actual != expected {
                    let repeat = local.prepare(&frames, &transforms)?;
                    let stable = actual == repeat;
                    let explained = stable
                        && sampling_boundary_only(
                            &actual,
                            &expected,
                            &frames,
                            camera,
                            &transforms,
                        )?;
                    let count = actual.iter().zip(&expected).filter(|(a, b)| a != b).count();
                    let samples = actual
                        .iter()
                        .zip(&expected)
                        .enumerate()
                        .filter(|(_, (a, b))| a != b)
                        .take(8)
                        .map(|(index, (a, b))| format!("{index}:{a}/{b}"))
                        .collect::<Vec<_>>();
                    eprintln!(
                        "QCOM pre-upload probe={name} mismatches={count} repeat_stable={stable} boundary_only={explained} samples={samples:?}"
                    );
                    if !explained {
                        return Err(Error::Contract(
                            "QCOM pre-upload warp differs from artifact AMD warp",
                        ));
                    }
                }
            }
            Ok(())
        })();
        self.model.write_input("new_img", &vec![0; IMAGE_BYTES])?;
        result?;
        self.local = Some(local);
        Ok(())
    }
}
impl<B: HcqBus> Runtime for NativeRuntime<B> {
    fn reset(&mut self) -> Result<(), Error> {
        self.model.reset_recurrent()
    }

    fn run(&mut self, packed: &[u8], info: &Info, output: &mut [u8]) -> Result<(), Error> {
        let start = info
            .layout
            .get("img")
            .ok_or(Error::Contract("worker image view missing"))?
            .offset;
        let frames = packed
            .get(start..start + 2 * info.frame_size)
            .ok_or(Error::Contract("worker image range missing"))?;
        let transforms = packed
            .get(..72)
            .ok_or(Error::Contract("worker transforms missing"))?;
        crate::worker_progress::mark("input_upload");
        #[cfg(feature = "native-skip-miri")]
        if let Some(local) = &mut self.local {
            crate::worker_progress::mark("local_prepare");
            let images = local.prepare(frames, transforms)?;
            crate::worker_progress::mark("input_upload");
            self.model.write_input("new_img", &images)?;
        } else {
            crate::worker_progress::mark("warp");
            self.model
                .prepare_input("new_img", |gpu, _| gpu.warp(frames, transforms))?;
        }
        #[cfg(not(feature = "native-skip-miri"))]
        self.model
            .prepare_input("new_img", |gpu, _| gpu.warp(frames, transforms))?;
        for name in ["desire", "traffic_convention", "action_t"] {
            self.model.write_input(name, info.input(packed, name)?)?;
        }
        if info.layout.contains_key("navigation") {
            self.model
                .write_input("navigation", info.input(packed, "navigation")?)?;
            let map = info.input(packed, "nav_map")?;
            if self.last_map != map {
                self.model.write_input("nav_map", map)?;
                self.last_map.clear();
                self.last_map.extend_from_slice(map);
            }
        }
        crate::worker_progress::mark("model_call");
        self.model.run()?;
        crate::worker_progress::mark("output_read");
        if self.model.output_size("outputs")? != output.len() {
            return Err(Error::Contract("worker model output length mismatch"));
        }
        self.model.read_output("outputs", output)
    }
}
