use crate::Error;
use openpilot_modeld::{inputs::PolicyInputs, parse::RawOutputs, prediction::DrivingPrediction};
use openpilot_usbgpu::client::{Client, Frame, Launch};
use std::{
    collections::BTreeMap,
    path::{Path, PathBuf},
};

#[derive(Clone, Copy, Default)]
pub struct Controls {
    pub desire: i32,
    pub is_rhd: bool,
    pub lateral_time: f64,
    pub longitudinal_time: f64,
}
impl Controls {
    pub fn apply(self, inputs: &mut PolicyInputs) {
        inputs.update(
            self.desire,
            self.is_rhd,
            self.lateral_time,
            self.longitudinal_time,
        );
    }
}

pub struct UsbModel {
    pub client: Client,
    pub controls: Controls,
    last_map_render: Option<u64>,
    inputs: PolicyInputs,
    artifact: Option<PathBuf>,
    slices: BTreeMap<String, [usize; 2]>,
}
impl UsbModel {
    pub fn load(worker: &Path, model: &Path, camera: [u32; 2]) -> Result<Self, Error> {
        let client = Client::load(worker, model, camera)?;
        let mut result = Self::from_client(client)?;
        result.artifact = Some(model.to_owned());
        Ok(result)
    }
    pub fn launch(launch: Launch<'_>) -> Result<Self, Error> {
        let artifact = launch.model.to_owned();
        let mut result = Self::from_client(Client::launch(launch)?)?;
        result.artifact = Some(artifact);
        Ok(result)
    }
    pub fn launch_with_assets(
        launch: Launch<'_>,
        binding: &openpilot_usbgpu::worker_artifact::Binding,
    ) -> Result<Self, Error> {
        let artifact = launch.model.to_owned();
        let mut result = Self::from_client(Client::launch_with_assets(launch, binding)?)?;
        result.artifact = Some(artifact);
        Ok(result)
    }
    pub fn launch_recovery_with_assets(
        launch: Launch<'_>,
        binding: &openpilot_usbgpu::worker_artifact::Binding,
    ) -> Result<Self, Error> {
        let artifact = launch.model.to_owned();
        let mut result = Self::from_client(Client::launch_recovery_with_assets(launch, binding)?)?;
        result.artifact = Some(artifact);
        Ok(result)
    }
    pub fn reset_after_probe(&mut self) -> Result<(), Error> {
        self.client.reset()?;
        self.inputs = PolicyInputs::new(
            self.slices
                .get("hidden_state")
                .map_or(1, |section| section[1] - section[0]),
        )?;
        self.controls = Controls::default();
        Ok(())
    }
    pub fn from_client(client: Client) -> Result<Self, Error> {
        let mut slices = BTreeMap::new();
        for (name, section) in &client.info.output_slices {
            let [Some(start), Some(stop), None | Some(1)] = section else {
                return Err(Error::Contract("unsupported USB model output slice"));
            };
            let stop = if name == "action" && stop - start == 4 {
                start + 2
            } else {
                *stop
            };
            slices.insert(name.clone(), [*start, stop]);
        }
        Ok(Self {
            client,
            controls: Controls::default(),
            last_map_render: None,
            artifact: None,
            inputs: PolicyInputs::new(
                slices
                    .get("hidden_state")
                    .map_or(1, |section| section[1] - section[0]),
            )?,
            slices,
        })
    }
    pub fn update(&mut self, controls: Controls) {
        self.controls = controls;
        controls.apply(&mut self.inputs);
    }

    pub fn set_navigation(&mut self, navigation: &[f32; 47]) -> Result<(), Error> {
        let map = openpilot_mapsd::output::changed_image(
            &mut self.last_map_render,
            openpilot_jetlink::now_ns(),
        );
        self.client.set_navigation(navigation, map.as_deref())?;
        Ok(())
    }
    pub fn infer(
        &mut self,
        main: &[u8],
        extra: &[u8],
        transforms: [[f32; 9]; 2],
    ) -> Result<DrivingPrediction, Error> {
        self.infer_with_prepare([main, extra], transforms, false)?
            .ok_or(Error::Contract("USB model omitted inference output"))
    }
    pub fn infer_with_prepare(
        &mut self,
        images: [&[u8]; 2],
        transforms: [[f32; 9]; 2],
        prepare_only: bool,
    ) -> Result<Option<DrivingPrediction>, Error> {
        let [main, extra] = images;
        let bytes = self.client.info.frame_size;
        let main = main
            .get(..bytes)
            .ok_or(Error::Contract("USB main NV12 planes are truncated"))?;
        let extra = extra
            .get(..bytes)
            .ok_or(Error::Contract("USB extra NV12 planes are truncated"))?;
        let values = self.client.run_with_prepare(
            Frame {
                main,
                extra,
                transforms,
                inputs: self.inputs.packed(),
            },
            prepare_only,
        );
        let values = match values {
            Ok(values) => values,
            Err(error) => {
                if let Some(path) = self
                    .artifact
                    .as_ref()
                    .filter(|_| !matches!(error, openpilot_usbgpu::Error::Cancelled))
                {
                    use openpilot_usbgpu::model_delivery::{failure, smoke};
                    if let Err(record_error) = smoke::wall_time()
                        .and_then(|time| failure::record_error(path, &error, "inference", time))
                    {
                        eprintln!("could not persist eGPU failure: {record_error}");
                    }
                }
                return Err(error.into());
            }
        };
        let Some(values) = values else {
            return Ok(None);
        };
        Ok(Some(DrivingPrediction::parse(&RawOutputs::new(
            values,
            &self.slices,
        )?)?))
    }
}
