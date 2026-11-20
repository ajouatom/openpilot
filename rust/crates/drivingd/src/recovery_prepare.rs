//! Source-matched AMD-only preparation, called solely by the recovery worker.
use crate::{
    usb_model::{Controls, UsbModel},
    Error,
};
use openpilot_usbgpu::{client::Launch, worker_artifact};
use std::{
    fs,
    path::{Path, PathBuf},
    sync::{
        atomic::{AtomicBool, Ordering},
        Arc,
    },
    thread,
    time::{Duration, Instant},
};

#[derive(Clone)]
pub struct Preparation {
    pub model: PathBuf,
    pub worker: PathBuf,
    pub assets: PathBuf,
    pub camera: [u32; 2],
    pub identity_file: PathBuf,
    pub identity: serde_json::Value,
    pub asset_manifest_sha256: String,
}
impl Preparation {
    pub fn capture(
        model: &Path,
        worker: &Path,
        assets: &Path,
        camera: [u32; 2],
    ) -> Result<Self, Error> {
        let parent = model
            .parent()
            .ok_or(Error::Contract("recovery artifact has no parent"))?;
        let identity_file = if openpilot_usbgpu::local_artifact::is_local(model) {
            assets
                .join("local")
                .join(
                    model
                        .file_name()
                        .ok_or(Error::Contract("local model filename missing"))?,
                )
                .join("current.json")
        } else if parent.join("installed.json").exists() {
            parent.join("installed.json")
        } else {
            parent.join("native-provider.json")
        };
        let identity = serde_json::from_slice(&fs::read(&identity_file)?)
            .map_err(openpilot_usbgpu::Error::from)?;
        let binding = worker_artifact::bind(model, assets).map_err(|error| {
            openpilot_usbgpu::Error::Protocol(format!("native model assets: {error}"))
        })?;
        Ok(Self {
            model: model.to_owned(),
            worker: worker.to_owned(),
            assets: assets.to_owned(),
            camera,
            identity_file,
            identity,
            asset_manifest_sha256: binding.manifest_sha256().into(),
        })
    }
    /// `startup`: the drive's first eGPU load. carrot loads it at modeld start
    /// with no memory floor; the floor below is carrot's rule for background
    /// recovery only (egpu_recovery.py). Applying it at startup refused Light
    /// Speed 2 on the IONIQ 9 (comma four) on every drive.
    pub fn prepare(self, cancelled: Arc<AtomicBool>, startup: bool) -> Result<UsbModel, String> {
        self.prepare_inner(cancelled, startup)
            .map_err(|error| error.to_string())
    }
    fn prepare_inner(self, cancelled: Arc<AtomicBool>, startup: bool) -> Result<UsbModel, Error> {
        let identity: serde_json::Value = serde_json::from_slice(&fs::read(&self.identity_file)?)
            .map_err(openpilot_usbgpu::Error::from)?;
        if identity != self.identity
            || self
                .model
                .parent()
                .is_none_or(|parent| parent.join("rejected").exists())
        {
            return Err(Error::Contract(
                "eGPU recovery artifact changed or rejected",
            ));
        }
        let available = mem_available(&fs::read_to_string("/proc/meminfo")?)
            .ok_or(Error::Contract("recovery MemAvailable missing"))?;
        if !startup && available < memory_floor(self.model.metadata()?.len()) {
            return Err(Error::Contract(
                "insufficient memory for background eGPU recovery",
            ));
        }
        let binding = worker_artifact::bind(&self.model, &self.assets).map_err(|error| {
            openpilot_usbgpu::Error::Protocol(format!("native model assets: {error}"))
        })?;
        if binding.manifest_sha256() != self.asset_manifest_sha256 {
            return Err(Error::Contract("eGPU recovery asset manifest changed"));
        }
        let mut model = UsbModel::launch_recovery_with_assets(
            Launch {
                worker: &self.worker,
                model: &self.model,
                camera: self.camera,
                timeout: openpilot_usbgpu::client::LOAD_INACTIVITY,
                cancelled: Arc::clone(&cancelled),
            },
            &binding,
        )?;
        let frames = vec![0; model.client.info.frame_size];
        let identity = [1., 0., 0., 0., 1., 0., 0., 0., 1.];
        for index in 0..5 {
            if cancelled.load(Ordering::Acquire) {
                return Err(openpilot_usbgpu::Error::Cancelled.into());
            }
            model.update(Controls {
                desire: 0,
                is_rhd: false,
                lateral_time: 0.2,
                longitudinal_time: 0.2,
            });
            let started = Instant::now();
            model.infer(&frames, &frames, [identity, identity])?;
            if index >= 2 && started.elapsed() > Duration::from_millis(50) {
                return Err(Error::Contract("eGPU recovery probe was missing or late"));
            }
            let delay = Instant::now();
            while delay.elapsed() < Duration::from_millis(50) {
                if cancelled.load(Ordering::Acquire) {
                    return Err(openpilot_usbgpu::Error::Cancelled.into());
                }
                thread::sleep(Duration::from_millis(5));
            }
        }
        model.reset_after_probe()?;
        Ok(model)
    }
}
#[must_use]
pub fn memory_floor(model_bytes: u64) -> u64 {
    (2u64 << 30).max(model_bytes.saturating_mul(2).saturating_add(512 << 20))
}
#[must_use]
pub fn mem_available(value: &str) -> Option<u64> {
    let mut matches = value
        .lines()
        .filter_map(|line| line.strip_prefix("MemAvailable:"));
    let mut fields = matches.next()?.split_whitespace();
    let count = fields.next()?.parse::<u64>().ok()?;
    if fields.next() != Some("kB") || fields.next().is_some() || matches.next().is_some() {
        return None;
    }
    count.checked_mul(1024)
}
