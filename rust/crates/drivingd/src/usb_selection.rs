use crate::{recovery::Recovery, recovery_prepare::Preparation, Error};
use openpilot_params::Params;
use openpilot_usbgpu::{
    hardware,
    model::{self, Paths},
};
use std::{
    path::PathBuf,
    sync::{
        atomic::{AtomicBool, Ordering},
        mpsc::{self, Receiver},
        Arc,
    },
    thread,
    time::Duration,
};

#[derive(Clone)]
pub struct Configuration {
    pub paths: Paths,
    pub devices: PathBuf,
    pub worker: PathBuf,
    pub cancelled: Arc<AtomicBool>,
}
impl Configuration {
    pub fn for_runtime(cancelled: Arc<AtomicBool>) -> Result<Self, Error> {
        Ok(Self {
            paths: Paths::for_runtime(
                &std::env::current_dir()?,
                std::env::var_os("USBGPU_ASSETS_ROOT")
                    .map(PathBuf::from)
                    .map_or_else(
                        || std::env::current_exe().map(|path| path.with_file_name("usbgpu-assets")),
                        Ok,
                    )?,
            )?,
            devices: hardware::SYSFS.into(),
            worker: std::env::current_exe()?.with_file_name("openpilot-usbgpu-worker"),
            cancelled,
        })
    }
    pub fn present(&self) -> Result<bool, Error> {
        Ok(openpilot_usbgpu::model_delivery::presence::present(
            &self.devices,
        ))
    }
}

fn refresh_status(params: &Params, key: &str, value: bool) -> Result<(), Error> {
    let bytes = if value { b"1" } else { b"0" };
    // Read persisted bytes each time so external edits, deletion and failed writes
    // are retried, while unchanged status avoids durable Params synchronization.
    if params.get(key)?.as_deref() != Some(bytes.as_slice()) {
        params.put_bool(key, value)?;
    }
    Ok(())
}

/// Artifact admission hashes and parses the selected package. Keep both discovery
/// and periodic status refresh off the camera thread, including on first ignition.
pub fn monitor(
    config: Configuration,
    params: Params,
    camera: [u32; 2],
) -> Result<Receiver<Recovery>, Error> {
    clear_activity(&params)?;
    let (send, receive) = mpsc::channel();
    thread::Builder::new()
        .name("egpu-discovery".into())
        .spawn(move || {
            let run = || -> Result<(), Error> {
                if std::path::Path::new("/TICI").is_file() {
                    crate::recovery_task::background_scheduling()?;
                }
                let mut selected = false;
                while !config.cancelled.load(Ordering::Acquire) {
                    let present = config.present()?;
                    refresh_status(&params, "UsbGpuPresent", present)?;
                    if present {
                        refresh_status(&params, "UsbGpuHardwareSeen", true)?;
                    }
                    let path = model::compiled_path(&config.paths);
                    refresh_status(&params, "UsbGpuCompiled", path.is_some())?;
                    if present && !selected && !params.get_bool("UsbGpuStartupFailed")? {
                        if let Some(path) = path {
                            let preparation = Preparation::capture(
                                &path,
                                &config.worker,
                                &config.paths.assets,
                                camera,
                            )?;
                            let mut recovery = Recovery::new(preparation)?;
                            recovery.policy.startup();
                            if send.send(recovery).is_err() {
                                return Ok(());
                            }
                            selected = true;
                        }
                    }
                    thread::sleep(Duration::from_secs(1));
                }
                Ok(())
            };
            if let Err(error) = run() {
                eprintln!("eGPU discovery failed: {error}");
            }
        })?;
    Ok(receive)
}

fn clear_activity(params: &Params) -> Result<(), Error> {
    params.put_bool("UsbGpuActive", false)?;
    params.put_bool("UsbGpuLoading", false)?;
    Ok(())
}

pub fn fail(params: &Params) -> Result<(), Error> {
    clear_activity(params)?;
    params.put_bool("UsbGpuStartupFailed", true)?;
    Ok(())
}

pub fn published(params: &Params) -> Result<(), Error> {
    params.put_bool("UsbGpuActive", true)?;
    params.put_bool("UsbGpuLoading", false)?;
    Ok(())
}
