//! Optional precompiled boot preparation; failures preserve internal-model startup.
mod runner;
use super::{
    failure, precompiled, presence, smoke,
    status::{self, Phase, Values},
    validation, Error,
};
use crate::model::{self, Manifest, Paths};
use runner::Failed;
use std::{
    io::Write,
    path::{Path, PathBuf},
    sync::atomic::AtomicBool,
};

pub struct Config {
    pub paths: Paths,
    pub devices: PathBuf,
    pub identity_root: PathBuf,
    pub runner: PathBuf,
    pub worker: PathBuf,
}

fn write(config: &Config, model: &Manifest, phase: Phase, detail: &str) -> Result<(), Error> {
    status::write(
        &config.paths.cache,
        phase,
        Values {
            model: Some(model),
            downloaded: Some(model.size),
            detail: Some(detail),
        },
        smoke::wall_time()?,
        None,
    )?;
    Ok(())
}

fn installed_attempt(
    config: &Config,
    model: &Manifest,
    path: &Path,
    cancelled: &AtomicBool,
) -> Result<Option<Failed>, Error> {
    if presence::present(&config.devices) {
        let binding = crate::worker_artifact::bind(path, &config.paths.assets)?;
        let device = validation::Device::read(&config.identity_root);
        let cameras = device.as_ref().map_or_else(
            |_| validation::cameras(""),
            |device| validation::cameras(&device.device),
        );
        let key = device.and_then(|device| {
            validation::key(
                path,
                &device,
                &validation::Provider {
                    worker: &config.worker,
                    runner: &config.runner,
                    manifest_sha256: binding.manifest_sha256(),
                },
            )
        });
        let key = match key {
            Ok(key) => key,
            Err(error) => {
                eprintln!("Precompiled validation identity unavailable: {error}");
                None
            }
        };
        if validation::cached(path, key.as_deref()) {
            writeln!(
                std::io::stdout().lock(),
                "Reusing successful precompiled eGPU validation for this device/runtime"
            )?;
        } else {
            if let Some(failed) = runner::validate(config, path, &cameras, &binding, cancelled)? {
                return Ok(Some(failed));
            }
            match validation::save(path, key.as_deref()) {
                Ok(()) => {}
                Err(Error::Io(error)) => {
                    eprintln!("Could not cache successful precompiled validation: {error}");
                }
                Err(error) => return Err(error),
            }
        }
    }
    write(
        config,
        model,
        Phase::Compiled,
        if crate::local_artifact::is_local(path) {
            "converted local run_policy model"
        } else {
            "downloaded precompiled model"
        },
    )?;
    if crate::local_artifact::is_local(path) {
        writeln!(
            std::io::stdout().lock(),
            "Using converted native local eGPU model: {}",
            path.display()
        )?;
    } else {
        writeln!(
            std::io::stdout().lock(),
            "Using precompiled eGPU model without SCons compilation: {}",
            path.display()
        )?;
    }
    Ok(None)
}

pub fn prepare(config: &Config, cancelled: &AtomicBool) -> Result<bool, Error> {
    let Some(model) = model::selected_manifest(&config.paths) else {
        return Ok(false);
    };
    let installed = precompiled::ensure_fallible(
        &super::agent(8),
        &super::agent(30),
        &model,
        &config.paths.cache,
        &config.paths.assets,
        &mut |done, _total| {
            status::write(
                &config.paths.cache,
                Phase::Downloading,
                Values {
                    model: Some(&model),
                    downloaded: Some(done),
                    detail: Some("precompiled model"),
                },
                smoke::wall_time()?,
                None,
            )?;
            Ok(())
        },
    );
    let installed = match installed {
        Ok(path) => path,
        Err(error) => {
            eprintln!("Precompiled eGPU model unavailable: {error}");
            None
        }
    };
    let installed = installed.or_else(|| {
        if model.precompiled_only() {
            return None;
        }
        let path = model::local_compiled_path(&config.paths, &model);
        crate::local_artifact::Package::open(&path, &config.paths.assets, Some(&model.sha256))
            .ok()
            .map(|_| path)
    });
    if let Some(path) = installed {
        let failed = match installed_attempt(config, &model, &path, cancelled) {
            Ok(None) => return Ok(true),
            Ok(Some(failed)) => failed,
            Err(error @ Error::Native(crate::Error::Cancelled)) => return Err(error),
            Err(error) => Failed::from(error),
        };
        let rejected = failure::record_with_worker(
            &path,
            &failed.detail,
            failed.kind,
            "boot_validation",
            smoke::wall_time()?,
            failed.worker.as_ref(),
        )?;
        if !rejected {
            // Not verified: the check failed in a way that may pass later
            // (timeout, PCIe not up). The drive starts on the internal model
            // and retries the eGPU in the background.
            eprintln!(
                "Precompiled eGPU validation did not pass ({}); retrying while driving",
                failed.detail
            );
            write(
                config,
                &model,
                Phase::WaitingForIgnition,
                "precompiled model validation did not finish; retried in the background while driving",
            )?;
            return Ok(true);
        }
    }
    let detail = if model.precompiled_only() {
        "precompiled model unavailable; using internal model"
    } else {
        "converted native local run_policy package unavailable; using internal model"
    };
    write(config, &model, Phase::Error, detail)?;
    Ok(false)
}
