use super::{
    archive, assets, catalog::Catalog, download_fallible, packaged, sha256, DownloadKind, Error,
    Event,
};
use crate::model::Manifest;
use std::{
    fs,
    io::Write,
    path::{Path, PathBuf},
};

fn marker(root: &Path, model: &Manifest) -> Result<Option<Catalog>, Error> {
    let bytes = match fs::read(root.join("installed.json")) {
        Ok(bytes) => bytes,
        Err(error) if error.kind() == std::io::ErrorKind::NotFound => return Ok(None),
        Err(error) => return Err(error.into()),
    };
    let value: serde_json::Value = match serde_json::from_slice(&bytes) {
        Ok(value) => value,
        Err(_) => return Ok(None),
    };
    let Some(url) = value["catalog_url"].as_str() else {
        return Ok(None);
    };
    match Catalog::parse(&bytes, &model.sha256, url) {
        Ok(catalog) => Ok(Some(catalog)),
        Err(Error::Invalid(_) | Error::Json(_)) => Ok(None),
        Err(error) => Err(error),
    }
}

fn package(catalog: &Catalog, assets: &Path) -> Result<assets::Package, Error> {
    if !catalog.generic {
        return Err(Error::Invalid(
            "native comma-run-model companions/runtime adapter unavailable".into(),
        ));
    }
    let package = assets::validate(assets, &catalog.pickle.sha256, catalog.pickle.size)?;
    if catalog.value["model_checkpoint"] != package.checkpoint {
        return Err(Error::Invalid(
            "native model checkpoint differs from precompiled catalog".into(),
        ));
    }
    Ok(package)
}

pub fn catalog_url(model: &Manifest) -> Result<String, Error> {
    crate::model::resolve(&model.url, "precompiled.json")
        .map_err(|error| Error::Invalid(error.to_string()))
}

/// Check native readiness without treating a Python import as an execution provider.
pub fn installed(model: &Manifest, cache: &Path, assets: &Path) -> Result<Option<PathBuf>, Error> {
    let root = cache.join("precompiled").join(&model.sha256);
    let Some(catalog) = marker(&root, model)? else {
        // A native-package-only install would run the Rust native worker;
        // wait for carrot's artifacts instead unless that worker is forced.
        if native_worker_forced() {
            return packaged::installed(model, cache, assets);
        }
        return Ok(None);
    };
    if root.join("rejected").exists() {
        return Ok(None);
    }
    let path = root.join("model.pkl");
    if !path.is_file() || path.metadata()?.len() != catalog.pickle.size {
        return Ok(None);
    }
    let directory = format!("runtime-{}", &catalog.runtime.sha256[..16]);
    if catalog.value["runtime_directory"] != directory || !root.join(&directory).is_dir() {
        return Ok(None);
    }
    package(&catalog, assets)?;
    Ok(Some(path))
}

/// Install the original catalog's verified artifacts plus existing native companions.
/// Missing native support retains downloaded data and leaves the internal model selected.
pub fn ensure(
    catalog_agent: &ureq::Agent,
    artifact_agent: &ureq::Agent,
    model: &Manifest,
    cache: &Path,
    assets: &Path,
    progress: &mut impl FnMut(u64, u64),
) -> Result<Option<PathBuf>, Error> {
    ensure_fallible(
        catalog_agent,
        artifact_agent,
        model,
        cache,
        assets,
        &mut |done, total| {
            progress(done, total);
            Ok(())
        },
    )
}

pub fn ensure_fallible(
    catalog_agent: &ureq::Agent,
    artifact_agent: &ureq::Agent,
    model: &Manifest,
    cache: &Path,
    assets: &Path,
    progress: &mut impl FnMut(u64, u64) -> Result<(), Error>,
) -> Result<Option<PathBuf>, Error> {
    let native = native_worker_forced();
    if let Some(existing) = installed(model, cache, assets)? {
        let root = existing
            .parent()
            .ok_or_else(|| Error::Invalid("installed model has no directory".into()))?;
        match marker(root, model)? {
            Some(catalog) => {
                if sha256(&existing)? == catalog.pickle.sha256
                    && sha256(&root.join("runtime.tar.gz"))? == catalog.runtime.sha256
                {
                    return Ok(Some(existing));
                }
            }
            None if native => return packaged::ensure(model, cache, assets),
            // A native-package install has no carrot runtime: install carrot's
            // artifacts so its own worker runs the model (client::python_worker).
            None => {}
        }
    }
    if native && packaged::present(assets, model)? {
        return packaged::ensure(model, cache, assets);
    }
    ensure_original(
        catalog_agent,
        artifact_agent,
        model,
        cache,
        assets,
        progress,
    )
}

/// USBGPU_NATIVE_WORKER=1 keeps the Rust native eGPU runtime and its packages.
fn native_worker_forced() -> bool {
    std::env::var_os("USBGPU_NATIVE_WORKER").is_some_and(|value| value == "1")
}

/// carrot's precompiled.json artifacts: model.pkl, the pinned tinygrad runtime
/// and installed.json, exactly as carrot-wip installs them.
fn ensure_original(
    catalog_agent: &ureq::Agent,
    artifact_agent: &ureq::Agent,
    model: &Manifest,
    cache: &Path,
    assets: &Path,
    progress: &mut impl FnMut(u64, u64) -> Result<(), Error>,
) -> Result<Option<PathBuf>, Error> {
    let url = catalog_url(model)?;
    let catalog = Catalog::fetch(catalog_agent, &model.sha256, url.as_str())?;
    let root = cache.join("precompiled").join(&model.sha256);
    let rejected = root.join("rejected");
    if rejected.is_file() && fs::read_to_string(&rejected)? == catalog.pickle.sha256 {
        if !super::failure::retry_rejection(
            &root,
            &catalog.pickle.sha256,
            &super::failure::boot_identity(),
        ) {
            return Ok(None);
        }
        match fs::remove_file(root.join("boot_validation.json")) {
            Ok(()) => {}
            Err(error) if error.kind() == std::io::ErrorKind::NotFound => {}
            Err(error) => return Err(error.into()),
        }
    }
    fs::create_dir_all(&root)?;
    let path = root.join("model.pkl");
    let original = cache.join(model.cache_filename());
    if catalog.generic
        && !path.exists()
        && original.is_file()
        && original.metadata()?.len() == catalog.pickle.size
        && sha256(&original)? == model.sha256
    {
        if let Err(link_error) = fs::hard_link(&original, &path) {
            eprintln!(
                "optional model hard link unavailable; copying verified artifact: {link_error}"
            );
            fs::copy(&original, &path)?;
        }
    }
    for (artifact, target) in [
        (&catalog.pickle, path.clone()),
        (&catalog.runtime, root.join("runtime.tar.gz")),
    ] {
        let mut observe = |event| match event {
            Event::Progress { downloaded, total } if target == path => progress(downloaded, total),
            Event::Progress { .. } | Event::Verifying => Ok(()),
        };
        download_fallible(
            artifact_agent,
            &artifact.url,
            &target,
            artifact.size,
            &artifact.sha256,
            DownloadKind::Precompiled,
            &mut observe,
        )?;
    }
    let directory = format!("runtime-{}", &catalog.runtime.sha256[..16]);
    archive::install(&root.join("runtime.tar.gz"), &root.join(&directory))?;
    package(&catalog, assets)?;
    let mut value = catalog.value;
    value["runtime_directory"] = serde_json::json!(directory);
    value["catalog_url"] = serde_json::json!(url.as_str());
    let mut staging = tempfile::Builder::new()
        .prefix(".installed-")
        .tempfile_in(&root)?;
    serde_json::to_writer_pretty(&mut staging, &value)?;
    staging.flush()?;
    staging.as_file().sync_all()?;
    staging
        .persist(root.join("installed.json"))
        .map_err(|error| error.error)?;
    match fs::remove_file(rejected) {
        Ok(()) => {}
        Err(error) if error.kind() == std::io::ErrorKind::NotFound => {}
        Err(error) => return Err(error.into()),
    }
    Ok(Some(path))
}
