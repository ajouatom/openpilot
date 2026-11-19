//! Validate the existing runtime package, keyed by the verified pickle hash.
use super::{sha256, Error};
use crate::{
    amd_metadata::Catalog,
    worker::{Info, Metadata},
};
use serde::Deserialize;
use sha2::{Digest, Sha256};
use std::{
    collections::BTreeMap,
    fs,
    path::{Component, Path, PathBuf},
};

#[derive(Deserialize)]
struct PackageManifest {
    version: u32,
    model_sha256: String,
    files: Vec<Entry>,
}
#[derive(Deserialize)]
struct Entry {
    path: String,
    bytes: u64,
    sha256: String,
}

pub struct Package {
    pub root: PathBuf,
    pub descriptor: PathBuf,
    pub metadata: PathBuf,
    pub checkpoint: String,
    pub manifest_sha256: String,
}

pub(super) fn regular(root: &Path, relative: &str) -> Result<PathBuf, Error> {
    let mut path = root.to_path_buf();
    let components = Path::new(relative).components().collect::<Vec<_>>();
    if components.is_empty()
        || components
            .iter()
            .any(|item| !matches!(item, Component::Normal(_)))
    {
        return Err(Error::Invalid("native asset escapes its package".into()));
    }
    for (index, component) in components.into_iter().enumerate() {
        path.push(component.as_os_str());
        let metadata = path.symlink_metadata()?;
        if metadata.file_type().is_symlink()
            || (index + 1 == Path::new(relative).components().count() && !metadata.is_file())
        {
            return Err(Error::Invalid(
                "native asset is not a regular packaged file".into(),
            ));
        }
    }
    Ok(path)
}

/// Validate required companions and every declared packaged asset before readiness.
///
/// # Errors
/// Missing, corrupt, out-of-package or mismatched artifacts remain unavailable.
/// The manifest is a trusted package integrity record, not a signature authority.
pub fn validate(root: &Path, digest: &str, model_size: u64) -> Result<Package, Error> {
    if digest.len() != 64 || !digest.bytes().all(|b| b.is_ascii_hexdigit()) {
        return Err(Error::Invalid("invalid native model digest".into()));
    }
    let manifest_file = match regular(root, &format!("models/{digest}/manifest.json")) {
        Err(Error::Io(e)) if e.kind() == std::io::ErrorKind::NotFound => {
            regular(root, "manifest.json")?
        }
        result => result?,
    };
    if manifest_file.metadata()?.len() > 1024 * 1024 {
        return Err(Error::Invalid("native asset manifest too large".into()));
    }
    let manifest_bytes = fs::read(manifest_file)?;
    let manifest_sha256 = format!("{:x}", Sha256::digest(&manifest_bytes));
    let manifest: PackageManifest = serde_json::from_slice(&manifest_bytes)?;
    if manifest.version != 1 || manifest.model_sha256 != digest || manifest.files.len() > 512 {
        return Err(Error::Invalid(
            "native asset package identity mismatch".into(),
        ));
    }
    let mut entries = BTreeMap::new();
    let mut total = 0u64;
    for entry in manifest.files {
        total = total
            .checked_add(entry.bytes)
            .ok_or_else(|| Error::Invalid("native asset size overflow".into()))?;
        if entry.bytes == 0 || total > 128 << 20 || entries.contains_key(&entry.path) {
            return Err(Error::Invalid("native asset package limits".into()));
        }
        let path = regular(root, &entry.path)?;
        if path.metadata()?.len() != entry.bytes || sha256(&path)? != entry.sha256 {
            return Err(Error::Invalid(format!(
                "native asset checksum mismatch: {}",
                entry.path
            )));
        }
        entries.insert(entry.path, path);
    }
    let required = |name: &str| {
        entries
            .get(name)
            .cloned()
            .ok_or_else(|| Error::Invalid(format!("missing native companion: {name}")))
    };
    let descriptor = required(&format!("models/{digest}/model.hcq.json"))?;
    let metadata = required(&format!("models/{digest}/model.hcq-meta.json"))?;
    required("tinygrad-LICENSE")?;
    crate::probe::validate_descriptor(&fs::read(required("probe-gfx1200.json")?)?, "gfx1200")?;
    let catalog = Catalog::bundled()?;
    for name in [
        "psp_14_0_2_sos.bin",
        "gc_12_0_0_pfp.bin",
        "gc_12_0_0_me.bin",
        "gc_12_0_0_mec.bin",
        "gc_12_0_0_rlc.bin",
        "gc_12_0_0_imu.bin",
        "sdma_7_0_0.bin",
        "smu_14_0_2.bin",
    ] {
        let path = required(&format!("firmware/{name}"))?;
        if catalog.firmware_hashes.get(name) != Some(&sha256(&path)?) {
            return Err(Error::Invalid(format!(
                "firmware differs from pinned AMD catalog: {name}"
            )));
        }
    }
    let descriptor_bytes = fs::read(&descriptor)?;
    crate::hcq_model::validate_descriptor(&descriptor_bytes, digest, model_size)?;
    let metadata_bytes = fs::read(&metadata)?;
    let value: Metadata = serde_json::from_slice(&metadata_bytes)?;
    if value.model_sha256 != digest {
        return Err(Error::Invalid(
            "native worker metadata identity mismatch".into(),
        ));
    }
    let checkpoint = value.checkpoint.clone();
    for [width, height] in [[1344, 760], [1928, 1208]] {
        let info = Info::new(serde_json::from_slice(&metadata_bytes)?, [width, height])?;
        let warp = required(&format!("warp-gfx1200-{width}x{height}.json"))?;
        let header: serde_json::Value = serde_json::from_slice(&fs::read(warp)?)?;
        if header["arch"] != "gfx1200" || header["camera"] != serde_json::json!([width, height]) {
            return Err(Error::Invalid("native AMD warp metadata mismatch".into()));
        }
        let prefix = format!("warp-qcom-a630-{width}x{height}");
        let graph: serde_json::Value =
            serde_json::from_slice(&fs::read(required(&format!("{prefix}/graph.json"))?)?)?;
        let provenance: serde_json::Value =
            serde_json::from_slice(&fs::read(required(&format!("{prefix}/provenance.json"))?)?)?;
        if graph["version"] != 1
            || graph["backend"] != "qcom-cl"
            || graph["arch"] != "a630"
            || provenance["camera"] != serde_json::json!([width, height])
            || provenance["frame_size"] != info.frame_size
        {
            return Err(Error::Invalid("native QCOM warp metadata mismatch".into()));
        }
        if graph["weights_sha256"] != sha256(&required(&format!("{prefix}/weights.bin"))?)? {
            return Err(Error::Invalid(
                "native QCOM warp weights checksum mismatch".into(),
            ));
        }
        let kernels = graph["kernels"]
            .as_array()
            .ok_or_else(|| Error::Invalid("native QCOM kernels missing".into()))?;
        if kernels.is_empty() || kernels.len() > 1024 {
            return Err(Error::Invalid("native QCOM kernel count".into()));
        }
        for (index, kernel) in kernels.iter().enumerate() {
            if kernel["binary_sha256"]
                != sha256(&required(&format!("{prefix}/kernel-{index}.bin"))?)?
            {
                return Err(Error::Invalid(
                    "native QCOM kernel checksum mismatch".into(),
                ));
            }
        }
    }
    Ok(Package {
        root: root.to_path_buf(),
        descriptor,
        metadata,
        checkpoint,
        manifest_sha256,
    })
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::{library::Library, model::Manifest};

    #[test]
    fn selectable_egpu_models_ship_matching_descriptors() {
        let repo = Path::new(env!("CARGO_MANIFEST_DIR")).join("../../..");
        let models = repo.join("device-inputs/usbgpu-models");
        let library = Library::for_runtime(&repo);
        for (key, entry) in library.available().unwrap() {
            let model: Manifest = serde_json::from_value(entry["egpu"].clone()).unwrap();
            let root = models.join(&model.sha256);
            let manifest: PackageManifest = serde_json::from_slice(
                &fs::read(root.join("manifest.json")).unwrap_or_else(|e| panic!("{key}: {e}")),
            )
            .unwrap();
            assert_eq!(manifest.model_sha256, model.sha256, "{key}");
            for name in ["model.hcq.json", "model.hcq-meta.json"] {
                let path = root.join(name);
                let relative = format!("models/{}/{name}", model.sha256);
                let file = manifest.files.iter().find(|f| f.path == relative).unwrap();
                assert_eq!(file.sha256, sha256(&path).unwrap(), "{key}/{name}");
                assert_eq!(file.bytes, path.metadata().unwrap().len(), "{key}/{name}");
            }
            let graph = fs::read(root.join("model.hcq.json")).unwrap();
            crate::hcq_model::validate_descriptor(&graph, &model.sha256, model.size).unwrap();
            let metadata: Metadata =
                serde_json::from_slice(&fs::read(root.join("model.hcq-meta.json")).unwrap())
                    .unwrap();
            assert_eq!(metadata.model_sha256, model.sha256, "{key}");
            let info = Info::new(metadata, [1344, 760]).unwrap();
            assert_eq!(
                info.layout.contains_key("nav_map"),
                library.noo(&key).unwrap(),
                "{key}"
            );
        }
    }
}
