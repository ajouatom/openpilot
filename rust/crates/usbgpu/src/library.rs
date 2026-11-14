//! Reviewed external models and a selection frozen for the whole device
//! boot, plus offroad cache maintenance. Port of carrot's
//! `common/big_model_catalog.py` and `selfdrive/modeld/model_library.py`.
//!
//! The setting is separate from downloaded/loaded model status: saving a
//! choice never changes a running model, including during worker recovery.
use serde_json::{json, Map, Value};
use std::{
    collections::{BTreeMap, BTreeSet, HashSet},
    fs, io,
    path::{Path, PathBuf},
    sync::Mutex,
    time::Instant,
};

pub const DEFAULT: &str = "n16";

#[derive(Debug, thiserror::Error)]
pub enum Error {
    #[error(transparent)]
    Io(#[from] io::Error),
    #[error(transparent)]
    Json(#[from] serde_json::Error),
    #[error("{0}")]
    Invalid(&'static str),
}

/// Catalog and model cache locations.
#[derive(Clone, Debug)]
pub struct Library {
    pub directory: PathBuf,
    pub storage: PathBuf,
    pub boot_id_path: PathBuf,
}

fn read_json(path: &Path) -> Map<String, Value> {
    fs::read(path)
        .ok()
        .and_then(|b| serde_json::from_slice::<Value>(&b).ok())
        .and_then(|v| v.as_object().cloned())
        .unwrap_or_default()
}

fn atomic_json(path: &Path, value: &Value) -> Result<(), Error> {
    let parent = path.parent().ok_or(Error::Invalid("no parent"))?;
    fs::create_dir_all(parent)?;
    let temporary = parent.join(format!(
        ".selection-{}-{}",
        std::process::id(),
        path.file_name().and_then(|n| n.to_str()).unwrap_or("json")
    ));
    {
        use io::Write;
        let mut file = fs::File::create(&temporary)?;
        file.write_all(serde_json::to_string_pretty(value)?.as_bytes())?;
        file.write_all(b"\n")?;
        file.sync_all()?;
    }
    fs::rename(&temporary, path).map_err(|e| {
        let _ = fs::remove_file(&temporary);
        e.into()
    })
}

/// `library_lock`: one process-wide mutex and an exclusive file lock.
static LOCK: Mutex<()> = Mutex::new(());

static SIZES: Mutex<BTreeMap<String, (Instant, u64)>> = Mutex::new(BTreeMap::new());

impl Library {
    /// The device's catalog in `repository`, and the cache root.
    pub fn for_runtime(repository: &Path) -> Self {
        let storage = std::env::var_os("CARROT_BIG_MODEL_DIR")
            .map(PathBuf::from)
            .unwrap_or_else(|| {
                if Path::new("/TICI").is_file() {
                    PathBuf::from("/data/media/0/carrot/models")
                } else {
                    std::env::var_os("HOME")
                        .map(PathBuf::from)
                        .unwrap_or_default()
                        .join(".comma/models")
                }
            });
        Self {
            directory: repository.join("openpilot/selfdrive/modeld/model_catalog"),
            storage,
            boot_id_path: PathBuf::from("/proc/sys/kernel/random/boot_id"),
        }
    }

    fn locked<T>(&self, f: impl FnOnce() -> Result<T, Error>) -> Result<T, Error> {
        let _guard = LOCK
            .lock()
            .unwrap_or_else(std::sync::PoisonError::into_inner);
        fs::create_dir_all(&self.storage)?;
        let file = fs::OpenOptions::new()
            .append(true)
            .create(true)
            .open(self.storage.join(".library.lock"))?;
        rustix::fs::flock(&file, rustix::fs::FlockOperation::LockExclusive)
            .map_err(io::Error::from)?;
        f()
    }

    pub fn catalog(&self) -> Result<Map<String, Value>, Error> {
        let value: Value = serde_json::from_slice(&fs::read(self.directory.join("catalog.json"))?)?;
        value
            .as_object()
            .cloned()
            .ok_or(Error::Invalid("catalog must be an object"))
    }

    /// Curator-owned choices; retired metadata stays for safe cache cleanup.
    pub fn available(&self) -> Result<Map<String, Value>, Error> {
        let models: Map<String, Value> = self
            .catalog()?
            .into_iter()
            .filter(|(_, e)| e.get("retired") != Some(&Value::Bool(true)))
            .collect();
        if !models.contains_key(DEFAULT) {
            return Err(Error::Invalid(
                "catalog must retain an available default model",
            ));
        }
        Ok(models)
    }

    pub fn boot_id(&self) -> String {
        fs::read_to_string(&self.boot_id_path)
            .map(|s| s.trim().to_owned())
            .unwrap_or_else(|_| "desktop".into())
    }

    pub fn preferred(&self) -> Result<String, Error> {
        let value = read_json(&self.storage.join("selection.json"));
        let models = self.available()?;
        Ok(match value.get("selected").and_then(Value::as_str) {
            Some(key) if models.contains_key(key) => key.to_owned(),
            _ => DEFAULT.into(),
        })
    }

    /// Missing or corrupt boot state keeps the branch's known default.
    pub fn effective_id(&self) -> Result<String, Error> {
        let value = read_json(&self.storage.join("boot-selection.json"));
        let catalog = self.catalog()?;
        Ok(
            match (
                value.get("boot").and_then(Value::as_str),
                value.get("selected").and_then(Value::as_str),
            ) {
                (Some(boot), Some(key)) if boot == self.boot_id() && catalog.contains_key(key) => {
                    key.to_owned()
                }
                _ => DEFAULT.into(),
            },
        )
    }

    /// Accept the pending preference once per real boot, before any backend.
    pub fn freeze_boot(&self) -> Result<String, Error> {
        self.locked(|| {
            let path = self.storage.join("boot-selection.json");
            let value = read_json(&path);
            let catalog = self.catalog()?;
            if let (Some(boot), Some(key)) = (
                value.get("boot").and_then(Value::as_str),
                value.get("selected").and_then(Value::as_str),
            ) {
                if boot == self.boot_id() && catalog.contains_key(key) {
                    return Ok(key.to_owned());
                }
            }
            let selected = self.preferred()?;
            atomic_json(
                &path,
                &json!({"boot": self.boot_id(), "selected": selected}),
            )?;
            Ok(selected)
        })
    }

    pub fn selected(&self) -> Result<Value, Error> {
        let key = self.effective_id()?;
        self.catalog()?
            .remove(&key)
            .ok_or(Error::Invalid("unknown model"))
    }

    pub fn select(&self, key: &str) -> Result<(), Error> {
        self.locked(|| {
            if !self.available()?.contains_key(key) {
                return Err(Error::Invalid("model is not available"));
            }
            let mut value = read_json(&self.storage.join("selection.json"));
            value.insert("selected".into(), Value::String(key.into()));
            atomic_json(&self.storage.join("selection.json"), &Value::Object(value))
        })
    }

    pub fn model_spec(&self, key: &str) -> Result<Value, Error> {
        let catalog = self.catalog()?;
        let spec = catalog
            .get(key)
            .and_then(|e| e.get("jetson_spec"))
            .and_then(Value::as_str)
            .ok_or(Error::Invalid("unknown model"))?;
        Ok(serde_json::from_slice(&fs::read(
            self.directory.join(spec),
        )?)?)
    }

    /// The ONNX a Jetlink host runs for `key`: `big_driving_supercombo.onnx`
    /// beside the eGPU export of the model whose spec it shares (Flip Flop
    /// runs MD v3's graph on a host).
    pub fn onnx_url(&self, key: &str) -> Result<String, Error> {
        let catalog = self.catalog()?;
        let entry = catalog.get(key).ok_or(Error::Invalid("unknown model"))?;
        let spec = entry["jetson_spec"]
            .as_str()
            .ok_or(Error::Invalid("unknown model"))?;
        let owner = catalog.get(spec.trim_end_matches(".json")).unwrap_or(entry);
        let url = owner["egpu"]["url"]
            .as_str()
            .filter(|url| url.starts_with("https://"))
            .ok_or(Error::Invalid("model url"))?;
        let base = url.rsplit_once('/').ok_or(Error::Invalid("model url"))?.0;
        Ok(format!("{base}/big_driving_supercombo.onnx"))
    }

    /// The model takes the navigation map and route inputs.
    pub fn noo(&self, key: &str) -> Result<bool, Error> {
        let inputs = &self.model_spec(key)?["input_shapes"];
        Ok(inputs["nav_map"] == json!([1, 1, 256, 256]) && inputs["navigation"] == json!([1, 47]))
    }

    fn protected_ids(&self) -> Result<BTreeSet<String>, Error> {
        let mut protected = BTreeSet::from([self.effective_id()?, self.preferred()?]);
        let state = read_json(&self.storage.join("state.json"));
        let active = state
            .get("active")
            .and_then(|a| a.get("sha256"))
            .and_then(Value::as_str);
        for (key, entry) in self.catalog()? {
            if active.is_some() && entry["egpu"]["sha256"].as_str() == active {
                protected.insert(key);
            }
        }
        Ok(protected)
    }

    fn paths(&self, key: &str) -> Result<[PathBuf; 3], Error> {
        let catalog = self.catalog()?;
        let item = &catalog.get(key).ok_or(Error::Invalid("unknown model"))?["egpu"];
        let filename = item["filename"]
            .as_str()
            .ok_or(Error::Invalid("filename"))?;
        let sha = item["sha256"].as_str().ok_or(Error::Invalid("sha256"))?;
        let path = Path::new(filename);
        let stem = path
            .file_stem()
            .and_then(|s| s.to_str())
            .unwrap_or_default();
        let ext = path
            .extension()
            .and_then(|s| s.to_str())
            .map(|e| format!(".{e}"))
            .unwrap_or_default();
        let model = self
            .storage
            .join(format!("{stem}-{}{ext}", &sha[..16.min(sha.len())]));
        let part = PathBuf::from(format!("{}.part", model.display()));
        Ok([model, part, self.storage.join("precompiled").join(sha)])
    }

    fn unique_bytes(items: &[PathBuf]) -> u64 {
        use std::os::unix::fs::MetadataExt;
        let mut seen = HashSet::new();
        let mut total = 0;
        let mut visit = |path: &Path| {
            if let Ok(meta) = fs::symlink_metadata(path) {
                if meta.is_file() && seen.insert((meta.dev(), meta.ino())) {
                    total += meta.len();
                }
            }
        };
        for item in items {
            let meta = fs::symlink_metadata(item);
            if meta.as_ref().is_ok_and(fs::Metadata::is_dir) {
                let mut stack = vec![item.clone()];
                while let Some(dir) = stack.pop() {
                    for entry in fs::read_dir(&dir).into_iter().flatten().flatten() {
                        let path = entry.path();
                        match fs::symlink_metadata(&path) {
                            Ok(m) if m.is_dir() => stack.push(path),
                            Ok(_) => visit(&path),
                            Err(_) => {}
                        }
                    }
                }
            } else {
                visit(item);
            }
        }
        total
    }

    fn cached_bytes(&self, key: &str) -> Result<u64, Error> {
        let mut sizes = SIZES
            .lock()
            .unwrap_or_else(std::sync::PoisonError::into_inner);
        if let Some((stamp, size)) = sizes.get(key) {
            if stamp.elapsed().as_secs_f64() <= 30. {
                return Ok(*size);
            }
        }
        let size = Self::unique_bytes(&self.paths(key)?);
        sizes.insert(key.to_owned(), (Instant::now(), size));
        Ok(size)
    }

    fn remove_cache(&self, key: &str, offroad: &dyn Fn() -> bool) -> Result<(), Error> {
        self.locked(|| {
            if !self.catalog()?.contains_key(key) {
                return Err(Error::Invalid("unknown model"));
            }
            // Same lock as launch/background delivery: never remove a file being installed.
            let lock = fs::OpenOptions::new()
                .append(true)
                .create(true)
                .open("/tmp/big_model_update.lock")?;
            rustix::fs::flock(&lock, rustix::fs::FlockOperation::NonBlockingLockExclusive)
                .map_err(io::Error::from)?;
            if !offroad() {
                return Err(Error::Invalid("offroad state changed"));
            }
            if self.protected_ids()?.contains(key) {
                return Err(Error::Invalid(
                    "running, installed active or selected model is protected",
                ));
            }
            let root = fs::canonicalize(&self.storage)?;
            let paths = self.paths(key)?;
            for path in &paths {
                let resolved = fs::canonicalize(path).unwrap_or_else(|_| path.clone());
                if !resolved.starts_with(&root)
                    || fs::symlink_metadata(path).is_ok_and(|m| m.file_type().is_symlink())
                {
                    return Err(Error::Invalid("model cache path escapes storage"));
                }
            }
            for path in &paths {
                if path.is_dir() {
                    fs::remove_dir_all(path)?;
                } else if path.exists() {
                    fs::remove_file(path)?;
                }
            }
            SIZES
                .lock()
                .unwrap_or_else(std::sync::PoisonError::into_inner)
                .remove(key);
            let queue_path = self.storage.join("jetson-delete.json");
            let mut keys: BTreeSet<String> = read_json(&queue_path)
                .get("models")
                .and_then(Value::as_array)
                .map(|a| {
                    a.iter()
                        .filter_map(|v| v.as_str().map(str::to_owned))
                        .collect()
                })
                .unwrap_or_default();
            keys.insert(key.to_owned());
            atomic_json(&queue_path, &json!({"models": keys}))
        })
    }

    /// Apply reviewed catalog retirements only while actually offroad.
    pub fn reconcile_retired(&self, offroad: &dyn Fn() -> bool) -> Vec<String> {
        if !offroad() {
            return Vec::new();
        }
        let mut removed = Vec::new();
        let Ok(catalog) = self.catalog() else {
            return removed;
        };
        for (key, entry) in catalog {
            if entry.get("retired") != Some(&Value::Bool(true)) {
                continue;
            }
            if !offroad() {
                break;
            }
            // Running/selected/active or delivery busy: retry after transition.
            if self.remove_cache(&key, offroad).is_ok() {
                removed.push(key);
            }
        }
        removed
    }

    /// `model_library.status()`; `monotonic` is `time.monotonic()`.
    pub fn status(&self, monotonic: f64) -> Result<Value, Error> {
        let preferred = self.preferred()?;
        let effective = self.effective_id()?;
        let protected = self.protected_ids()?;
        let host = read_json(&self.storage.join("jetson-library.json"));
        let host_fresh = host.get("boot").and_then(Value::as_str) == Some(self.boot_id().as_str())
            && host
                .get("updated")
                .and_then(Value::as_f64)
                .is_some_and(|u| (0. ..30.).contains(&(monotonic - u)));
        let queue: Vec<String> = read_json(&self.storage.join("jetson-delete.json"))
            .get("models")
            .and_then(Value::as_array)
            .map(|a| {
                a.iter()
                    .filter_map(|v| v.as_str().map(str::to_owned))
                    .collect()
            })
            .unwrap_or_default();
        let mut rows = Vec::new();
        for (key, entry) in self.available()? {
            let spec = self.model_spec(&key)?;
            let sha = spec["sha256"].clone();
            rows.push(json!({
                "id": key, "name": entry["name"], "noo": self.noo(&key)?,
                "protected": protected.contains(&key), "egpu_bytes": self.cached_bytes(&key)?,
                "download_bytes": entry["egpu"]["size"], "jetson_sha256": sha,
                "jetson_bytes": if host_fresh { host.get("models").and_then(|m| m.get(sha.as_str().unwrap_or_default())).cloned().unwrap_or(Value::Null) } else { Value::Null },
                "delete_pending": queue.contains(&key),
            }));
        }
        let catalog = self.catalog()?;
        Ok(json!({
            "models": rows, "effective": effective, "selected": preferred,
            "reboot_required": effective != preferred, "internal": "CD210",
            "effective_name": catalog[&effective]["name"], "effective_noo": self.noo(&effective)?,
            "jetson_inventory_fresh": host_fresh,
        }))
    }
}

impl Library {
    /// The frozen selection's manifest URL (`big_model.DEFAULT_MANIFEST_URL`).
    pub fn selected_manifest_url(&self) -> Option<String> {
        self.selected()
            .ok()?
            .get("manifest_url")?
            .as_str()
            .map(str::to_owned)
    }

    /// `fetch_manifest`: reviewed catalog pins resolve without network.
    pub fn catalog_manifest(&self, url: &str) -> Option<crate::model::Manifest> {
        self.catalog().ok()?.values().find_map(|entry| {
            (entry.get("manifest_url")?.as_str()? == url)
                .then(|| serde_json::from_value(entry.get("egpu")?.clone()).ok())
                .flatten()
        })
    }
}

/// The library of the repository this process runs from.
pub fn current() -> Library {
    Library::for_runtime(&std::env::current_dir().unwrap_or_default())
}

#[cfg(test)]
mod tests {
    use super::*;

    fn library() -> Library {
        let root = std::env::temp_dir().join(format!(
            "library-{}-{:?}",
            std::process::id(),
            std::thread::current().id()
        ));
        let _ = fs::remove_dir_all(&root);
        let mut lib =
            Library::for_runtime(Path::new(concat!(env!("CARGO_MANIFEST_DIR"), "/../../..")));
        lib.storage = root.join("models");
        lib.boot_id_path = root.join("boot_id");
        fs::create_dir_all(&root).unwrap();
        fs::write(&lib.boot_id_path, "boot-a\n").unwrap();
        lib
    }

    #[test]
    fn selection_applies_on_the_next_boot_only() {
        let lib = library();
        assert_eq!(lib.effective_id().unwrap(), DEFAULT);
        assert_eq!(lib.freeze_boot().unwrap(), DEFAULT);
        lib.select("light-speed").unwrap();
        assert_eq!(lib.effective_id().unwrap(), DEFAULT, "frozen for this boot");
        assert_eq!(lib.freeze_boot().unwrap(), DEFAULT);
        fs::write(&lib.boot_id_path, "boot-b\n").unwrap();
        assert_eq!(lib.freeze_boot().unwrap(), "light-speed");
        assert!(
            lib.select("md-v3").is_err(),
            "retired models cannot be selected"
        );
        assert!(lib.noo(DEFAULT).is_ok());
        let status = lib.status(0.).unwrap();
        assert_eq!(status["effective"], "light-speed");
        assert_eq!(status["reboot_required"], false);
    }

    #[test]
    fn host_onnx_lives_beside_the_export_whose_graph_it_shares() {
        let lib = library();
        assert_eq!(
            lib.onnx_url("n16").unwrap(),
            "https://upload.shind0.synology.me/models/comma4-big-nav-n16-96244b1f/big_driving_supercombo.onnx"
        );
        assert_eq!(
            lib.onnx_url("flip-flop").unwrap(),
            "https://upload.shind0.synology.me/models/comma4-big-md-v3-1365126a/big_driving_supercombo.onnx"
        );
    }
}
