//! Preserve transient artifacts and reject persistent model failures.
mod retry;
mod terminal;
pub use retry::retry_rejection;
pub use terminal::{
    gpu_bootloader_timeout, gpu_transport_failure, gpu_unreachable, gpu_usb_link_failure,
    gpu_worker_busy,
};

use crate::Error;
use serde::Deserialize;
use std::{fs, io::Write, path::Path};

#[derive(Clone, Copy, Deserialize)]
#[serde(rename_all = "snake_case")]
pub enum Kind {
    Detail,
    Timeout,
    BrokenPipe,
}

#[must_use]
pub fn pcie_not_ready(detail: &str) -> bool {
    let lower = detail.to_lowercase();
    [
        "pcie link not up",
        "pcie power off failed",
        "pcie power on failed",
        "usb bridge reset failed",
        "read(0xb450",
        "f0 out failed: -1",
        "libusb_open: no such device",
        "amd:0 does not exist",
    ]
    .iter()
    .any(|marker| lower.contains(marker))
}

#[must_use]
pub fn transient(detail: &str, kind: Kind) -> bool {
    matches!(kind, Kind::Timeout | Kind::BrokenPipe)
        || pcie_not_ready(detail)
        || detail.contains("precompiled eGPU worker timed out")
        || detail.contains("precompiled eGPU worker exited")
}

fn remove(path: &Path) -> Result<(), Error> {
    match fs::remove_file(path) {
        Ok(()) => Ok(()),
        Err(error) if error.kind() == std::io::ErrorKind::NotFound => Ok(()),
        Err(error) => Err(error.into()),
    }
}

pub fn record(
    path: &Path,
    detail: &str,
    kind: Kind,
    phase: &str,
    wall_time: f64,
) -> Result<bool, Error> {
    record_with_worker(path, detail, kind, phase, wall_time, None)
}

pub fn record_error(
    path: &Path,
    error: &Error,
    phase: &str,
    wall_time: f64,
) -> Result<bool, Error> {
    let kind = match error.worker_cause() {
        Error::Io(value) if value.kind() == std::io::ErrorKind::TimedOut => Kind::Timeout,
        Error::Io(value) if value.kind() == std::io::ErrorKind::BrokenPipe => Kind::BrokenPipe,
        _ => Kind::Detail,
    };
    record_with_worker(
        path,
        &crate::worker::diagnostic(error),
        kind,
        phase,
        wall_time,
        error.worker_snapshot(),
    )
}

pub fn record_with_worker(
    path: &Path,
    detail: &str,
    kind: Kind,
    phase: &str,
    wall_time: f64,
    worker: Option<&serde_json::Value>,
) -> Result<bool, Error> {
    if crate::local_artifact::is_local(path) {
        return crate::local_artifact::validation::record(path, detail, kind, phase, wall_time);
    }
    let root = path
        .parent()
        .ok_or(Error::Contract("installed model has no directory"))?;
    remove(&root.join("boot_validation.json"))?;
    let value =
        super::packaged::identity(path).map_err(|error| Error::Protocol(error.to_string()))?;
    let digest = value["pickle"]["sha256"]
        .as_str()
        .ok_or(Error::Contract("installed model hash missing"))?;
    let rejected = !(transient(detail, kind)
        || gpu_bootloader_timeout(detail, phase)
        || gpu_worker_busy(detail)
        || gpu_transport_failure(detail)
        || gpu_unreachable(detail)
        || gpu_usb_link_failure(
            detail,
            value["runtime"]["sha256"]
                .as_str()
                .or_else(|| value["runtime_archive_sha256"].as_str())
                .unwrap_or(""),
        ));
    let start = detail
        .char_indices()
        .rev()
        .nth(16383)
        .map_or(0, |(index, _)| index);
    let mut failure = serde_json::json!({"time":wall_time,"phase":phase,"rejected":rejected,
        "pickle_sha256":digest,"boot_id":boot_identity(),"error":&detail[start..]});
    if let Some(worker) = worker {
        failure["worker"] = worker.clone();
    }
    // The server exports at most 32 KiB. Retain valid JSON even for multibyte errors.
    while serde_json::to_vec(&failure)?.len() > 32768 {
        let text = failure["error"].as_str().unwrap_or("");
        if text.is_empty() {
            failure.as_object_mut().unwrap().remove("worker");
            break;
        }
        let mut start = text.len().div_ceil(4);
        while !text.is_char_boundary(start) {
            start += 1;
        }
        failure["error"] = serde_json::Value::String(text[start..].to_owned());
    }
    let mut staging = tempfile::Builder::new()
        .prefix(".failure-")
        .tempfile_in(root)?;
    serde_json::to_writer(&mut staging, &failure)?;
    staging.flush()?;
    staging
        .persist(root.join("last_failure.json"))
        .map_err(|error| error.error)?;
    if rejected {
        fs::write(root.join("rejected"), digest)?;
    }
    Ok(rejected)
}

/// A missing boot identity cannot authorize cross-boot rejection migration.
#[must_use]
pub fn boot_identity() -> String {
    fs::read_to_string("/proc/sys/kernel/random/boot_id")
        .map(|value| value.trim().to_owned())
        .unwrap_or_default()
}

/// Preserve only this boot validation attempt's worker snapshot across the runner process.
pub fn worker_since(path: &Path, earliest: f64) -> Option<serde_json::Value> {
    use std::io::Read;
    let root = path.parent()?;
    let file = fs::File::open(root.join("last_failure.json")).ok()?;
    let mut bytes = Vec::new();
    file.take(32769).read_to_end(&mut bytes).ok()?;
    if bytes.len() > 32768 {
        return None;
    }
    let value: serde_json::Value = serde_json::from_slice(&bytes).ok()?;
    let identity = super::packaged::identity(path).ok()?;
    let boot = boot_identity();
    if boot.is_empty()
        || value["boot_id"].as_str() != Some(&boot)
        || value["pickle_sha256"] != identity["pickle"]["sha256"]
        || value["time"].as_f64()? < earliest
    {
        return None;
    }
    value
        .get("worker")
        .filter(|value| value.is_object())
        .cloned()
}
