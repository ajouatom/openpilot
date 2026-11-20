//! A previous boot's recoverable failure permits revalidation, never a receipt.
use super::{gpu_bootloader_timeout, gpu_transport_failure, gpu_unreachable, gpu_usb_link_failure};
use serde_json::Value;
use std::{fs, path::Path};

#[must_use]
pub fn retry_rejection(root: &Path, digest: &str, current_boot: &str) -> bool {
    let Some(failure) = fs::read(root.join("last_failure.json"))
        .ok()
        .and_then(|v| serde_json::from_slice::<Value>(&v).ok())
    else {
        return false;
    };
    let phase = failure["phase"].as_str().unwrap_or("");
    let boot = failure["boot_id"].as_str().unwrap_or("");
    let Some(detail) = failure["error"].as_str() else {
        return false;
    };
    if failure["rejected"] != true
        || failure["pickle_sha256"] != digest
        || !matches!(phase, "load" | "boot_validation" | "inference")
        || boot.is_empty()
        || current_boot.is_empty()
        || boot == current_boot
    {
        return false;
    }
    // An unreachable GPU (TLP Unsupported Request) was recorded as a model
    // defect before b88eed0c7; lift those quarantines on the next boot too.
    if gpu_transport_failure(detail)
        || gpu_bootloader_timeout(detail, phase)
        || gpu_unreachable(detail)
    {
        return true;
    }
    let manifest = fs::read(root.join("installed.json"))
        .or_else(|error| {
            if error.kind() == std::io::ErrorKind::NotFound {
                fs::read(root.join("native-provider.json"))
            } else {
                Err(error)
            }
        })
        .ok()
        .and_then(|v| serde_json::from_slice::<Value>(&v).ok());
    if manifest.as_ref().is_some_and(|v| {
        v["pickle"]["sha256"] == digest
            && gpu_usb_link_failure(
                detail,
                v["runtime"]["sha256"]
                    .as_str()
                    .or_else(|| v["runtime_archive_sha256"].as_str())
                    .unwrap_or(""),
            )
    }) {
        return true;
    }
    if phase != "inference" {
        return false;
    }
    let terminal = detail.trim_end().rsplit('\n').next().unwrap_or("");
    if matches!(
        terminal,
        "ValueError: invalid precompiled model output"
            | "ValueError: non-finite model output"
            | "non-finite model output"
    ) {
        return true;
    }
    let Some((name, message)) = terminal.split_once(": ") else {
        return false;
    };
    let mut parts = name.split('.').peekable();
    while let Some(part) = parts.next() {
        if parts.peek().is_none() {
            return !message.is_empty()
                && matches!(part, "NonFiniteModelOutput" | "RepeatedInvalidModelOutput");
        }
        let mut chars = part.chars();
        if !chars
            .next()
            .is_some_and(|v| v.is_ascii_alphabetic() || v == '_')
            || !chars.all(|v| v.is_alphanumeric() || v == '_')
        {
            return false;
        }
    }
    false
}
