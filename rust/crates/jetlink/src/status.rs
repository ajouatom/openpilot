//! Source display status: saved HELLO is identity; only fresh telemetry is health.
use crate::peer::{self, Kind};
use serde::Serialize;
use serde_json::{json, Value};
use std::{fs, net::IpAddr, path::Path};
pub const LINK_STATUS: &str = "/dev/shm/carrot-jetlink.json";
pub const MODEL_STATUS: &str = "/dev/shm/carrot-jetlink-model.json";
pub const RECOVERY_STATUS: &str = "/dev/shm/carrot-egpu-recovery.json";
#[derive(Clone, Debug, PartialEq, Eq, Serialize)]
pub struct Badge {
    pub text: String,
    pub style: String,
}
impl Badge {
    fn new(text: impl Into<String>, style: &str) -> Self {
        Self {
            text: text.into(),
            style: style.into(),
        }
    }
}
fn numeric(value: &Value) -> Option<f64> {
    value
        .as_f64()
        .or_else(|| value.as_bool().map(|value| if value { 1. } else { 0. }))
}
fn float(value: &Value) -> Option<f64> {
    numeric(value).or_else(|| openpilot_runtime_core::python_float::parse(value.as_str()?))
}
fn truth(value: &Value) -> bool {
    match value {
        Value::Null => false,
        Value::Bool(value) => *value,
        Value::Number(_) => numeric(value) != Some(0.),
        Value::String(value) => !value.is_empty(),
        Value::Array(value) => !value.is_empty(),
        Value::Object(value) => !value.is_empty(),
    }
}
fn text(value: &Value) -> String {
    match value {
        Value::String(value) => value.clone(),
        Value::Null => "None".into(),
        Value::Bool(true) => "True".into(),
        Value::Bool(false) => "False".into(),
        _ => value.to_string(),
    }
}
fn limited(value: &Value) -> String {
    text(value).chars().take(240).collect()
}
pub fn read_json(path: &Path) -> Value {
    fs::read(path)
        .ok()
        .and_then(|data| serde_json::from_slice::<Value>(&data).ok())
        .filter(Value::is_object)
        .unwrap_or_else(|| json!({}))
}
pub fn fresh(value: &Value, now: f64) -> bool {
    numeric(&value["updated"]).is_some_and(|updated| (0. ..3.).contains(&(now - updated)))
}
pub fn read_fresh(path: &Path, now: f64) -> Value {
    let value = read_json(path);
    if fresh(&value, now) {
        value
    } else {
        json!({})
    }
}
pub fn read_recovery(path: &Path, now: f64) -> Value {
    let value = read_fresh(path, now);
    if matches!(
        value["state"].as_str(),
        Some("waiting" | "recovering" | "ready" | "stopped")
    ) {
        value
    } else {
        json!({})
    }
}
pub fn read_delivery(cache: &Path) -> Value {
    let value = read_json(&cache.join("status.json"));
    if numeric(&value["schema_version"]) == Some(1.)
        && matches!(
            value["state"].as_str(),
            Some(
                "checking"
                    | "downloading"
                    | "verifying"
                    | "ready"
                    | "waiting_for_ignition"
                    | "compiling"
                    | "compiled"
                    | "error"
                    | "waiting_for_network"
                    | "installing"
                    | "installed"
            )
        )
    {
        value
    } else {
        json!({})
    }
}
pub fn recovery_badge(value: &Value) -> Option<Badge> {
    Some(match value["state"].as_str()? {
        "waiting" | "recovering" => Badge::new("eGPU RETRY", "loading"),
        "ready" => Badge::new("eGPU READY", "ready"),
        "stopped" => Badge::new("eGPU ERROR", "error"),
        _ => return None,
    })
}
pub fn delivery_badge(value: &Value) -> Option<Badge> {
    Some(match value["state"].as_str()? {
        "waiting_for_network" => Badge::new(
            if value["error_code"] == "clock" {
                "eGPU CLOCK"
            } else {
                "eGPU NET"
            },
            "loading",
        ),
        "downloading" | "verifying" | "installing" => Badge::new("eGPU SETUP", "loading"),
        "installed" => Badge::new("eGPU NEXT", "ready"),
        "error" => Badge::new(
            if value["error_code"] == "rejected" {
                "eGPU FILE"
            } else {
                "eGPU ERROR"
            },
            "error",
        ),
        _ => return None,
    })
}
pub fn update_badge(value: &Value) -> Badge {
    match value["state"].as_str() {
        Some("waiting_internet") => {
            Badge::new("Jetson 업데이트 필요 · 인터넷 연결 대기", "loading")
        }
        Some("downloading") => {
            let percent = numeric(&value["fraction"])
                .filter(|value| (0. ..=1.).contains(value))
                .map_or_else(String::new, |value| {
                    format!(
                        " {:.0}%",
                        if value == 0. {
                            0.
                        } else {
                            (value * 100.).trunc()
                        }
                    )
                });
            Badge::new(format!("Jetson 업데이트 다운로드 중{percent}"), "loading")
        }
        Some("verifying") => Badge::new("Jetson 업데이트 검증·적용 중", "loading"),
        Some("failed") => Badge::new("Jetson 업데이트 실패 · 재시도 대기", "error"),
        _ => Badge::new("Jetson 업데이트 확인 중", "loading"),
    }
}
pub fn model_name(sha256: &str) -> String {
    match sha256 {
        "09d080f36965bb2a0790500452bd328aa03c484d0222aa79d1ad9f021a522aec" => "Cinque v2".into(),
        crate::contract::SHA256 => "Native Cinque v3".into(),
        _ => format!("App model {}", sha256.chars().take(12).collect::<String>()),
    }
}

pub fn host_label(value: &Value, mode: &str) -> &'static str {
    if !value.is_object() {
        return "Jetlink";
    }
    let mode = if mode == "auto" {
        match peer::classify(value, "usb") {
            Some(Kind::Mac) => "usb",
            Some(Kind::Android) => "android",
            _ => "auto",
        }
    } else {
        mode
    };
    if mode == "ios" && peer::classify(value, "tcp") == Some(Kind::Ios) {
        return "iOS";
    }
    if mode == "android" && peer::classify(value, "usb") == Some(Kind::Android) {
        return "Android";
    }
    match value["carrot_host"].as_str() {
        Some("jetson") => return "jetSON",
        Some("mac") => return "MAC",
        _ => {}
    }
    if value["backend"] == "trt"
        && text(value.get("device").unwrap_or(&Value::String(String::new())))
            .to_lowercase()
            .contains("orin")
    {
        return "jetSON";
    }
    if mode == "usb" && peer::classify(value, "usb") == Some(Kind::Mac) {
        return "MAC";
    }
    "Jetlink"
}
#[derive(Clone, Debug, Serialize)]
pub struct Diagnostics {
    pub label: String,
    pub state: String,
    pub severity: String,
    pub reason: String,
    pub addresses: Vec<String>,
    pub temp_c: Option<Value>,
    pub active: bool,
    pub fresh: bool,
    /// The model the host serves or prepares, for the web's model library.
    pub sha256: Option<String>,
    /// Download/upload/build progress while preparing it.
    pub preparation: Value,
}
pub fn diagnostics(saved: &Value, link: &Value, model: &Value, now: f64) -> Option<Diagnostics> {
    if !truth(&saved["peer"])
        && !matches!(
            saved["state"].as_str(),
            Some("connecting" | "loading" | "retrying" | "blocked")
        )
        && !truth(&model["active"])
    {
        return None;
    }
    let mut result = Diagnostics {
        label: host_label(&saved["peer"], saved["transport"].as_str().unwrap_or("usb")).into(),
        state: link["state"].as_str().unwrap_or("disconnected").into(),
        severity: "unknown".into(),
        reason: String::new(),
        addresses: Vec::new(),
        temp_c: None,
        active: truth(&model["active"]),
        fresh: false,
        sha256: link["sha256"]
            .as_str()
            .filter(|sha| sha.len() == 64 && sha.bytes().all(|b| b.is_ascii_hexdigit()))
            .map(str::to_owned),
        preparation: if link["state"] == "loading" {
            let item = &link["preparation"];
            serde_json::json!({
                "stage": item["stage"].as_str().map(|v| v.chars().take(40).collect::<String>()),
                "fraction": item["fraction"].as_f64().filter(|v| (0. ..=1.).contains(v)),
                "message": item["message"].as_str().map(|v| v.chars().take(200).collect::<String>()),
            })
        } else {
            Value::Null
        },
    };
    if link["state"] == "updating" && result.label == "jetSON" {
        let badge = update_badge(&link["host_update"]);
        result.severity = if badge.style == "error" {
            "error"
        } else {
            "warning"
        }
        .into();
        result.reason = badge.text;
        result.fresh = true;
        result.active = false;
        return Some(result);
    }
    let error = if !truth(&model["active"]) && truth(&model["error"]) {
        limited(&model["error"])
    } else {
        String::new()
    };
    if !error.is_empty() {
        result.severity = "error".into();
        result.reason = error.clone();
    }
    if link["state"] == "waiting" && !truth(&model["active"]) && error.is_empty() {
        result.reason = "Host not connected".into();
        return Some(result);
    }
    if matches!(
        result.state.as_str(),
        "retrying" | "stopped" | "disconnected" | "waiting" | "blocked"
    ) {
        result.severity = "error".into();
        result.reason = if truth(&link["error"]) {
            limited(&link["error"])
        } else {
            "Host connection unavailable".into()
        };
        return Some(result);
    }
    let health = &link["telemetry"]["carrot_health"];
    let age = float(&link["telemetry_updated"])
        .zip(float(&health["age_s"]))
        .map(|(updated, age)| now - updated + age);
    if !age.is_some_and(|age| (0. ..3.).contains(&age)) {
        return Some(result);
    }
    result.fresh = true;
    result.severity = health["severity"]
        .as_str()
        .filter(|value| matches!(*value, "ok" | "warning" | "error" | "unknown"))
        .unwrap_or("unknown")
        .into();
    result.reason = health.get("reason").map_or_else(String::new, limited);
    if numeric(&health["temp_c"])
        .is_some_and(|value| value.is_finite() && (-20. ..=150.).contains(&value))
    {
        result.temp_c = Some(health["temp_c"].clone());
    }
    if let Some(entries) = health["addresses"].as_array() {
        for entry in entries.iter().take(8) {
            let address = entry["address"]
                .as_str()
                .and_then(|value| value.parse::<IpAddr>().ok())
                .or_else(|| {
                    entry["address"]
                        .as_u64()
                        .and_then(|value| u32::try_from(value).ok())
                        .map(|value| IpAddr::V4(value.into()))
                });
            if let Some(address) = address {
                let local = match address {
                    IpAddr::V4(value) => value.is_link_local(),
                    IpAddr::V6(value) => value.is_unicast_link_local(),
                };
                if !address.is_loopback() && !address.is_unspecified() && !local {
                    result.addresses.push(address.to_string());
                }
            }
        }
    }
    if !error.is_empty() {
        result.severity = "error".into();
        result.reason = error;
    }
    Some(result)
}
pub fn badge(saved: &Value, link: &Value, model: &Value, now: f64) -> Option<Badge> {
    let mut label =
        host_label(&link["peer"], link["transport"].as_str().unwrap_or("usb")).to_owned();
    if link["state"] == "updating" && label == "jetSON" {
        return Some(update_badge(&link["host_update"]));
    }
    if let Some(health) = diagnostics(saved, link, model, now) {
        label = health.label;
        if health.state == "waiting" && health.severity == "unknown" {
            return None;
        }
        if health.severity == "error" {
            return Some(Badge::new(format!("{label} ERROR"), "error"));
        }
        if health.severity == "warning" {
            return Some(Badge::new(format!("{label} HOT"), "loading"));
        }
        if label == "jetSON"
            && link["state"] == "ready"
            && (!health.fresh || health.severity == "unknown")
        {
            return Some(Badge::new(format!("{label} DATA?"), "loading"));
        }
    }
    if truth(&model["active"]) {
        return Some(Badge::new(label, "active"));
    }
    Some(match link["state"].as_str()? {
        "ready" => Badge::new(format!("{label} READY"), "ready"),
        "connecting" | "loading" => Badge::new(format!("{label} WAIT"), "loading"),
        "retrying" => Badge::new(format!("{label} RETRY"), "error"),
        _ => return None,
    })
}
pub fn write(path: &Path, value: &Value) -> std::io::Result<()> {
    let temporary = path.with_extension("tmp");
    fs::write(&temporary, serde_json::to_vec(value)?)?;
    fs::rename(temporary, path)
}
