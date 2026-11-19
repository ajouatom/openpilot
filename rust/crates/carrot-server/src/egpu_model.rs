pub mod diagnostics;
use crate::{http::Application, http_response, params::Backend, Error, Value};
use hyper::{header, Method, Request, Response, StatusCode};
use num_traits::{ToPrimitive, Zero};
use openpilot_usbgpu::{
    hardware::USB_IDS,
    model::{self, Manifest, Paths},
};
use std::{
    env, fs,
    io::Write,
    path::{Path, PathBuf},
    sync::Arc,
    time::{SystemTime, UNIX_EPOCH},
};

const STATUS_ROUTE: &str = "/api/egpu/model";
const DIAGNOSTICS_ROUTE: &str = "/api/egpu/model/diagnostics";
const RESTART_ROUTE: &str = "/api/egpu/model/compile-restart";
const DEFAULT_MANIFEST_URL: &str = model::DEFAULT_MANIFEST_URL;
const STATES: [&str; 11] = [
    "checking",
    "downloading",
    "verifying",
    "ready",
    "waiting_for_ignition",
    "compiling",
    "compiled",
    "error",
    "waiting_for_network",
    "installing",
    "installed",
];
const UPDATING: [&str; 7] = [
    "checking",
    "downloading",
    "verifying",
    "compiling",
    "error",
    "waiting_for_network",
    "installing",
];
const BUSY: [&str; 6] = [
    "checking",
    "downloading",
    "verifying",
    "compiling",
    "waiting_for_network",
    "installing",
];

pub struct ModelFiles {
    pub paths: Paths,
    pub usb_devices: PathBuf,
    pub timestamp: Option<f64>,
    pub telemetry: diagnostics::Telemetry,
}

pub fn matches(path: &str) -> bool {
    matches!(path, STATUS_ROUTE | RESTART_ROUTE | DIAGNOSTICS_ROUTE)
}

fn stored_status(cache: &Path) -> Value {
    fs::read_to_string(cache.join("status.json"))
        .ok()
        .and_then(|text| Value::parse(&text).ok())
        .filter(|value| {
            matches!(value, Value::Object(_))
                && value.get("schema_version").number_eq(1)
                && STATES.iter().any(|state| value.get("state").text_eq(state))
        })
        .unwrap_or_else(|| Value::Object(Vec::new()))
}

fn name(value: &Value) -> Option<String> {
    let Value::Text(_) = value else { return None };
    let value = value.string().ok()?;
    (!value.is_empty()
        && value
            .bytes()
            .all(|b| b.is_ascii_alphanumeric() || b"._-".contains(&b)))
    .then_some(value)
}

fn manifest(value: &Value) -> Option<Manifest> {
    let model_id = name(value.get("model_id"))?;
    let filename = name(value.get("filename"))?;
    if !filename.ends_with(".pkl") && !filename.ends_with(".onnx") {
        return None;
    }
    let Value::Integer(size) = value.get("size") else {
        return None;
    };
    let size = size
        .to_u64()
        .filter(|size| (1..=4 * 1024 * 1024 * 1024).contains(size))?;
    let Value::Text(_) = value.get("sha256") else {
        return None;
    };
    let sha256 = value.get("sha256").string().ok()?;
    if sha256.len() != 64
        || !sha256
            .bytes()
            .all(|b| b.is_ascii_digit() || (b'a'..=b'f').contains(&b))
    {
        return None;
    }
    let Value::Text(_) = value.get("url") else {
        return None;
    };
    let source_url = value.get("url").string().ok()?;
    if source_url.is_empty() {
        return None;
    }
    let base = url::Url::parse(DEFAULT_MANIFEST_URL).ok()?;
    let source_url = source_url
        .trim_start_matches(|c: char| c.is_ascii() && c <= ' ')
        .replace(['\t', '\r', '\n'], "");
    let relative = if source_url
        .get(..8)
        .is_some_and(|prefix| prefix.eq_ignore_ascii_case("https://"))
    {
        let remainder = &source_url[8..];
        if remainder.is_empty() || remainder.starts_with(['/', '?', '#']) {
            remainder
        } else {
            &source_url
        }
    } else {
        &source_url
    };
    let url = base.join(relative).ok()?;
    if url.scheme() != "https" {
        return None;
    }
    Some(Manifest {
        model_id,
        filename,
        size,
        sha256,
        url: url.into(),
    })
}

fn cache_filename(manifest: &Manifest) -> String {
    let file = Path::new(&manifest.filename);
    let stem = file
        .file_stem()
        .unwrap_or(file.as_os_str())
        .to_string_lossy();
    let suffix = file
        .extension()
        .map(|suffix| format!(".{}", suffix.to_string_lossy()))
        .unwrap_or_default();
    format!("{stem}-{}{suffix}", &manifest.sha256[..16])
}

impl ModelFiles {
    pub fn original(repository: &Path) -> Self {
        let cache = env::var_os("CARROT_BIG_MODEL_DIR")
            .filter(|value| !value.is_empty())
            .map(PathBuf::from)
            .unwrap_or_else(|| {
                if Path::new("/TICI").is_file() {
                    "/data/media/0/carrot/models".into()
                } else {
                    PathBuf::from(env::var_os("HOME").unwrap_or_default()).join(".comma/models")
                }
            });
        Self {
            paths: Paths {
                models: repository.join("openpilot/selfdrive/modeld/models"),
                cache,
                assets: model::assets(repository),
            },
            usb_devices: "/sys/bus/usb/devices".into(),
            timestamp: None,
            telemetry: Default::default(),
        }
    }

    fn active_manifest(&self) -> Result<Option<Manifest>, Error> {
        let text = match fs::read_to_string(self.paths.cache.join("state.json")) {
            Ok(text) => text,
            Err(error)
                if error.kind() == std::io::ErrorKind::NotFound
                    || error.kind() == std::io::ErrorKind::InvalidData =>
            {
                return Ok(None)
            }
            Err(error) => return Err(error.into()),
        };
        let Ok(state) = Value::parse(&text) else {
            return Ok(None);
        };
        if state.get("previous") != &Value::Null && manifest(state.get("previous")).is_none() {
            return Ok(None);
        }
        let Some(manifest) = manifest(state.get("active")) else {
            return Ok(None);
        };
        let path = self.paths.cache.join(cache_filename(&manifest));
        if !path.is_file() {
            return Ok(None);
        }
        Ok((path.metadata()?.len() == manifest.size).then_some(manifest))
    }

    fn present(&self) -> bool {
        let Ok(entries) = fs::read_dir(&self.usb_devices) else {
            return false;
        };
        entries.filter_map(Result::ok).any(|entry| {
            let path = entry.path();
            let read_id = |name| {
                fs::read_to_string(path.join(name)).ok().and_then(|text| {
                    let trimmed = text.trim().strip_prefix('+').unwrap_or(text.trim());
                    u16::from_str_radix(
                        trimmed
                            .strip_prefix("0x")
                            .or_else(|| trimmed.strip_prefix("0X"))
                            .unwrap_or(trimmed),
                        16,
                    )
                    .ok()
                })
            };
            let Some(vendor) = read_id("idVendor") else {
                return false;
            };
            let Some(product) = read_id("idProduct") else {
                return false;
            };
            USB_IDS.contains(&(vendor, product))
                && fs::read_to_string(path.join("speed"))
                    .ok()
                    .and_then(|value| openpilot_runtime_core::python_float::parse(&value))
                    .is_some_and(|speed| speed >= 5000.)
        })
    }

    pub fn status(&self, params: &Backend) -> Result<Value, Error> {
        let hardware_seen = params
            .native_params()
            .is_some_and(|params| params.get_bool("UsbGpuHardwareSeen").unwrap_or(false));
        if !hardware_seen {
            return Ok(Value::object([
                ("ok", Value::Bool(true)),
                ("available", Value::Bool(false)),
            ]));
        }
        let status = stored_status(&self.paths.cache);
        let manifest = self.active_manifest()?;
        let same_model = manifest
            .as_ref()
            .is_some_and(|model| !status.truth() || status.get("sha256").text_eq(&model.sha256));
        let phase = status.get("state");
        let updating = UPDATING.iter().any(|state| phase.text_eq(state));
        let compiled =
            same_model && !updating && model::active_compiled_path(&self.paths).is_some();
        let mut state = if updating {
            phase.clone()
        } else if compiled {
            Value::text(if phase.text_eq("installed") {
                "installed"
            } else {
                "compiled"
            })
        } else if same_model {
            Value::text(if phase.text_eq("waiting_for_ignition") {
                "waiting_for_ignition"
            } else {
                "ready"
            })
        } else {
            Value::text("checking")
        };
        let fallback = Value::integer(
            manifest
                .as_ref()
                .filter(|_| same_model)
                .map_or(0, |manifest| manifest.size),
        );
        let downloaded = if status.has("downloaded_bytes") {
            status.get("downloaded_bytes")
        } else {
            &fallback
        }
        .int()?;
        let total = if status.has("total_bytes") {
            status.get("total_bytes")
        } else {
            &fallback
        }
        .int()?;
        let progress = if total > num_bigint::BigInt::zero() {
            let percentage = Value::Integer(downloaded.clone()).float()? * 100.
                / Value::Integer(total.clone()).float()?;
            let bounded = if percentage.is_nan() {
                0.
            } else {
                percentage.clamp(0., 100.)
            };
            Value::Float(
                format!("{bounded:.1}")
                    .parse()
                    .map_err(|_| Error::Source("invalid progress".into()))?,
            )
        } else {
            Value::Null
        };
        let engaged = params
            .native_params()
            .is_some_and(|params| params.get_bool("IsEngaged").unwrap_or(false));
        let identity = |key: &str| {
            if status.truth() {
                status.get(key).clone()
            } else {
                manifest
                    .as_ref()
                    .map(|m| {
                        Value::text(if key == "model_id" {
                            &m.model_id
                        } else {
                            &m.sha256
                        })
                    })
                    .unwrap_or(Value::Null)
            }
        };
        let boolean = |key| {
            params
                .native_params()
                .is_some_and(|params| params.get_bool(key).unwrap_or(false))
        };
        let active = boolean("UsbGpuActive") && !boolean("UsbGpuLoading");
        let mut error_code = status.get("error_code").clone();
        let mut detail = status.get("detail").clone();
        let blocked = manifest.as_ref().is_some_and(|manifest| {
            self.paths
                .cache
                .join("precompiled")
                .join(&manifest.sha256)
                .join("rejected")
                .exists()
        });
        if boolean("UsbGpuStartupFailed") && !active {
            let failure = manifest.as_ref().map_or_else(
                || Value::Object(Vec::new()),
                |manifest| {
                    diagnostics::current(
                        &self.paths.cache,
                        &manifest.sha256,
                        &self.telemetry.boot_id,
                    )
                },
            );
            state = Value::text("error");
            error_code = if failure.has("error_code") {
                failure.get("error_code").clone()
            } else {
                Value::text("runtime")
            };
            detail = match failure.get("worker").get("stage") {
                Value::Text(_) => Value::text(&format!(
                    "Last worker stage: {}",
                    failure.get("worker").get("stage").string()?
                )),
                _ => Value::Null,
            };
        }
        let recovery = if active {
            Value::Object(Vec::new())
        } else {
            self.telemetry.recovery()?
        };
        if ["waiting", "recovering", "ready"]
            .iter()
            .any(|phase| recovery.get("state").text_eq(phase))
        {
            state = Value::text(if recovery.get("state").text_eq("ready") {
                "recovery_ready"
            } else {
                "recovering"
            });
            error_code = Value::Null;
            detail = Value::Null;
        } else if recovery.get("state").text_eq("stopped") {
            state = Value::text("recovery_stopped");
            error_code = Value::Null;
            detail = Value::Null;
        }
        let can_restart = !blocked
            && !error_code.text_eq("rejected")
            && !recovery.truth()
            && same_model
            && !compiled
            && !engaged
            && !BUSY.iter().any(|phase| state.text_eq(phase));
        Ok(Value::object([
            ("ok", Value::Bool(true)),
            ("available", Value::Bool(true)),
            ("state", state),
            ("model_id", identity("model_id")),
            ("sha256", identity("sha256")),
            ("downloaded_bytes", Value::Integer(downloaded)),
            ("total_bytes", Value::Integer(total)),
            ("progress", progress),
            ("detail", detail),
            ("error_code", error_code),
            (
                "retry_count",
                if status.has("retry_count") {
                    status.get("retry_count").clone()
                } else {
                    Value::integer(0)
                },
            ),
            (
                "retry_in_seconds",
                if status.has("retry_in_seconds") {
                    status.get("retry_in_seconds").clone()
                } else {
                    Value::integer(0)
                },
            ),
            ("active", Value::Bool(active)),
            ("recovery", recovery),
            ("started_at", status.get("started_at").clone()),
            ("updated_at", status.get("updated_at").clone()),
            ("compiled", Value::Bool(compiled)),
            ("engaged", Value::Bool(engaged)),
            ("can_restart", Value::Bool(can_restart)),
        ]))
    }

    pub fn diagnostics(&self) -> Result<(StatusCode, Value), Error> {
        let manifest = self.active_manifest()?;
        let failure = manifest.as_ref().map_or_else(
            || Value::Object(Vec::new()),
            |manifest| diagnostics::saved(&self.paths.cache, &manifest.sha256),
        );
        if !failure.truth() {
            return Ok((
                StatusCode::NOT_FOUND,
                Value::object([
                    ("ok", Value::Bool(false)),
                    (
                        "error",
                        Value::text("No matching saved eGPU failure report."),
                    ),
                ]),
            ));
        }
        let manifest =
            manifest.ok_or_else(|| Error::Source("missing matching diagnostic manifest".into()))?;
        let same_boot = fs::read_to_string(&self.telemetry.boot_id)
            .ok()
            .filter(|id| !id.trim().is_empty() && failure.get("boot_id").truth())
            .map_or(Value::Null, |id| {
                Value::Bool(failure.get("boot_id").text_eq(id.trim()))
            });
        Ok((
            StatusCode::OK,
            Value::object([
                ("model_id", Value::text(&manifest.model_id)),
                ("sha256", Value::text(&manifest.sha256)),
                ("failure_from_current_boot", same_boot),
                ("usb_present", Value::Bool(self.present())),
                ("delivery_status", stored_status(&self.paths.cache)),
                ("failure", failure),
            ]),
        ))
    }

    fn write_restart_status(&self, manifest: &Manifest) -> Result<(), Error> {
        let now = self.timestamp.map(Ok).unwrap_or_else(|| {
            SystemTime::now()
                .duration_since(UNIX_EPOCH)
                .map(|duration| duration.as_secs_f64())
                .map_err(|error| Error::Source(error.to_string()))
        })?;
        let previous = stored_status(&self.paths.cache);
        let started = if previous.get("state").text_eq("waiting_for_ignition") {
            if previous.get("started_at").truth() {
                previous.get("started_at").float()?
            } else {
                now
            }
        } else {
            now
        };
        let status = Value::object([
            ("schema_version", Value::integer(1)),
            ("state", Value::text("waiting_for_ignition")),
            ("started_at", Value::Float(started)),
            ("updated_at", Value::Float(now)),
            ("model_id", Value::text(&manifest.model_id)),
            ("sha256", Value::text(&manifest.sha256)),
            ("downloaded_bytes", Value::integer(manifest.size)),
            ("total_bytes", Value::integer(manifest.size)),
            (
                "detail",
                Value::text("restart requested; compilation will begin during boot"),
            ),
        ]);
        fs::create_dir_all(&self.paths.cache)?;
        let mut temporary = tempfile::Builder::new()
            .prefix(".status-")
            .suffix(".json")
            .tempfile_in(&self.paths.cache)?;
        let mut ordered = match status {
            Value::Object(fields) => fields,
            _ => unreachable!(),
        };
        ordered.sort_by(|left, right| left.0.cmp(&right.0));
        let bytes = Value::Object(ordered)
            .encode()?
            .replace(": ", ":")
            .replace(", ", ",");
        temporary.write_all(bytes.as_bytes())?;
        temporary.write_all(b"\n")?;
        temporary.as_file().sync_all()?;
        temporary
            .persist(self.paths.cache.join("status.json"))
            .map_err(|error| Error::Io(error.error))?;
        Ok(())
    }

    pub fn restart(&self, params: &mut Backend) -> Result<(StatusCode, Value), Error> {
        let reject = |code, error| {
            (
                code,
                Value::object([("ok", Value::Bool(false)), ("error", Value::text(error))]),
            )
        };
        if !params.has_params() {
            return Ok(reject(
                StatusCode::INTERNAL_SERVER_ERROR,
                "params unavailable",
            ));
        }
        let status = self.status(params)?;
        if !status.get("available").truth() {
            return Ok(reject(
                StatusCode::NOT_FOUND,
                "eGPU has never been connected",
            ));
        }
        if status.get("engaged").truth() {
            return Ok(reject(
                StatusCode::CONFLICT,
                "disengage openpilot and park before restarting",
            ));
        }
        if !status.get("can_restart").truth() {
            return Ok((
                StatusCode::CONFLICT,
                Value::object([
                    ("ok", Value::Bool(false)),
                    ("error", Value::text("model is not ready for compilation")),
                    ("state", status.get("state").clone()),
                ]),
            ));
        }
        if !self.present() {
            return Ok(reject(
                StatusCode::CONFLICT,
                "turn ignition on and wait for eGPU power",
            ));
        }
        let Some(manifest) = self.active_manifest()? else {
            return Ok(reject(
                StatusCode::CONFLICT,
                "verified model is unavailable",
            ));
        };
        if self.active_manifest()?.is_none() {
            return Ok(reject(
                StatusCode::CONFLICT,
                "verified model is unavailable",
            ));
        }
        self.write_restart_status(&manifest)?;
        params.put("DoReboot", &Value::Bool(true), None)?;
        Ok((
            StatusCode::OK,
            Value::object([
                ("ok", Value::Bool(true)),
                ("reboot_requested", Value::Bool(true)),
            ]),
        ))
    }
}

pub async fn handle<T>(
    request: &Request<T>,
    app: Arc<Application>,
    files: Arc<ModelFiles>,
    path: &str,
) -> Response<crate::http::Body> {
    let head = request.method() == Method::HEAD;
    let restart = path == RESTART_ROUTE;
    let diagnostic = path == DIAGNOSTICS_ROUTE;
    if if restart {
        request.method() != Method::POST
    } else {
        !matches!(request.method(), &Method::GET | &Method::HEAD)
    } {
        let mut response = http_response::text(
            StatusCode::METHOD_NOT_ALLOWED,
            "405: Method Not Allowed",
            head,
        );
        response.headers_mut().insert(
            header::ALLOW,
            header::HeaderValue::from_static(if restart { "POST" } else { "GET,HEAD" }),
        );
        return response;
    }
    let result = tokio::task::spawn_blocking(move || {
        let mut params = app
            .params
            .lock()
            .map_err(|_| Error::Source("Params lock poisoned".into()))?;
        if restart {
            files.restart(&mut params)
        } else if diagnostic {
            files.diagnostics()
        } else {
            let mut value = files.status(&params)?;
            if let Value::Object(fields) = &mut value {
                fields.push((
                    "jetlink".chars().map(u32::from).collect(),
                    files.telemetry.jetlink()?,
                ));
                let library = crate::egpu_library::status(&app.config.repository);
                fields.push((
                    "library".chars().map(u32::from).collect(),
                    Value::parse(&library.to_string())?,
                ));
            }
            Ok((StatusCode::OK, value))
        }
    })
    .await;
    let failure = || {
        let mut response = http_response::text(
            StatusCode::INTERNAL_SERVER_ERROR,
            "500 Internal Server Error\n\nServer got itself in trouble",
            head,
        );
        response.headers_mut().insert(
            header::CONNECTION,
            header::HeaderValue::from_static("close"),
        );
        response
    };
    match result {
        Ok(Ok((code, payload))) => {
            let mut response =
                http_response::json_response(code, payload, head, "").unwrap_or_else(|_| failure());
            if diagnostic && code == StatusCode::OK {
                response.headers_mut().insert(
                    header::CACHE_CONTROL,
                    header::HeaderValue::from_static("no-store"),
                );
                response.headers_mut().insert(
                    header::CONTENT_DISPOSITION,
                    header::HeaderValue::from_static(
                        "attachment; filename=\"egpu-diagnostics.json\"",
                    ),
                );
            }
            response
        }
        Ok(Err(error)) => {
            eprintln!("eGPU model: {error}");
            failure()
        }
        Err(error) => {
            eprintln!("eGPU model task: {error}");
            failure()
        }
    }
}
