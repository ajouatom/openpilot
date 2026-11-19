//! Model library API and offroad maintenance from carrot's
//! `server/features/egpu_model.py` (`api_library`, `library_maintenance`).
use crate::http::{Body, RequestBody};
use http_body_util::BodyExt;
use hyper::{Method, Request, Response, StatusCode};
use openpilot_usbgpu::library::Library;
use std::path::Path;

pub const ROUTE: &str = "/api/egpu/model/library";

fn monotonic() -> f64 {
    let t = rustix::time::clock_gettime(rustix::time::ClockId::Monotonic);
    t.tv_sec as f64 + t.tv_nsec as f64 * 1e-9
}

/// jetlinkd's model cache (`JETLINK_MODEL_CACHE`), where the App pick lives.
fn jetlink_cache() -> std::path::PathBuf {
    std::env::var_os("JETLINK_MODEL_CACHE")
        .map(std::path::PathBuf::from)
        .unwrap_or_else(|| std::path::PathBuf::from("/data/jetlink"))
}

/// A Jetlink App switches models whenever asked, so a choice made here is
/// handed to it as a pick at the next connection while parked instead of
/// waiting for a reboot (the eGPU keeps its next-boot rule).
fn pick_for_app(library: &Library, key: &str) -> Result<(), String> {
    let at = std::time::SystemTime::now()
        .duration_since(std::time::UNIX_EPOCH)
        .map_err(|e| e.to_string())?
        .as_secs_f64();
    write_pick(library, key, at)
}

/// The pick an untouched model library stands for: the card shows its
/// choice (N16 by default) as the model, but no pick was sent until one was
/// made here, so an App with no model loaded waited forever ("No such file
/// or directory" from the missing link socket). Dated 1970, it is older than
/// any pick made in the App, so a model chosen there still wins.
pub fn ensure_default_app_pick(repository: &Path) {
    let cache = jetlink_cache();
    if cache.join(openpilot_jetlink::app_pick::FILE).exists() {
        return;
    }
    let library = Library::for_runtime(repository);
    let result = library
        .preferred()
        .map_err(|e| e.to_string())
        .and_then(|key| write_pick(&library, &key, 1.));
    if let Err(error) = result {
        eprintln!("model library: default App pick not saved: {error}");
    }
}

fn write_pick(library: &Library, key: &str, at: f64) -> Result<(), String> {
    let catalog = library.catalog().map_err(|e| e.to_string())?;
    let spec = library.model_spec(key).map_err(|e| e.to_string())?;
    let spec = openpilot_jetlink::model_spec::ModelSpec::parse(
        &serde_json::to_vec(&spec).map_err(|e| e.to_string())?,
    )
    .map_err(|e| e.to_string())?;
    let pick = openpilot_jetlink::app_pick::AppPick {
        model: key.into(),
        name: catalog[key]["name"].as_str().unwrap_or(key).into(),
        url: library.onnx_url(key).map_err(|e| e.to_string())?,
        at,
        spec,
    };
    openpilot_jetlink::app_pick::write(&jetlink_cache(), &pick).map_err(|e| e.to_string())
}

/// The comma's own driving model (`InternalDrivingModel`): 0 CD210, 1 Cheddar.
/// drivingd reads it when it starts, so a change applies from the next drive.
const INTERNAL_KEY: &str = "InternalDrivingModel";
const INTERNAL_MODELS: [&str; 2] = ["cd210", "cheddar"];

fn internal_status() -> serde_json::Value {
    let params = openpilot_params::Params::for_runtime().ok();
    let read = |key| {
        params
            .as_ref()
            .and_then(|params| params.get(key).ok().flatten())
    };
    let value = read(INTERNAL_KEY).unwrap_or_default();
    let selected = if value.trim_ascii() == b"1" {
        "cheddar"
    } else {
        "cd210"
    };
    // What drivingd actually loaded on the last start: {selected, running,
    // error}. Missing until the first drive after the comma starts.
    let last = read("InternalDrivingModelStatus")
        .and_then(|bytes| serde_json::from_slice::<serde_json::Value>(&bytes).ok())
        .unwrap_or(serde_json::Value::Null);
    serde_json::json!({
        "selected": selected,
        "models": [{"id": "cd210", "name": "CD210"}, {"id": "cheddar", "name": "Cheddar"}],
        "last": last,
    })
}

fn select_internal(key: &str) -> Result<(), String> {
    let index = INTERNAL_MODELS
        .iter()
        .position(|model| *model == key)
        .ok_or("unknown internal model")?;
    openpilot_params::Params::for_runtime()
        .and_then(|params| params.put(INTERNAL_KEY, index.to_string().as_bytes()))
        .map_err(|error| error.to_string())
}

/// `model_library.status()` for the status payload, with the App pick.
pub fn status(repository: &Path) -> serde_json::Value {
    let mut value = Library::for_runtime(repository)
        .status(monotonic())
        .unwrap_or_else(|error| serde_json::json!({"error": error.to_string()}));
    value["internal_model"] = internal_status();
    let cache = jetlink_cache();
    if let Some(pick) = openpilot_jetlink::app_pick::read(&cache) {
        let failed = openpilot_jetlink::app_pick::current(&cache).is_none();
        value["app_pick"] =
            serde_json::json!({"model": pick.model, "sha256": pick.spec.sha256, "failed": failed});
    }
    value
}

fn reply(code: StatusCode, value: serde_json::Value) -> Response<Body> {
    let mut response = Response::new(Body::from(value.to_string()));
    *response.status_mut() = code;
    response.headers_mut().insert(
        hyper::header::CONTENT_TYPE,
        hyper::header::HeaderValue::from_static("application/json; charset=utf-8"),
    );
    response
}

pub async fn handle(request: Request<RequestBody>, repository: &Path) -> Response<Body> {
    if request.method() != Method::POST {
        return reply(
            StatusCode::METHOD_NOT_ALLOWED,
            serde_json::json!({"ok": false, "error": "405"}),
        );
    }
    let Ok(collected) = request.into_body().collect().await else {
        return reply(
            StatusCode::CONFLICT,
            serde_json::json!({"ok": false, "error": "invalid body"}),
        );
    };
    let bytes = collected.to_bytes();
    let repository = repository.to_owned();
    let result = tokio::task::spawn_blocking(move || -> Result<serde_json::Value, String> {
        let body: serde_json::Value =
            serde_json::from_slice(&bytes).map_err(|_| "invalid model action")?;
        let fields = body.as_object().ok_or("invalid model action")?;
        if fields.len() != 2 || !fields.contains_key("action") || !fields.contains_key("model") {
            return Err("invalid model action".into());
        }
        let library = Library::for_runtime(&repository);
        let key = body["model"].as_str().unwrap_or_default();
        if body["action"] == "internal" {
            select_internal(key)?;
            let mut status = library.status(monotonic()).map_err(|e| e.to_string())?;
            status["internal_model"] = internal_status();
            return Ok(serde_json::json!({"ok": true, "library": status}));
        }
        let available = library.available().map_err(|e| e.to_string())?;
        if body["action"] != "select" || !available.contains_key(key) {
            return Err("unknown model action".into());
        }
        library.select(key).map_err(|e| e.to_string())?;
        let app_pick = pick_for_app(&library, key);
        if let Err(error) = &app_pick {
            eprintln!("model library: App pick not saved: {error}");
        }
        let mut status = library.status(monotonic()).map_err(|e| e.to_string())?;
        status["internal_model"] = internal_status();
        if app_pick.is_ok() {
            status["app_pick"] = serde_json::json!({"model": key});
        }
        Ok(serde_json::json!({"ok": true, "library": status, "app_pick_error": app_pick.err()}))
    })
    .await;
    match result {
        Ok(Ok(value)) => reply(StatusCode::OK, value),
        Ok(Err(error)) => reply(
            StatusCode::CONFLICT,
            serde_json::json!({"ok": false, "error": error}),
        ),
        Err(error) => reply(
            StatusCode::CONFLICT,
            serde_json::json!({"ok": false, "error": error.to_string()}),
        ),
    }
}

/// `library_maintenance`: apply catalog retirements every 30 s offroad.
pub fn start_maintenance(repository: &Path, params: Option<openpilot_params::Params>) {
    let Some(params) = params else {
        return;
    };
    let library = Library::for_runtime(repository);
    let _ = std::thread::Builder::new()
        .name("model-library".into())
        .spawn(move || loop {
            let offroad = || {
                params.get_bool("IsOffroad").unwrap_or(false)
                    && !params.get_bool("IsOnroad").unwrap_or(false)
            };
            library.reconcile_retired(&offroad);
            std::thread::sleep(std::time::Duration::from_secs(30));
        });
}
