//! carrot's `server/features/navigation.py`: the Carrot Web navigation page,
//! Mapbox destination search, the session destination navd routes to
//! (`NavDestinationTemporary`, cleared when the drive ends), and the shared
//! nav-model map preview mapsd writes.
use crate::{
    http::{Application, Body, RequestBody},
    http_response::{response, text},
};
use http_body_util::BodyExt;
use hyper::{header, Method, Request, Response, StatusCode};
use serde_json::{json, Value};
use std::{
    io::Read,
    path::Path,
    sync::{Arc, Mutex},
    time::{Duration, Instant, SystemTime},
};

const MAX_RESPONSE: u64 = 1024 * 1024;
const MAX_REQUEST: usize = 16 * 1024;
const MAX_LATITUDE: f64 = 85.05112878;
/// mapsd's preview and its reason (`navigation/preview.py`).
pub const PREVIEW_PATH: &str = "/dev/shm/carrot_nav_map.png";
pub const STATUS_PATH: &str = "/dev/shm/carrot_nav_status.json";
/// Touched while a page shows the map, so mapsd renders it.
pub const WEB_DEMAND_PATH: &str = "/dev/shm/carrot_nav_web";
const PREVIEW_MAX_AGE: f64 = 2.;
const MEMORY_PARAMS: &str = "/dev/shm/params";

pub fn matches(path: &str) -> bool {
    matches!(
        path,
        "/navigation"
            | "/api/navigation/search"
            | "/api/navigation/destination"
            | "/api/navigation/map.png"
    )
}

/// One search at a time and at most one a second (`SearchState`).
struct Search {
    busy: bool,
    next: Option<Instant>,
}
static SEARCH: Mutex<Search> = Mutex::new(Search {
    busy: false,
    next: None,
});

fn json_reply(status: StatusCode, value: &Value, head: bool, no_store: bool) -> Response<Body> {
    let bytes = serde_json::to_vec(value).unwrap_or_else(|_| b"{}".to_vec());
    let mut result = response(status, bytes, "application/json; charset=utf-8", head);
    if no_store {
        result.headers_mut().insert(
            header::CACHE_CONTROL,
            header::HeaderValue::from_static("no-store"),
        );
    }
    result
}

fn error(status: StatusCode, message: &str, head: bool) -> Response<Body> {
    json_reply(status, &json!({"error": message}), head, false)
}

/// `float()` of a JSON number or numeric text.
fn float(value: &Value) -> Option<f64> {
    match value {
        Value::Number(number) => number.as_f64(),
        Value::String(text) => text.trim().parse().ok(),
        Value::Bool(flag) => Some(f64::from(u8::from(*flag))),
        _ => None,
    }
}

/// `destination()`: coordinates on the map and a place name of at most 256
/// characters.
pub fn destination(value: &Value) -> Option<Value> {
    let object = value.as_object()?;
    let latitude = float(object.get("latitude")?)?;
    let longitude = float(object.get("longitude")?)?;
    if !latitude.is_finite()
        || !longitude.is_finite()
        || latitude.abs() > MAX_LATITUDE
        || longitude.abs() > 180.
    {
        return None;
    }
    let name = match object.get("place_name") {
        None => "Destination".to_owned(),
        Some(Value::String(text)) => text.clone(),
        Some(other) => other.to_string(),
    };
    Some(json!({
        "latitude": latitude,
        "longitude": longitude,
        "place_name": name.chars().take(256).collect::<String>(),
    }))
}

fn params(app: &Application) -> Option<openpilot_params::Params> {
    app.params.lock().ok()?.native_params().cloned()
}

fn memory() -> Option<openpilot_params::Params> {
    openpilot_params::Params::for_runtime_at(Path::new(MEMORY_PARAMS)).ok()
}

fn read_text(params: Option<&openpilot_params::Params>, name: &str) -> String {
    params
        .and_then(|params| params.get(name).ok().flatten())
        .map(|bytes| String::from_utf8_lossy(&bytes).trim().to_owned())
        .unwrap_or_default()
}

/// `preview_fresh`: the file's mtime within 2 s of the wall clock.
pub fn fresh(path: &str) -> bool {
    std::fs::metadata(path)
        .and_then(|meta| meta.modified())
        .ok()
        .and_then(|modified| SystemTime::now().duration_since(modified).ok())
        .is_some_and(|age| age.as_secs_f64() < PREVIEW_MAX_AGE)
}

fn preview_reason() -> String {
    std::fs::read(STATUS_PATH)
        .ok()
        .and_then(|bytes| serde_json::from_slice::<Value>(&bytes).ok())
        .and_then(|value| value["reason"].as_str().map(str::to_owned))
        .unwrap_or_else(|| "waiting".into())
}

/// Mapbox Geocoding v6 forward search; up to five places.
fn geocode(query: &str, token: &str) -> Result<Vec<Value>, String> {
    let mut url = url::Url::parse("https://api.mapbox.com/search/geocode/v6/forward")
        .map_err(|_| "Mapbox search is unavailable.".to_owned())?;
    url.query_pairs_mut()
        .append_pair("q", query)
        .append_pair("access_token", token)
        .append_pair("limit", "5")
        .append_pair("autocomplete", "false")
        .append_pair("permanent", "false");
    let config = ureq::Agent::config_builder()
        .http_status_as_error(false)
        .timeout_global(Some(Duration::from_secs(8)))
        .user_agent("CarrotPilot navigation search")
        .build();
    let agent = ureq::Agent::new_with_config(config);
    // Never let the URL (it carries the token) reach a message.
    let unavailable = || "Mapbox search is unavailable.".to_owned();
    let mut reply = agent.get(url.as_str()).call().map_err(|_| unavailable())?;
    let status = reply.status().as_u16();
    if status != 200 {
        return Err(format!("Mapbox search failed (HTTP {status})."));
    }
    let mut body = Vec::new();
    reply
        .body_mut()
        .as_reader()
        .take(MAX_RESPONSE + 1)
        .read_to_end(&mut body)
        .map_err(|_| unavailable())?;
    if body.len() as u64 > MAX_RESPONSE {
        return Err(unavailable());
    }
    let value: Value = serde_json::from_slice(&body).map_err(|_| unavailable())?;
    let mut results = Vec::new();
    for feature in value["features"].as_array().into_iter().flatten().take(5) {
        let coordinates = feature["geometry"]["coordinates"]
            .as_array()
            .ok_or_else(unavailable)?;
        let (lon, lat) = (
            coordinates.first().ok_or_else(unavailable)?,
            coordinates.get(1).ok_or_else(unavailable)?,
        );
        let properties = &feature["properties"];
        let name = properties["full_address"]
            .as_str()
            .filter(|text| !text.is_empty())
            .or_else(|| properties["name"].as_str().filter(|text| !text.is_empty()))
            .unwrap_or(query);
        let place = destination(&json!({"latitude": lat, "longitude": lon, "place_name": name}))
            .ok_or_else(unavailable)?;
        results.push(place);
    }
    Ok(results)
}

async fn search(request: &Request<RequestBody>, app: &Application, head: bool) -> Response<Body> {
    let query = url::form_urlencoded::parse(request.uri().query().unwrap_or("").as_bytes())
        .find_map(|(key, value)| (key == "q").then(|| value.trim().to_owned()))
        .unwrap_or_default();
    if !(2..=200).contains(&query.chars().count()) {
        return error(StatusCode::BAD_REQUEST, "Enter 2–200 characters.", head);
    }
    let token = read_text(params(app).as_ref(), "MapboxPublicKey");
    if token.is_empty() {
        return error(
            StatusCode::CONFLICT,
            "Set the Mapbox Public key in Settings → Network.",
            head,
        );
    }
    {
        let Ok(mut state) = SEARCH.lock() else {
            return error(StatusCode::INTERNAL_SERVER_ERROR, "search state", head);
        };
        if state.busy || state.next.is_some_and(|next| Instant::now() < next) {
            return error(
                StatusCode::TOO_MANY_REQUESTS,
                "Please wait before searching again.",
                head,
            );
        }
        state.busy = true;
        state.next = Some(Instant::now() + Duration::from_secs(1));
    }
    let result = tokio::task::spawn_blocking(move || geocode(&query, &token)).await;
    if let Ok(mut state) = SEARCH.lock() {
        state.busy = false;
    }
    match result {
        Ok(Ok(results)) => json_reply(StatusCode::OK, &json!({"results": results}), head, true),
        Ok(Err(message)) => error(StatusCode::BAD_GATEWAY, &message, head),
        Err(_) => error(
            StatusCode::BAD_GATEWAY,
            "Mapbox search is unavailable.",
            head,
        ),
    }
}

fn current(app: &Application) -> Value {
    let temporary = read_text(memory().as_ref(), "NavDestinationTemporary");
    let value = if temporary.is_empty() {
        read_text(params(app).as_ref(), "NavDestination")
    } else {
        temporary
    };
    serde_json::from_str::<Value>(&value)
        .ok()
        .and_then(|value| destination(&value))
        .unwrap_or(Value::Null)
}

fn clear(app: &Application) {
    // navd reads a missing NavDestination as no destination.
    if let Some(params) = params(app) {
        let _ = params.remove("NavDestination");
        let _ = params.remove("NavDestinationWaypoints");
    }
}

async fn read_body(request: Request<RequestBody>) -> Option<Vec<u8>> {
    let mut body = request.into_body();
    let mut bytes = Vec::new();
    while let Some(frame) = body.frame().await {
        let frame = frame.ok()?;
        if let Ok(data) = frame.into_data() {
            if bytes.len() + data.len() > MAX_REQUEST {
                return None;
            }
            bytes.extend_from_slice(&data);
        }
    }
    Some(bytes)
}

fn preview(app: &Application, head: bool) -> Response<Body> {
    let _ = std::fs::write(WEB_DEMAND_PATH, b"");
    let bytes = if fresh(PREVIEW_PATH) {
        std::fs::read(PREVIEW_PATH).ok()
    } else {
        None
    };
    let Some(bytes) = bytes else {
        let onroad = read_text(params(app).as_ref(), "IsOnroad") == "1";
        let mut result = response(StatusCode::NO_CONTENT, Vec::new(), "image/png", head);
        let headers = result.headers_mut();
        headers.insert(
            header::CACHE_CONTROL,
            header::HeaderValue::from_static("no-store"),
        );
        headers.insert(
            "X-Nav-Map-Status",
            header::HeaderValue::from_static(if onroad { "waiting" } else { "offroad" }),
        );
        return result;
    };
    let mut result = response(StatusCode::OK, bytes, "image/png", head);
    let headers = result.headers_mut();
    headers.insert(
        header::CACHE_CONTROL,
        header::HeaderValue::from_static("no-store"),
    );
    if let Ok(reason) = header::HeaderValue::from_str(&preview_reason()) {
        headers.insert("X-Nav-Map-Status", reason);
    }
    result
}

pub async fn handle(request: Request<RequestBody>, app: Arc<Application>) -> Response<Body> {
    let head = request.method() == Method::HEAD;
    let path = request.uri().path().to_owned();
    let method = request.method().clone();
    let allowed = match path.as_str() {
        "/api/navigation/destination" => "DELETE,GET,HEAD,POST",
        _ => "GET,HEAD",
    };
    if !allowed.split(',').any(|item| item == method.as_str()) {
        let mut result = text(
            StatusCode::METHOD_NOT_ALLOWED,
            "405: Method Not Allowed",
            head,
        );
        result
            .headers_mut()
            .insert(header::ALLOW, header::HeaderValue::from_static(allowed));
        return result;
    }
    match path.as_str() {
        "/navigation" => {
            let page = app
                .config
                .repository
                .join("openpilot/selfdrive/carrot/web/navigation.html");
            let mut result = match crate::static_web::file_response(&page, &request).await {
                Ok(result) => result,
                Err(_) => return text(StatusCode::NOT_FOUND, "404: Not Found", head),
            };
            result.headers_mut().insert(
                header::CACHE_CONTROL,
                header::HeaderValue::from_static("no-cache"),
            );
            result
        }
        "/api/navigation/search" => search(&request, &app, head).await,
        "/api/navigation/map.png" => preview(&app, head),
        _ => match method {
            Method::POST => {
                let Some(value) = read_body(request)
                    .await
                    .and_then(|bytes| serde_json::from_slice::<Value>(&bytes).ok())
                    .and_then(|value| destination(&value))
                else {
                    return error(StatusCode::BAD_REQUEST, "Invalid destination.", head);
                };
                // The active destination lives in RAM; manager clears it when
                // the drive ends.
                clear(&app);
                let stored = memory().and_then(|memory| {
                    memory
                        .put("NavDestinationTemporary", value.to_string().as_bytes())
                        .ok()
                });
                if stored.is_none() {
                    return error(
                        StatusCode::INTERNAL_SERVER_ERROR,
                        "Could not store the destination.",
                        head,
                    );
                }
                json_reply(StatusCode::OK, &json!({"destination": value}), head, true)
            }
            Method::DELETE => {
                clear(&app);
                if let Some(memory) = memory() {
                    let _ = memory.remove("NavDestinationTemporary");
                }
                json_reply(StatusCode::OK, &json!({"destination": null}), head, false)
            }
            _ => {
                let key = !read_text(params(&app).as_ref(), "MapboxPublicKey").is_empty();
                json_reply(
                    StatusCode::OK,
                    &json!({
                        "destination": current(&app),
                        "map_ready": fresh(PREVIEW_PATH) && preview_reason() == "ready",
                        "key_configured": key,
                    }),
                    head,
                    true,
                )
            }
        },
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn destination_checks_coordinates_and_trims_the_name() {
        let ok = destination(
            &json!({"latitude": "37.5", "longitude": 127, "place_name": "x".repeat(300)}),
        )
        .unwrap();
        assert_eq!(ok["latitude"], 37.5);
        assert_eq!(ok["place_name"].as_str().unwrap().chars().count(), 256);
        assert_eq!(
            destination(&json!({"latitude": 1, "longitude": 2})).unwrap()["place_name"],
            "Destination"
        );
        assert!(destination(&json!({"latitude": 86, "longitude": 0})).is_none());
        assert!(destination(&json!({"latitude": 0, "longitude": 181})).is_none());
        assert!(destination(&json!({"latitude": "nan", "longitude": 0})).is_none());
        assert!(destination(&json!([1, 2])).is_none());
    }

    #[test]
    fn routes_are_the_original_four() {
        for path in [
            "/navigation",
            "/api/navigation/search",
            "/api/navigation/destination",
            "/api/navigation/map.png",
        ] {
            assert!(matches(path));
        }
        assert!(!matches("/api/navigation"));
    }
}
