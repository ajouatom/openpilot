//! openpilot-mapsd: carrot's `selfdrive/navigation/mapsd.py`. Renders the
//! N16 route map while a navigation graph asks for it (DEMAND_PATH) and a
//! route, a position and a fresh instruction exist; black otherwise.
use openpilot_cereal::{custom_capnp::carrot_man, log_capnp::event};
use openpilot_mapsd::{
    output::{self, Map, DEMAND_PATH, MAP_BYTES},
    remote::{Archive, Mapbox},
    renderer::{MapRenderer, Position},
    route::{RouteTracker, MAX_LATITUDE},
    tiles::{HybridTiles, Remote},
    Error,
};
use openpilot_messaging::{runtime::SubMaster, state::Options};
use std::{
    path::{Path, PathBuf},
    sync::{
        atomic::{AtomicBool, Ordering},
        Arc,
    },
    time::Duration,
};

fn monotonic() -> f64 {
    let t = rustix::time::clock_gettime(rustix::time::ClockId::Monotonic);
    t.tv_sec as f64 + t.tv_nsec as f64 * 1e-9
}

enum Source {
    Mapbox(Mapbox),
    Archive(Archive),
}
impl Remote for Source {
    fn version(&mut self) -> Result<Option<String>, Error> {
        match self {
            Self::Mapbox(r) => r.version(),
            Self::Archive(r) => r.version(),
        }
    }
    fn fetch(&mut self, z: u8, x: u64, y: u64) -> Result<Option<Vec<u8>>, Error> {
        match self {
            Self::Mapbox(r) => r.fetch(z, x, y),
            Self::Archive(r) => r.fetch(z, x, y),
        }
    }
}

fn text(params: &openpilot_params::Params, key: &str) -> String {
    params.get(key).ok().flatten().map(|v| String::from_utf8_lossy(&v).trim().to_owned()).unwrap_or_default()
}

/// As carrot: a Mapbox token wins; otherwise a configured PMTiles URL.
fn remote(params: &openpilot_params::Params) -> (String, Option<Source>) {
    let token = std::env::var("MAPBOX_TOKEN").ok().filter(|t| !t.is_empty()).unwrap_or_else(|| text(params, "MapboxPublicKey"));
    if !token.is_empty() {
        return (format!("mapbox:{}", token.len()), Some(Source::Mapbox(Mapbox::new(token))));
    }
    let url = text(params, "MapTilesUrl");
    if url.starts_with("https://") || url.starts_with("http://") {
        return (url.clone(), Some(Source::Archive(Archive::new(url))));
    }
    (String::new(), None)
}

fn store_root() -> PathBuf {
    if Path::new("/TICI").exists() {
        PathBuf::from("/data/navigation/tiles")
    } else {
        PathBuf::from(std::env::var("HOME").unwrap_or_default()).join(".comma/navigation/tiles")
    }
}

struct Fix {
    latitude: f64,
    longitude: f64,
    speed: f64,
    time: f64,
}

/// carrot `navigation_position`: the phone/vehicle fusion in carrotMan,
/// else the freshest GPS fix.
fn position(sub: &SubMaster, now: f64) -> Option<Fix> {
    let state = &sub.state;
    if let Ok(topic) = state.topic("carrotMan") {
        if topic.seen && topic.valid && (0. ..0.5).contains(&(now - topic.receive_time)) {
            if let Some(event::Which::CarrotMan(Ok(c))) = topic.event().ok().and_then(|e| e.which().ok()) {
                let c: carrot_man::Reader = c;
                let source = c.get_position_source().ok().and_then(|s| s.to_str().ok()).unwrap_or("");
                let fix = c.get_position_fix_mono_time() as f64 * 1e-9;
                let pos = c.get_position_mono_time() as f64 * 1e-9;
                let (lat, lon, angle, speed) = (f64::from(c.get_x_pos_lat()), f64::from(c.get_x_pos_lon()), f64::from(c.get_x_pos_angle()), f64::from(c.get_x_pos_speed()));
                if (source.is_empty() || c.get_position_heading_valid())
                    && fix > 0. && (0. ..2.).contains(&(now - fix)) && (0. ..0.5).contains(&(now - pos))
                    && [lat, lon, angle, speed].iter().all(|v| v.is_finite())
                    && lat.abs() <= MAX_LATITUDE && lon.abs() <= 180.
                {
                    return Some(Fix { latitude: lat, longitude: lon, speed: (speed / 3.6).max(0.), time: pos });
                }
            }
        }
    }
    let mut best: Option<Fix> = None;
    for name in ["gpsLocation", "gpsLocationExternal"] {
        let Ok(topic) = state.topic(name) else { continue };
        if !topic.seen || !topic.valid || !(0. ..2.).contains(&(now - topic.receive_time)) {
            continue;
        }
        let Ok(e) = topic.event() else { continue };
        let gps = match e.which() {
            Ok(event::Which::GpsLocation(Ok(g))) | Ok(event::Which::GpsLocationExternal(Ok(g))) => g,
            _ => continue,
        };
        let (lat, lon, speed) = (gps.get_latitude(), gps.get_longitude(), f64::from(gps.get_speed()));
        if gps.get_has_fix() && [lat, lon, speed].iter().all(|v| v.is_finite()) && lat.abs() <= MAX_LATITUDE && lon.abs() <= 180.
            && best.as_ref().is_none_or(|b| topic.receive_time > b.time)
        {
            best = Some(Fix { latitude: lat, longitude: lon, speed: speed.max(0.), time: topic.receive_time });
        }
    }
    best
}

fn main() -> Result<(), Box<dyn std::error::Error>> {
    let stop = Arc::new(AtomicBool::new(false));
    for signal in [signal_hook::consts::SIGTERM, signal_hook::consts::SIGINT] {
        signal_hook::flag::register(signal, Arc::clone(&stop))?;
    }
    // Auxiliary work: little cores, low priority (as carrot's mapsd).
    let _ = rustix::process::nice(15);
    let params = openpilot_params::Params::for_runtime()?;
    let mut sub = SubMaster::for_runtime(
        &["carrotMan", "gpsLocation", "gpsLocationExternal", "navRoute", "navInstructionCarrot"],
        Options::default(),
    )?;
    let mut renderer = MapRenderer::default();
    let (mut remote_key, source) = remote(&params);
    let clock: Box<dyn FnMut() -> f64 + Send> = Box::new(monotonic);
    let mut tiles = HybridTiles::new(&store_root(), source, clock)?;
    let mut route: Vec<[f64; 2]> = Vec::new();
    let mut route_id = 0u64;
    let mut tracker: Option<RouteTracker> = None;
    let (mut next_render, mut next_config, mut was_ready) = (0f64, 0f64, false);
    let black = vec![0u8; MAP_BYTES];
    let out = Path::new(output::PATH);
    while !stop.load(Ordering::Relaxed) {
        sub.update(Duration::from_millis(100))?;
        let now = monotonic();
        if now >= next_config {
            next_config = now + 5.;
            let (key, source) = remote(&params);
            if key != remote_key {
                remote_key = key;
                let clock: Box<dyn FnMut() -> f64 + Send> = Box::new(monotonic);
                tiles = HybridTiles::new(&store_root(), source, clock)?;
            }
        }
        if let Ok(topic) = sub.state.topic("navRoute") {
            if topic.updated {
                if let Some(event::Which::NavRoute(Ok(r))) = topic.event().ok().and_then(|e| e.which().ok()) {
                    let id = r.get_carrot_route_id();
                    if id != route_id {
                        route_id = id;
                        route = if topic.valid {
                            r.get_coordinates().map(|c| c.iter().map(|p| [f64::from(p.get_longitude()), f64::from(p.get_latitude())]).collect()).unwrap_or_default()
                        } else {
                            Vec::new()
                        };
                        tracker = if id != 0 { RouteTracker::new(route.clone()) } else { None };
                        next_render = 0.;
                    }
                }
            }
        }
        let demand = std::fs::metadata(DEMAND_PATH)
            .and_then(|m| m.modified())
            .ok()
            .and_then(|m| std::time::SystemTime::now().duration_since(m).ok())
            .is_some_and(|age| age < Duration::from_secs(3));
        let fix = position(&sub, now);
        let instruction = sub.state.topic("navInstructionCarrot").ok().is_some_and(|t| t.seen && t.valid && (0. ..2.).contains(&(now - t.receive_time)));
        let ready = demand && tracker.is_some() && fix.is_some() && instruction;
        if now < next_render && !(was_ready && !ready) {
            was_ready = ready;
            continue;
        }
        was_ready = ready;
        let started = monotonic();
        let mut map = Map { route_id, rendered_ns: (started * 1e9) as u64, valid: false, image: black.clone() };
        if ready {
            let fix = fix.expect("ready");
            let advance = fix.speed * (now - fix.time).max(0.);
            if let Some((lon, lat, bearing)) = tracker.as_mut().and_then(|t| t.update_pose(fix.longitude, fix.latitude, advance, 50.)) {
                match renderer.render(&mut tiles, Position { latitude: lat, longitude: lon, bearing }, Some(&route)) {
                    Ok(result) if result.valid() => {
                        map.valid = true;
                        map.image = result.image;
                    }
                    Ok(_) => {}
                    Err(error) => eprintln!("mapsd: map unavailable: {error}"),
                }
            }
        }
        let elapsed = monotonic() - started;
        if elapsed >= 0.5 {
            map.valid = false;
            map.image.fill(0);
        }
        // At most 10 Hz, ~10% of one core (carrot render_interval).
        let interval = (elapsed * 10.).max(0.1);
        next_render = monotonic() + if map.valid { interval } else { interval.max(1.) };
        if demand || ready {
            if let Err(error) = output::write(out, &map) {
                eprintln!("mapsd: write map: {error}");
            }
        }
    }
    Ok(())
}
