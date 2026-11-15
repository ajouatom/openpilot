//! openpilot-mapsd: carrot's `selfdrive/navigation/mapsd.py`. Renders the
//! N16 route map while something wants it (an eGPU, a navigation graph, the
//! cluster HUD's nav-model view, a Carrot Web page) and a route, a position
//! and a fresh instruction exist; black otherwise. The image also goes to a
//! small PNG preview with the reason for what it shows.
use openpilot_cereal::{car_capnp::car_state, custom_capnp::carrot_man, log_capnp::event};
use openpilot_mapsd::{
    output::{self, Map, DEMAND_PATH, MAP_BYTES},
    preview,
    remote::{Archive, Mapbox},
    renderer::{MapRenderer, Position},
    route::{RouteSelection, RouteTracker},
    tiles::{HybridTiles, Remote},
    Error,
};
use openpilot_logging::{log_site, producer::{Factory, Logger}, record::Record, Fields, Value};
use openpilot_navd::position::{CarPark, CarrotFix, GpsFix, ParkedPosition, Sample};
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

/// Why the route map was or was not drawn, summarised every 30 s while
/// something wants the map: a `mapsdStatus` event in the rlog and a line on
/// the console. Without it a drive cannot tell a black map from a real one.
#[derive(Default)]
struct Status {
    since: f64,
    counts: Vec<(&'static str, i128)>,
}

impl Status {
    const INTERVAL: f64 = 30.;

    fn count(&mut self, reason: &'static str, tiles: &str, now: f64, logger: Option<&mut Logger>) {
        if self.counts.is_empty() {
            self.since = now;
        }
        match self.counts.iter_mut().find(|(name, _)| *name == reason) {
            Some((_, n)) => *n += 1,
            None => self.counts.push((reason, 1)),
        }
        if now - self.since < Self::INTERVAL {
            return;
        }
        let summary: Vec<String> = self.counts.iter().map(|(name, n)| format!("{name}={n}")).collect();
        eprintln!("mapsd: maps in the last {:.0} s (tiles from {tiles}): {}", now - self.since, summary.join(" "));
        if let Some(logger) = logger {
            let mut fields = Fields::default();
            for (name, n) in &self.counts {
                fields.insert((*name).to_owned(), Value::Integer(*n));
            }
            fields.insert("seconds".to_owned(), Value::Float(now - self.since));
            fields.insert("tiles".to_owned(), Value::Text(tiles.to_owned()));
            // Evidence only: a logging failure must not stop the map.
            if let Ok(record) = Record::event("mapsdStatus", Vec::new(), fields) {
                let _ = logger.emit(log_site!(), record);
            }
        }
        self.counts.clear();
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

/// What the shared parked-position fusion (`openpilot_navd::position`) reads,
/// as mapsd.py's SubMaster gives it.
fn sample(sub: &SubMaster, now: f64) -> Sample {
    let state = &sub.state;
    let mut sample = Sample {
        now,
        ..Sample::default()
    };
    if let Ok(topic) = state.topic("carrotMan") {
        if topic.seen {
            if let Some(event::Which::CarrotMan(Ok(c))) = topic.event().ok().and_then(|e| e.which().ok()) {
                let c: carrot_man::Reader = c;
                sample.carrot = Some(CarrotFix {
                    valid: topic.valid,
                    received: topic.receive_time,
                    position_source_empty: c
                        .get_position_source()
                        .ok()
                        .and_then(|s| s.to_str().ok())
                        .unwrap_or("")
                        .is_empty(),
                    heading_valid: c.get_position_heading_valid(),
                    fix_mono_ns: c.get_position_fix_mono_time(),
                    position_mono_ns: c.get_position_mono_time(),
                    latitude: f64::from(c.get_x_pos_lat()),
                    longitude: f64::from(c.get_x_pos_lon()),
                    angle: f64::from(c.get_x_pos_angle()),
                    speed: f64::from(c.get_x_pos_speed()),
                });
            }
        }
    }
    for name in ["gpsLocation", "gpsLocationExternal"] {
        let Ok(topic) = state.topic(name) else { continue };
        if !topic.seen {
            continue;
        }
        let Ok(e) = topic.event() else { continue };
        let gps = match e.which() {
            Ok(event::Which::GpsLocation(Ok(g))) | Ok(event::Which::GpsLocationExternal(Ok(g))) => g,
            _ => continue,
        };
        sample.gps.push(GpsFix {
            valid: topic.valid,
            has_fix: gps.get_has_fix(),
            received: topic.receive_time,
            log_mono_time: topic.log_mono_time,
            latitude: gps.get_latitude(),
            longitude: gps.get_longitude(),
            bearing: f64::from(gps.get_bearing_deg()),
            speed: f64::from(gps.get_speed()),
        });
    }
    if let Ok(topic) = state.topic("carState") {
        if topic.seen {
            if let Some(event::Which::CarState(Ok(car))) = topic.event().ok().and_then(|e| e.which().ok()) {
                sample.car = Some(CarPark {
                    valid: topic.valid,
                    alive: topic.alive,
                    received: topic.receive_time,
                    park: car.get_gear_shifter() == Ok(car_state::GearShifter::Park),
                    standstill: car.get_standstill(),
                    v_ego: f64::from(car.get_v_ego()),
                });
            }
        }
    }
    sample
}

/// mapsd.py's demand, checked once a second: an eGPU, a Jetlink/driving nav
/// graph (DEMAND_PATH), the cluster HUD's nav-model view, or a web page.
fn demand(params: &openpilot_params::Params) -> bool {
    let model = std::fs::metadata(DEMAND_PATH)
        .and_then(|m| m.modified())
        .ok()
        .and_then(|m| std::time::SystemTime::now().duration_since(m).ok())
        .is_some_and(|age| age < Duration::from_secs(3));
    params.get_bool("UsbGpuActive").unwrap_or(false)
        || model
        || text(params, "ClusterHudScreenMode") == "7"
        || preview::fresh(Path::new(preview::WEB_DEMAND_PATH))
}

/// Auxiliary work shares only the little cores, at normal priority: never
/// compete with camera, control, radar, driving model or DM on cores 4..7.
fn little_cores() {
    let mut set = rustix::thread::CpuSet::new();
    for core in 0..4 {
        set.set(core);
    }
    if let Err(error) = rustix::thread::sched_setaffinity(None, &set) {
        eprintln!("mapsd: little-core affinity: {error}");
    }
    let _ = rustix::process::nice(15);
}

fn main() -> Result<(), Box<dyn std::error::Error>> {
    let stop = Arc::new(AtomicBool::new(false));
    for signal in [signal_hook::consts::SIGTERM, signal_hook::consts::SIGINT] {
        signal_hook::flag::register(signal, Arc::clone(&stop))?;
    }
    little_cores();
    let params = openpilot_params::Params::for_runtime()?;
    let mut sub = SubMaster::for_runtime(
        &["carrotMan", "gpsLocation", "gpsLocationExternal", "navRoute", "navInstructionCarrot", "carState"],
        Options::default(),
    )?;
    let mut renderer = MapRenderer::default();
    let (mut remote_key, source) = remote(&params);
    let clock: Box<dyn FnMut() -> f64 + Send> = Box::new(monotonic);
    let mut tiles = HybridTiles::new(&store_root(), source, clock)?;
    let mut route: Vec<[f64; 2]> = Vec::new();
    let mut route_id = 0u64;
    let mut selection = RouteSelection::default();
    let mut status = Status::default();
    let mut logger = Factory::for_runtime().ok().map(|factory| factory.logger());
    let mut tracker: Option<RouteTracker> = None;
    let mut position = ParkedPosition::default();
    let (mut next_render, mut next_config, mut was_ready) = (0f64, 0f64, false);
    let (mut demand_checked, mut wanted, mut retry_at, mut next_preview) = (0f64, false, 0f64, 0f64);
    let black = vec![0u8; MAP_BYTES];
    let out = Path::new(output::PATH);
    let preview_dir = Path::new(preview::PREVIEW_PATH).parent().unwrap_or(Path::new("/dev/shm"));
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
                retry_at = 0.;
                next_render = 0.;
            }
        }
        if let Ok(topic) = sub.state.topic("navRoute") {
            if topic.updated {
                if let Some(event::Which::NavRoute(Ok(r))) = topic.event().ok().and_then(|e| e.which().ok()) {
                    let coordinates: Vec<[f64; 2]> = if topic.valid {
                        r.get_coordinates().map(|c| c.iter().map(|p| [f64::from(p.get_longitude()), f64::from(p.get_latitude())]).collect()).unwrap_or_default()
                    } else {
                        Vec::new()
                    };
                    if let Some((id, selected)) = selection.update(r.get_carrot_route_id(), &coordinates) {
                        route_id = id;
                        route = selected;
                        tracker = RouteTracker::new(route.clone());
                        next_render = 0.;
                    }
                }
            }
        }
        let fix = position.update(&sample(&sub, now), 0.5);
        let instruction = sub.state.topic("navInstructionCarrot").ok().is_some_and(|t| t.seen && t.valid && (0. ..2.).contains(&(now - t.receive_time)));
        if now >= demand_checked {
            wanted = demand(&params);
            demand_checked = now + 1.;
        }
        let ready = wanted && tracker.is_some() && fix.is_some() && instruction;
        if now < next_render && (ready || !was_ready) {
            was_ready = ready;
            continue;
        }
        was_ready = ready;
        let started = monotonic();
        let mut map = Map { route_id, rendered_ns: (started * 1e9) as u64, valid: false, image: black.clone() };
        let mut reason = if !wanted {
            "idle"
        } else if tracker.is_none() {
            "route"
        } else if fix.is_none() {
            "position"
        } else if !instruction {
            "instruction"
        } else {
            "tiles"
        };
        if ready && now >= retry_at {
            let fix = fix.expect("ready");
            let advance = fix.speed * (now - fix.time).max(0.);
            match tracker.as_mut().and_then(|t| t.update_pose(fix.longitude, fix.latitude, advance, 50.)) {
                Some((lon, lat, bearing)) => {
                    match renderer.render(&mut tiles, Position { latitude: lat, longitude: lon, bearing }, Some(&route)) {
                        Ok(result) if result.valid() => {
                            map.valid = true;
                            map.image = result.image;
                            reason = "ready";
                        }
                        Ok(_) => reason = "tiles",
                        Err(error) => {
                            eprintln!("mapsd: map unavailable; retry in 30 seconds: {error}");
                            retry_at = monotonic() + 30.;
                        }
                    }
                }
                None => reason = "route_position",
            }
        }
        let finished = monotonic();
        let elapsed = finished - started;
        if elapsed >= 0.5 {
            map.valid = false;
            map.image.fill(0);
            reason = "slow";
        }
        // At most 10 Hz, ~10% of one core (carrot render_interval).
        let interval = (elapsed * 10.).max(0.1);
        next_render = finished + if map.valid { interval } else { interval.max(1.) };
        if wanted {
            let tiles = if remote_key.starts_with("mapbox:") { "mapbox" } else if remote_key.is_empty() { "none" } else { "url" };
            status.count(reason, tiles, finished, logger.as_mut());
        }
        if wanted || ready {
            if let Err(error) = output::write(out, &map) {
                eprintln!("mapsd: write map: {error}");
            }
        }
        // The shared preview for Carrot Web and the cluster HUD, twice a
        // second. Both of its readers create demand, so with none nobody can
        // be looking and the PNG work is skipped.
        if wanted && finished >= next_preview {
            if let Err(error) = preview::write(&map.image, 256, 256, reason, preview_dir) {
                eprintln!("mapsd: write preview: {error}");
            }
            next_preview = finished + 0.5;
        }
    }
    Ok(())
}
