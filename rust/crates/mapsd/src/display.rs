//! The onroad screen's map panel (`ShowNavMap`: 1 map, 2 satellite).
//!
//! carrot once showed a Mapbox map, or its satellite imagery, to the right of
//! the camera; the Qt/MapLibre renderer behind it was dropped for its CPU
//! cost. Here the panel is made of raster tiles: mapsd, which already has the
//! network, the Mapbox token and the little cores, keeps the tiles around the
//! car in a small store on the device, and the UI decodes a tile only when it
//! enters the panel, then moves, rotates and overlays it on the GPU. A tile
//! costs one download per 30 days and one decode per drive past it.
use crate::renderer::mercator;
use std::{
    fs, io,
    path::{Path, PathBuf},
    time::{Duration, SystemTime},
};

/// Rewritten by the UI every second while the panel shows: "1" or "2".
pub const DEMAND_PATH: &str = "/dev/shm/carrot-nav-display-demand";
/// The route the panel draws, `[[lon, lat], ...]` as JSON; mapsd writes it
/// while the panel shows and the route changes.
pub const ROUTE_PATH: &str = "/dev/shm/carrot-nav-display-route.json";
/// One 512-pixel tile is ~240 m at 37° N.
pub const ZOOM: u8 = 17;
pub const TILE_PIXELS: f64 = 512.;
/// Tiles reaching within this distance of the car are fetched, nearest first.
pub const RADIUS_M: f64 = 350.;
pub const MAX_AGE: Duration = Duration::from_secs(30 * 24 * 3600);
/// Disk kept for the panel's tiles (~60 KB each); oldest go first.
pub const STORE_LIMIT: u64 = 256 << 20;
const DEMAND_FRESH: Duration = Duration::from_secs(3);
const EARTH_CIRCUMFERENCE: f64 = 40_075_016.686;

#[derive(Clone, Copy, Debug, PartialEq, Eq, Hash)]
pub enum Style {
    Map,
    Satellite,
}

impl Style {
    pub fn from_mode(mode: i64) -> Option<Self> {
        match mode {
            1 => Some(Self::Map),
            2 => Some(Self::Satellite),
            _ => None,
        }
    }
    pub fn mode(self) -> u8 {
        match self {
            Self::Map => 1,
            Self::Satellite => 2,
        }
    }
    pub fn name(self) -> &'static str {
        match self {
            Self::Map => "map",
            Self::Satellite => "satellite",
        }
    }
    /// The Mapbox URL of a 512-pixel tile, without the token.
    pub fn url(self, z: u8, x: u64, y: u64) -> String {
        match self {
            // Static Tiles API: a 512 tile covers the usual z/x/y square.
            Self::Map => format!(
                "https://api.mapbox.com/styles/v1/mapbox/navigation-night-v1/tiles/512/{z}/{x}/{y}"
            ),
            Self::Satellite => {
                format!("https://api.mapbox.com/v4/mapbox.satellite/{z}/{x}/{y}@2x.jpg80")
            }
        }
    }
}

pub fn store_root() -> PathBuf {
    if Path::new("/TICI").exists() {
        PathBuf::from("/data/navigation/raster")
    } else {
        PathBuf::from(std::env::var("HOME").unwrap_or_default()).join(".comma/navigation/raster")
    }
}

fn tile_stem(root: &Path, style: Style, z: u8, x: u64, y: u64) -> PathBuf {
    root.join(style.name()).join(format!("{z}_{x}_{y}"))
}

/// The stored image of a tile. raylib picks its decoder by the extension.
pub fn stored_tile(root: &Path, style: Style, z: u8, x: u64, y: u64) -> Option<PathBuf> {
    let stem = tile_stem(root, style, z, x, y);
    ["jpg", "png"]
        .into_iter()
        .map(|extension| stem.with_extension(extension))
        .find(|path| path.is_file())
}

/// The file a downloaded tile goes to, by its magic bytes; None for neither
/// a JPEG nor a PNG.
pub fn tile_file(
    root: &Path,
    style: Style,
    z: u8,
    x: u64,
    y: u64,
    bytes: &[u8],
) -> Option<PathBuf> {
    let extension = if bytes.starts_with(&[0xff, 0xd8, 0xff]) {
        "jpg"
    } else if bytes.starts_with(b"\x89PNG\r\n\x1a\n") {
        "png"
    } else {
        return None;
    };
    Some(tile_stem(root, style, z, x, y).with_extension(extension))
}

/// Tile pixels per world unit (`mercator`'s 0..1) at ZOOM.
pub fn world_scale() -> f64 {
    f64::from(1u32 << ZOOM) * TILE_PIXELS
}

/// Metres per tile pixel at `latitude`.
pub fn meters_per_pixel(latitude: f64) -> f64 {
    EARTH_CIRCUMFERENCE * latitude.to_radians().cos() / world_scale()
}

/// The point's tile pixel at ZOOM.
pub fn world_pixel(longitude: f64, latitude: f64) -> [f64; 2] {
    let [x, y] = mercator(longitude, latitude);
    [x * world_scale(), y * world_scale()]
}

/// Tiles whose square comes within `radius` tile pixels of `center`,
/// nearest first.
pub fn tiles_near(center: [f64; 2], radius: f64) -> Vec<(u64, u64)> {
    let count = 1i64 << ZOOM;
    let low = |v: f64| ((v - radius) / TILE_PIXELS).floor() as i64;
    let high = |v: f64| ((v + radius) / TILE_PIXELS).floor() as i64;
    let mut tiles = Vec::new();
    for y in low(center[1]).max(0)..=high(center[1]).min(count - 1) {
        for x in low(center[0])..=high(center[0]) {
            let nearest = |v: f64, start: i64| {
                let start = start as f64 * TILE_PIXELS;
                v.clamp(start, start + TILE_PIXELS)
            };
            let dx = nearest(center[0], x) - center[0];
            let dy = nearest(center[1], y) - center[1];
            let distance = dx.hypot(dy);
            if distance <= radius {
                tiles.push((distance, (x.rem_euclid(count) as u64, y as u64)));
            }
        }
    }
    tiles.sort_by(|a, b| a.0.total_cmp(&b.0));
    tiles.into_iter().map(|(_, tile)| tile).collect()
}

/// The tiles mapsd keeps ready around the car.
pub fn tiles_around(longitude: f64, latitude: f64) -> Vec<(u64, u64)> {
    tiles_near(
        world_pixel(longitude, latitude),
        RADIUS_M / meters_per_pixel(latitude),
    )
}

pub fn write_demand(path: &Path, style: Style) -> io::Result<()> {
    fs::write(path, [b'0' + style.mode()])
}

/// The panel's style while the UI keeps asking.
pub fn read_demand(path: &Path) -> Option<Style> {
    let modified = fs::metadata(path).and_then(|m| m.modified()).ok()?;
    if SystemTime::now().duration_since(modified).ok()? >= DEMAND_FRESH {
        return None;
    }
    let bytes = fs::read(path).ok()?;
    Style::from_mode(i64::from(*bytes.first()?) - i64::from(b'0'))
}

pub fn write_route(path: &Path, route: &[[f64; 2]]) -> io::Result<()> {
    let temporary = path.with_extension("tmp");
    fs::write(
        &temporary,
        serde_json::to_vec(route).map_err(io::Error::other)?,
    )?;
    fs::rename(&temporary, path)
}

pub fn read_route(path: &Path) -> Option<Vec<[f64; 2]>> {
    serde_json::from_slice(&fs::read(path).ok()?).ok()
}

/// Oldest files first until the store fits `limit`.
pub fn trim(root: &Path, limit: u64) -> io::Result<()> {
    let mut files = Vec::new();
    for style in fs::read_dir(root)?.flatten() {
        for file in fs::read_dir(style.path()).into_iter().flatten().flatten() {
            if let Ok(meta) = file.metadata() {
                files.push((
                    meta.modified().unwrap_or(SystemTime::UNIX_EPOCH),
                    meta.len(),
                    file.path(),
                ));
            }
        }
    }
    let mut total: u64 = files.iter().map(|f| f.1).sum();
    files.sort();
    for (_, size, path) in files {
        if total <= limit {
            break;
        }
        fs::remove_file(path)?;
        total -= size;
    }
    Ok(())
}

#[cfg(feature = "native")]
pub use fetch::Fetcher;

#[cfg(feature = "native")]
mod fetch {
    use super::*;
    use crate::Error;
    use std::{collections::HashMap, io::Read};

    /// One tile per call, so the model map in the same loop never waits on
    /// more than one request.
    pub struct Fetcher {
        token: String,
        root: PathBuf,
        agent: ureq::Agent,
        /// When a tile was last tried; a failed tile waits 60 s.
        tried: HashMap<(Style, u64, u64), f64>,
        offline_until: f64,
        written: u64,
    }

    impl Fetcher {
        pub fn new(token: String, root: PathBuf) -> Self {
            let agent = ureq::Agent::config_builder()
                .http_status_as_error(false)
                .timeout_global(Some(Duration::from_secs(5)))
                .max_redirects(5)
                .build()
                .into();
            Self {
                token,
                root,
                agent,
                tried: HashMap::new(),
                offline_until: 0.,
                written: 0,
            }
        }

        pub fn token(&self) -> &str {
            &self.token
        }

        fn fresh(path: &Path) -> bool {
            fs::metadata(path)
                .and_then(|m| m.modified())
                .ok()
                .and_then(|m| SystemTime::now().duration_since(m).ok())
                .is_some_and(|age| age < MAX_AGE)
        }

        /// Downloads the nearest missing or stale tile around the position.
        /// Returns whether a request was made.
        pub fn step(&mut self, style: Style, longitude: f64, latitude: f64, now: f64) -> bool {
            if self.token.is_empty() || now < self.offline_until {
                return false;
            }
            self.tried.retain(|_, at| now - *at < 60.);
            let next = tiles_around(longitude, latitude)
                .into_iter()
                .find(|&(x, y)| {
                    !self.tried.contains_key(&(style, x, y))
                        && stored_tile(&self.root, style, ZOOM, x, y)
                            .is_none_or(|p| !Self::fresh(&p))
                });
            let Some((x, y)) = next else {
                return false;
            };
            self.tried.insert((style, x, y), now);
            match self.download(style, x, y) {
                Ok(()) => {}
                Err(error) => {
                    eprintln!("mapsd: panel tile unavailable; retry in 30 seconds: {error}");
                    self.offline_until = now + 30.;
                }
            }
            true
        }

        fn download(&mut self, style: Style, x: u64, y: u64) -> Result<(), Error> {
            // The token is a query parameter: keep the URL out of errors and logs.
            let mut response = self
                .agent
                .get(&style.url(ZOOM, x, y))
                .query("access_token", &self.token)
                .call()
                .map_err(|_| Error::Remote("Mapbox raster tiles unavailable".into()))?;
            match response.status().as_u16() {
                200 => {}
                // Outside the imagery: nothing to draw, nothing to retry soon.
                404 => return Ok(()),
                status => {
                    return Err(Error::Remote(format!(
                        "Mapbox raster tiles answered {status}"
                    )))
                }
            }
            let mut bytes = Vec::new();
            response
                .body_mut()
                .as_reader()
                .take((2 << 20) + 1)
                .read_to_end(&mut bytes)
                .map_err(|e| Error::Remote(e.kind().to_string()))?;
            if bytes.len() > 2 << 20 {
                return Err(Error::Remote("raster tile exceeds 2 MB".into()));
            }
            let path = tile_file(&self.root, style, ZOOM, x, y, &bytes)
                .ok_or(Error::Remote("raster tile is neither JPEG nor PNG".into()))?;
            if let Some(parent) = path.parent() {
                fs::create_dir_all(parent)?;
            }
            // A refreshed tile may change its format; never keep both.
            if let Some(old) = stored_tile(&self.root, style, ZOOM, x, y) {
                if old != path {
                    let _ = fs::remove_file(old);
                }
            }
            let temporary = path.with_extension("tmp");
            fs::write(&temporary, &bytes)?;
            fs::rename(&temporary, &path)?;
            self.written += bytes.len() as u64;
            if self.written > 16 << 20 {
                self.written = 0;
                trim(&self.root, STORE_LIMIT)?;
            }
            Ok(())
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn modes() {
        assert_eq!(Style::from_mode(0), None);
        assert_eq!(Style::from_mode(1), Some(Style::Map));
        assert_eq!(Style::from_mode(2), Some(Style::Satellite));
        assert_eq!(Style::from_mode(3), None);
    }

    #[test]
    fn scale_near_seoul() {
        let mpp = meters_per_pixel(37.5);
        assert!((0.46..0.48).contains(&mpp), "{mpp}");
    }

    #[test]
    fn nearest_tile_first_and_all_within_radius() {
        let (lon, lat) = (127.0276, 37.4979);
        let center = world_pixel(lon, lat);
        let own = (
            (center[0] / TILE_PIXELS) as u64,
            (center[1] / TILE_PIXELS) as u64,
        );
        let tiles = tiles_around(lon, lat);
        assert_eq!(tiles[0], own);
        // 350 m is ~1.5 tiles: at most a 4x4 block, at least the 3x3 around.
        assert!((9..=16).contains(&tiles.len()), "{}", tiles.len());
        for dx in -1i64..=1 {
            for dy in -1i64..=1 {
                let tile = ((own.0 as i64 + dx) as u64, (own.1 as i64 + dy) as u64);
                assert!(tiles.contains(&tile));
            }
        }
    }

    #[test]
    fn files_by_magic() {
        let root = Path::new("/r");
        assert_eq!(
            tile_file(root, Style::Satellite, 17, 1, 2, &[0xff, 0xd8, 0xff, 0xe0]),
            Some(PathBuf::from("/r/satellite/17_1_2.jpg"))
        );
        assert_eq!(
            tile_file(root, Style::Map, 17, 1, 2, b"\x89PNG\r\n\x1a\n...."),
            Some(PathBuf::from("/r/map/17_1_2.png"))
        );
        assert_eq!(tile_file(root, Style::Map, 17, 1, 2, b"<html>"), None);
    }

    #[test]
    fn demand_and_route_round_trip() {
        let dir = std::env::temp_dir().join(format!("nav-display-{}", std::process::id()));
        fs::create_dir_all(&dir).unwrap();
        let demand = dir.join("demand");
        assert_eq!(read_demand(&demand), None);
        write_demand(&demand, Style::Satellite).unwrap();
        assert_eq!(read_demand(&demand), Some(Style::Satellite));
        let route = dir.join("route.json");
        write_route(&route, &[[127.0, 37.5], [127.001, 37.501]]).unwrap();
        assert_eq!(read_route(&route).unwrap().len(), 2);
        let _ = fs::remove_dir_all(dir);
    }

    #[test]
    fn trim_keeps_the_newest() {
        let dir = std::env::temp_dir().join(format!("nav-trim-{}", std::process::id()));
        fs::create_dir_all(dir.join("map")).unwrap();
        for (index, name) in ["a.jpg", "b.jpg", "c.jpg"].iter().enumerate() {
            let path = dir.join("map").join(name);
            fs::write(&path, vec![0u8; 100]).unwrap();
            let time = SystemTime::UNIX_EPOCH + Duration::from_secs(1_000 + index as u64);
            fs::File::options()
                .write(true)
                .open(&path)
                .unwrap()
                .set_modified(time)
                .unwrap();
        }
        trim(&dir, 200).unwrap();
        assert!(!dir.join("map/a.jpg").exists());
        assert!(dir.join("map/b.jpg").exists() && dir.join("map/c.jpg").exists());
        let _ = fs::remove_dir_all(dir);
    }
}
