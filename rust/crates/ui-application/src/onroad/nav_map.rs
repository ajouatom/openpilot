//! `ShowNavMap`: a Mapbox map (1) or satellite imagery (2) at the right of
//! the driving view, heading up, with the route and the car.
//!
//! carrot's old map panel rendered Mapbox styles with Qt/MapLibre and was
//! dropped for its CPU cost. Here mapsd keeps raster tiles around the car on
//! the device (`openpilot_mapsd::display`); this decodes a tile file on a
//! worker thread only when it comes near the panel and uploads it once. Every
//! frame then only places up to 16 textures, the route and the car on the
//! GPU, and reads a parameter, the demand file and the route file at most
//! once a second.
use crate::{context::Context, paint::color, params::Read, state::messages};
use openpilot_mapsd::display::{self, Style, TILE_PIXELS, ZOOM};
use openpilot_startup_ui::renderer::DecodedImage;
use openpilot_ui_framework::{
    draw::{Draw, ImageDraw, PixelBuffer, TextDraw, TextureResource},
    geometry::{Point, Rect},
    text::Font,
    text_layout::float,
    Error,
};
use std::{
    collections::HashMap,
    path::{Path, PathBuf},
    sync::mpsc,
    thread,
    time::SystemTime,
};

const PARAM_EVERY: f64 = 2.0;
const FILE_EVERY: f64 = 1.0;
/// A missing tile file is looked for again after this long.
const RECHECK: f64 = 1.0;
const STATS_PER_FRAME: usize = 4;
const UPLOADS_PER_FRAME: usize = 1;
const MAX_TEXTURES: usize = 30;
/// Metres the panel spans from top to bottom.
const VIEW_METERS: f64 = 300.;
/// The car sits this far down the panel, so more of the road ahead shows.
const EGO_Y: f64 = 0.72;
/// Older positions are extrapolated along the heading for at most this long.
const EXTRAPOLATE: f64 = 1.0;

type Key = (u64, u64);

enum Tile {
    Missing { recheck: f64 },
    Decoding,
    Ready(Box<dyn TextureResource>),
    Failed,
}

struct Decoder {
    requests: mpsc::Sender<(Key, PathBuf)>,
    results: mpsc::Receiver<(Key, Result<DecodedImage, String>)>,
}

impl Decoder {
    fn start() -> Result<Self, Error> {
        let (requests, inbox) = mpsc::channel::<(Key, PathBuf)>();
        let (outbox, results) = mpsc::channel();
        // Ends when the panel (the request sender) goes away.
        thread::Builder::new()
            .name("ui-nav-map-decode".into())
            .spawn(move || {
                for (key, path) in inbox {
                    let image = DecodedImage::load(&path).map_err(|e| e.to_string());
                    if outbox.send((key, image)).is_err() {
                        break;
                    }
                }
            })?;
        Ok(Self { requests, results })
    }
}

#[derive(Clone, Copy, Debug, PartialEq)]
struct Pose {
    longitude: f64,
    latitude: f64,
    bearing: f64,
    speed: f64,
}

impl Pose {
    /// `seconds` along the heading at the pose's speed.
    fn ahead(self, seconds: f64) -> Self {
        let distance = self.speed * seconds.clamp(0., EXTRAPOLATE);
        let bearing = self.bearing.to_radians();
        Self {
            latitude: self.latitude + distance * bearing.cos() / 111_320.,
            longitude: self.longitude
                + distance * bearing.sin() / (111_320. * self.latitude.to_radians().cos()),
            ..self
        }
    }
}

/// Where a world tile pixel lands on the screen: relative to the car,
/// scaled, then turned so the heading points up.
fn place(ego: Point, ego_world: [f64; 2], world: [f64; 2], scale: f64, bearing: f64) -> Point {
    let (sin, cos) = bearing.to_radians().sin_cos();
    let dx = (world[0] - ego_world[0]) * scale;
    let dy = (world[1] - ego_world[1]) * scale;
    Point {
        x: ego.x + float(dx * cos + dy * sin),
        y: ego.y + float(-dx * sin + dy * cos),
    }
}

/// The panel at the right of the driving view, below the top HUD row.
fn panel(area: Rect) -> Rect {
    let height = (f64::from(area.height) * 0.62).min(f64::from(area.width) * 0.3);
    let width = height;
    Rect {
        x: float(f64::from(area.x + area.width) - width - 24.),
        y: float(f64::from(area.y + area.height) - height - 150.),
        width: float(width),
        height: float(height),
    }
}

fn modified(path: &Path) -> Option<SystemTime> {
    std::fs::metadata(path).and_then(|m| m.modified()).ok()
}

pub struct NavMap {
    root: PathBuf,
    style: Option<Style>,
    has_token: bool,
    next_param: f64,
    next_demand: f64,
    next_route: f64,
    route_stamp: Option<SystemTime>,
    /// The route in tile pixels.
    route: Vec<[f64; 2]>,
    tiles: HashMap<Key, Tile>,
    decoder: Option<Decoder>,
    raw: Option<(Pose, f64)>,
}

impl NavMap {
    pub fn new() -> Self {
        Self {
            root: display::store_root(),
            style: None,
            has_token: false,
            next_param: 0.,
            next_demand: 0.,
            next_route: 0.,
            route_stamp: None,
            route: Vec::new(),
            tiles: HashMap::new(),
            decoder: None,
            raw: None,
        }
    }

    fn refresh(&mut self, context: &Context, now: f64) {
        if now >= self.next_param {
            self.next_param = now + PARAM_EVERY;
            let style =
                Style::from_mode(i64::from(context.params.integer("ShowNavMap").unwrap_or(0)));
            if style != self.style {
                self.style = style;
                // Another style, other images: the textures go; the decoder
                // thread stops with the panel.
                self.tiles.clear();
                self.route_stamp = None;
                if style.is_none() {
                    self.decoder = None;
                    self.route.clear();
                }
            }
            self.has_token = context
                .params
                .bytes("MapboxPublicKey")
                .ok()
                .flatten()
                .is_some_and(|token| !token.trim_ascii().is_empty());
        }
        let Some(style) = self.style else {
            return;
        };
        if now >= self.next_demand {
            self.next_demand = now + FILE_EVERY;
            if let Err(error) = display::write_demand(Path::new(display::DEMAND_PATH), style) {
                eprintln!("ui: nav map demand: {error}");
            }
        }
        if now >= self.next_route {
            self.next_route = now + FILE_EVERY;
            let path = Path::new(display::ROUTE_PATH);
            let stamp = modified(path);
            if stamp != self.route_stamp {
                self.route_stamp = stamp;
                self.route = display::read_route(path)
                    .unwrap_or_default()
                    .into_iter()
                    .map(|[lon, lat]| display::world_pixel(lon, lat))
                    .collect();
            }
        }
    }

    /// carrot's own position (carrotMan), moved on along the heading between
    /// its updates so the map glides instead of stepping.
    fn pose(&mut self, context: &Context, now: f64) -> Option<Pose> {
        {
            let messages = context.messages.borrow();
            let carrot = messages::carrot_man(&messages.state).ok()?;
            let pose = Pose {
                latitude: f64::from(carrot.get_x_pos_lat()),
                longitude: f64::from(carrot.get_x_pos_lon()),
                bearing: f64::from(carrot.get_x_pos_angle()),
                speed: f64::from(carrot.get_x_pos_speed()).max(0.) / 3.6,
            };
            let usable = pose.latitude.is_finite()
                && pose.longitude.is_finite()
                && pose.bearing.is_finite()
                && (pose.latitude != 0. || pose.longitude != 0.)
                && pose.latitude.abs() < 85.;
            if !usable {
                self.raw = None;
                return None;
            }
            if self.raw.is_none_or(|(raw, _)| raw != pose) {
                self.raw = Some((pose, now));
            }
        }
        self.raw.map(|(pose, at)| pose.ahead(now - at))
    }

    fn load(&mut self, needed: &[Key], now: f64, draw: &mut dyn Draw) -> Result<(), Error> {
        let Some(style) = self.style else {
            return Ok(());
        };
        if self.decoder.is_none() {
            self.decoder = Some(Decoder::start()?);
        }
        let Some(decoder) = &self.decoder else {
            return Ok(());
        };
        for _ in 0..UPLOADS_PER_FRAME {
            let Ok((key, image)) = decoder.results.try_recv() else {
                break;
            };
            // A late result for a tile that left the panel is dropped.
            if !matches!(self.tiles.get(&key), Some(Tile::Decoding)) {
                continue;
            }
            let tile = match image {
                Ok(image) => Tile::Ready(draw.upload_image(PixelBuffer {
                    dimensions: (image.width, image.height),
                    rgba: &image.rgba,
                })?),
                Err(error) => {
                    eprintln!("ui: nav map tile {key:?}: {error}");
                    Tile::Failed
                }
            };
            self.tiles.insert(key, tile);
        }
        let mut stats = 0;
        for &key in needed {
            let due = match self.tiles.get(&key) {
                None => true,
                Some(Tile::Missing { recheck }) => now >= *recheck,
                Some(_) => false,
            };
            if !due || stats >= STATS_PER_FRAME {
                continue;
            }
            stats += 1;
            let sent = display::stored_tile(&self.root, style, ZOOM, key.0, key.1)
                .is_some_and(|path| decoder.requests.send((key, path)).is_ok());
            let tile = if sent {
                Tile::Decoding
            } else {
                Tile::Missing {
                    recheck: now + RECHECK,
                }
            };
            self.tiles.insert(key, tile);
        }
        if self.tiles.len() > MAX_TEXTURES {
            self.tiles.retain(|key, _| needed.contains(key));
        }
        Ok(())
    }

    fn label(draw: &mut dyn Draw, text: &str, position: Point, size: f32) -> Result<(), Error> {
        let shadow = Point {
            x: position.x + 2.,
            y: position.y + 2.,
        };
        for (at, tint) in [
            (shadow, color(0, 0, 0, 200)),
            (position, color(235, 235, 235, 230)),
        ] {
            draw.text(TextDraw {
                font: Font::SemiBold,
                text,
                position: at,
                size,
                spacing: 0.,
                color: tint,
            })?;
        }
        Ok(())
    }

    /// Draws the panel inside `area` (the driving view); leaves the scissor
    /// on `area`.
    pub fn draw(
        &mut self,
        context: &Context,
        area: Rect,
        now: f64,
        draw: &mut dyn Draw,
    ) -> Result<(), Error> {
        self.refresh(context, now);
        let Some(style) = self.style else {
            return Ok(());
        };
        let rect = panel(area);
        draw.scissor(Some(rect))?;
        draw.rounded(rect, 0.0, color(18, 22, 30, 235))?;
        let pose = self.pose(context, now);
        let mut drawn = 0;
        if let Some(pose) = pose {
            let ego = Point {
                x: rect.x + rect.width / 2.,
                y: rect.y + float(f64::from(rect.height) * EGO_Y),
            };
            let ego_world = display::world_pixel(pose.longitude, pose.latitude);
            let scale =
                f64::from(rect.height) / (VIEW_METERS / display::meters_per_pixel(pose.latitude));
            // Every tile that can show in the panel, at any heading.
            let reach = f64::from(rect.width / 2.).hypot(f64::from(rect.height) * EGO_Y) / scale;
            let needed = display::tiles_near(ego_world, reach);
            self.load(&needed, now, draw)?;
            let size = float(TILE_PIXELS * scale);
            for key in &needed {
                let Some(Tile::Ready(texture)) = self.tiles.get(key) else {
                    continue;
                };
                // Corners keep their unwrapped x near the car at the antimeridian.
                let wrap = f64::from(1u32 << ZOOM) * TILE_PIXELS;
                let mut corner = [key.0 as f64 * TILE_PIXELS, key.1 as f64 * TILE_PIXELS];
                corner[0] += ((ego_world[0] - corner[0]) / wrap).round() * wrap;
                let at = place(ego, ego_world, corner, scale, pose.bearing);
                let (width, height) = texture.dimensions();
                draw.image(ImageDraw {
                    id: texture.id(),
                    source: Rect {
                        x: 0.,
                        y: 0.,
                        width: width as f32,
                        height: height as f32,
                    },
                    destination: Rect {
                        x: at.x,
                        y: at.y,
                        width: size,
                        height: size,
                    },
                    origin: Point::default(),
                    rotation: float(-pose.bearing),
                    tint: color(255, 255, 255, 255),
                })?;
                drawn += 1;
            }
            let far = f64::from(rect.width.max(rect.height)) * 1.5;
            let near = |p: Point| f64::from(p.x - ego.x).hypot(f64::from(p.y - ego.y)) < far;
            let mut previous: Option<Point> = None;
            for world in &self.route {
                let point = place(ego, ego_world, *world, scale, pose.bearing);
                if let Some(start) = previous {
                    if near(start) || near(point) {
                        draw.line(start, point, 12., color(40, 140, 255, 235))?;
                    }
                }
                previous = Some(point);
            }
            draw.line(
                ego,
                Point {
                    x: ego.x,
                    y: ego.y - 34.,
                },
                8.,
                color(255, 255, 255, 255),
            )?;
            draw.circle(ego, 17., color(255, 255, 255, 255))?;
            draw.circle(ego, 12., color(30, 120, 255, 255))?;
        }
        let status = if !self.has_token {
            Some("Mapbox token needed (Carrot Web)")
        } else if pose.is_none() {
            Some("Waiting for position")
        } else if drawn == 0 {
            Some("Loading map")
        } else {
            None
        };
        if let Some(text) = status {
            Self::label(
                draw,
                text,
                Point {
                    x: rect.x + 24.,
                    y: rect.y + 24.,
                },
                34.,
            )?;
        }
        let credit = match style {
            Style::Map => "© Mapbox © OpenStreetMap",
            Style::Satellite => "© Mapbox © OpenStreetMap © Maxar",
        };
        Self::label(
            draw,
            credit,
            Point {
                x: rect.x + 14.,
                y: rect.y + rect.height - 34.,
            },
            22.,
        )?;
        draw.border(rect, 0.0, color(255, 255, 255, 70))?;
        draw.scissor(Some(area))?;
        Ok(())
    }
}

impl Default for NavMap {
    fn default() -> Self {
        Self::new()
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    const EGO: Point = Point { x: 500., y: 500. };

    fn close(point: Point, x: f32, y: f32) -> bool {
        (point.x - x).abs() < 1e-3 && (point.y - y).abs() < 1e-3
    }

    #[test]
    fn heading_points_up() {
        let ego_world = [1000., 1000.];
        // Heading north: a point north (smaller y) is straight above.
        assert!(close(
            place(EGO, ego_world, [1000., 900.], 1., 0.),
            500.,
            400.
        ));
        // Heading east: a point east is straight above, north is to the left.
        assert!(close(
            place(EGO, ego_world, [1100., 1000.], 1., 90.),
            500.,
            400.
        ));
        assert!(close(
            place(EGO, ego_world, [1000., 900.], 1., 90.),
            400.,
            500.
        ));
        // Heading south: north is below; the scale applies.
        assert!(close(
            place(EGO, ego_world, [1000., 900.], 2., 180.),
            500.,
            700.
        ));
    }

    #[test]
    fn extrapolation_moves_along_the_heading_and_stops() {
        let pose = Pose {
            longitude: 127.,
            latitude: 37.5,
            bearing: 90.,
            speed: 20.,
        };
        let moved = pose.ahead(0.5);
        assert!((moved.latitude - 37.5).abs() < 1e-9);
        let metres = (moved.longitude - 127.) * 111_320. * 37.5f64.to_radians().cos();
        assert!((metres - 10.).abs() < 1e-6, "{metres}");
        assert_eq!(pose.ahead(5.), pose.ahead(EXTRAPOLATE));
        assert_eq!(pose.ahead(-1.), pose);
    }

    #[test]
    fn panel_sits_inside_the_view_on_the_right() {
        let area = Rect {
            x: 30.,
            y: 30.,
            width: 2100.,
            height: 1020.,
        };
        let rect = panel(area);
        assert!(rect.x > area.x + area.width / 2.);
        assert!(rect.x + rect.width <= area.x + area.width);
        assert!(rect.y >= area.y && rect.y + rect.height <= area.y + area.height);
        assert!((rect.width - rect.height).abs() < 1e-3);
    }
}
