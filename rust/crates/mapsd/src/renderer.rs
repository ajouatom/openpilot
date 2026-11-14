//! Deterministic CPU rasterization of roads and the route (carrot
//! `renderer.py`): 1 m per pixel, the ego three quarters down the frame,
//! white roads whose casings widen the stroke, bridges cutting over the
//! roads beneath, a grey route and a white ego arrow.
use crate::{vector_tile::Road, Error};
use std::collections::{HashMap, VecDeque};

pub const SIZE: usize = 256;
pub const ZOOM: u8 = 15;
const METERS_PER_PIXEL: f64 = 1.0;
const FORWARD_OFFSET: f64 = 64.0;
const EARTH_CIRCUMFERENCE: f64 = 40_075_016.685_578_49;
const ROAD_INTENSITY: u8 = 255;
const ROUTE_INTENSITY: u8 = 128;
const ROUTE_WIDTH: f64 = 5.0;
const CUT_WIDTH: f64 = 3.0;
const MAX_ROAD_DISTANCE: f64 = 100.0;
const ARROW: [[f64; 2]; 4] = [[0., -9.], [7., 7.], [0., 3.], [-7., 7.]];
const ARROW_OUTLINE: f64 = 2.0;

fn interpolate_width(zoom: f64, stops: &[f64]) -> f64 {
    let (zooms, widths): (Vec<f64>, Vec<f64>) = stops.chunks(2).map(|c| (c[0], c[1])).unzip();
    if zoom <= zooms[0] {
        return widths[0];
    }
    if zoom >= *zooms.last().expect("stops") {
        return *widths.last().expect("stops");
    }
    for i in 0..zooms.len() - 1 {
        if zooms[i] <= zoom && zoom <= zooms[i + 1] {
            let fraction = (1.5f64.powf(zoom - zooms[i]) - 1.) / (1.5f64.powf(zooms[i + 1] - zooms[i]) - 1.);
            return widths[i] + fraction * (widths[i + 1] - widths[i]);
        }
    }
    widths[0]
}

/// (interior, casing) width stops per training road class.
const STYLES: [(&[f64], &[f64]); 5] = [
    (&[5., 0.75, 18., 32.], &[10., 1., 18., 2.]),            // motorway, trunk, primary
    (&[5., 0.1, 18., 26.], &[10., 0.75, 18., 2.]),           // secondary, tertiary
    (&[12., 0.5, 14., 2., 18., 18.], &[12., 0.75, 20., 2.]), // streets
    (&[14., 0.5, 18., 12.], &[12., 0.75, 20., 2.]),          // service, minor links
    (&[14., 1., 18., 12.], &[12., 0.75, 20., 2.]),           // track
];

fn road_style(road: &Road) -> Option<usize> {
    let detail = road.detail.as_str();
    if matches!(detail, "platform" | "pier" | "raceway") {
        return None;
    }
    if matches!(detail, "motorway" | "trunk" | "primary") || (detail.is_empty() && road.kind == "highway") {
        return Some(0);
    }
    if matches!(detail, "secondary" | "tertiary") || (detail.is_empty() && road.kind == "major_road") {
        return Some(1);
    }
    if detail == "track" {
        return Some(4);
    }
    if matches!(detail, "secondary_link" | "tertiary_link" | "service" | "driveway" | "parking_aisle" | "alley")
        || (road.kind == "other" && detail != "living_street")
    {
        return Some(3);
    }
    Some(2)
}

#[derive(Clone, Copy, Debug)]
pub struct Position {
    pub latitude: f64,
    pub longitude: f64,
    /// Degrees clockwise from north.
    pub bearing: f64,
}

/// One road segment in normalized Web Mercator.
#[derive(Clone, Copy)]
struct Segment {
    start: [f64; 2],
    end: [f64; 2],
    style: u8,
    level: i8,
    /// Round caps at (start, end): only at a polyline's ends.
    caps: [bool; 2],
}

pub fn mercator(longitude: f64, latitude: f64) -> [f64; 2] {
    let y = (1. - latitude.to_radians().tan().asinh() / std::f64::consts::PI) / 2.;
    [(longitude + 180.) / 360., y]
}

fn project(point: [f64; 2], origin: [f64; 2], position: &Position) -> [f64; 2] {
    let scale = EARTH_CIRCUMFERENCE * position.latitude.to_radians().cos();
    let east = ((point[0] - origin[0] + 0.5).rem_euclid(1.0) - 0.5) * scale;
    let north = -(point[1] - origin[1]) * scale;
    let angle = position.bearing.to_radians();
    let right = east * angle.cos() - north * angle.sin();
    let forward = east * angle.sin() + north * angle.cos();
    [
        SIZE as f64 / 2. + right / METERS_PER_PIXEL,
        SIZE as f64 / 2. - (forward - FORWARD_OFFSET) / METERS_PER_PIXEL,
    ]
}

/// Strokes with round joins and one pixel of analytic edge antialiasing.
/// Max blending unless `overlay`, which replaces the pixels beneath.
fn draw_segment(image: &mut [u8], start: [f64; 2], end: [f64; 2], radius: f64, intensity: u8, overlay: bool, caps: [bool; 2]) {
    let reach = radius + 1.;
    let x0 = (start[0].min(end[0]) - reach).floor().max(0.) as i64;
    let y0 = (start[1].min(end[1]) - reach).floor().max(0.) as i64;
    let x1 = ((start[0].max(end[0]) + reach).ceil() as i64).min(SIZE as i64 - 1);
    let y1 = ((start[1].max(end[1]) + reach).ceil() as i64).min(SIZE as i64 - 1);
    if x1 < x0 || y1 < y0 {
        return;
    }
    let (dx, dy) = (end[0] - start[0], end[1] - start[1]);
    let length2 = (dx * dx + dy * dy).max(1e-12);
    for y in y0..=y1 {
        let py = y as f64 + 0.5 - start[1];
        let row = y as usize * SIZE;
        for x in x0..=x1 {
            let px = x as f64 + 0.5 - start[0];
            let along = (px * dx + py * dy) / length2;
            let t = along.clamp(0., 1.);
            let mut alpha = (radius + 0.5 - (px - t * dx).hypot(py - t * dy)).clamp(0., 1.);
            if (!caps[0] && along < 0.) || (!caps[1] && along > 1.) {
                alpha = 0.;
            }
            let pixel = &mut image[row + x as usize];
            if overlay {
                *pixel = (f64::from(*pixel) * (1. - alpha) + f64::from(intensity) * alpha).round() as u8;
            } else {
                *pixel = (*pixel).max((alpha * f64::from(intensity)).round() as u8);
            }
        }
    }
}

fn draw_vehicle(image: &mut [u8], center: [f64; 2]) {
    let arrow = ARROW.map(|p| [p[0] + center[0], p[1] + center[1]]);
    let x0 = (arrow.iter().map(|p| p[0]).fold(f64::INFINITY, f64::min).floor() - 2.).max(0.) as usize;
    let y0 = (arrow.iter().map(|p| p[1]).fold(f64::INFINITY, f64::min).floor() - 2.).max(0.) as usize;
    let x1 = ((arrow.iter().map(|p| p[0]).fold(f64::NEG_INFINITY, f64::max).ceil() + 2.) as usize).min(SIZE);
    let y1 = ((arrow.iter().map(|p| p[1]).fold(f64::NEG_INFINITY, f64::max).ceil() + 2.) as usize).min(SIZE);
    let inside = |a: [f64; 2], b: [f64; 2], c: [f64; 2], px: f64, py: f64| {
        let side = |p: [f64; 2], q: [f64; 2]| (q[0] - p[0]) * (py - p[1]) - (q[1] - p[1]) * (px - p[0]);
        let s = [side(a, b), side(b, c), side(c, a)];
        s.iter().all(|v| *v >= 0.) || s.iter().all(|v| *v <= 0.)
    };
    for y in y0..y1 {
        for x in x0..x1 {
            let (px, py) = (x as f64 + 0.5, y as f64 + 0.5);
            if inside(arrow[0], arrow[1], arrow[2], px, py) || inside(arrow[0], arrow[2], arrow[3], px, py) {
                image[y * SIZE + x] = 255;
            }
        }
    }
    for i in 0..4 {
        draw_segment(image, arrow[i], arrow[(i + 1) % 4], ARROW_OUTLINE / 2., 0, true, [true, true]);
    }
}

fn nearest_distance(point: [f64; 2], segments: &[([f64; 2], [f64; 2])]) -> f64 {
    segments
        .iter()
        .map(|(s, e)| {
            let d = [e[0] - s[0], e[1] - s[1]];
            let t = (((point[0] - s[0]) * d[0] + (point[1] - s[1]) * d[1]) / (d[0] * d[0] + d[1] * d[1]).max(1e-12)).clamp(0., 1.);
            (s[0] + t * d[0] - point[0]).hypot(s[1] + t * d[1] - point[1])
        })
        .fold(f64::INFINITY, f64::min)
}

/// Decompressed MVT tiles at ZOOM, from wherever they are kept.
pub trait TileSource {
    /// Ok(None): no such tile (sea, outside the archive). Err: unavailable now.
    fn tile(&mut self, z: u8, x: u64, y: u64) -> Result<Option<Vec<u8>>, Error>;
}

pub struct RenderResult {
    pub image: Vec<u8>,
    pub complete: bool,
    pub route_present: bool,
    pub road_distance: f64,
}

impl RenderResult {
    pub fn valid(&self) -> bool {
        self.complete && self.route_present && self.road_distance <= MAX_ROAD_DISTANCE
    }
}

/// Decoded segments per tile; a small LRU.
pub struct MapRenderer {
    cache: HashMap<(u64, u64), Option<Vec<Segment>>>,
    order: VecDeque<(u64, u64)>,
}

impl Default for MapRenderer {
    fn default() -> Self {
        Self { cache: HashMap::new(), order: VecDeque::new() }
    }
}

impl MapRenderer {
    fn roads(&mut self, source: &mut dyn TileSource, x: u64, y: u64) -> Result<Option<&Vec<Segment>>, Error> {
        if !self.cache.contains_key(&(x, y)) {
            let segments = match source.tile(ZOOM, x, y)? {
                None => None,
                Some(data) => {
                    let scale = (1u64 << ZOOM) as f64;
                    let mut segments = Vec::new();
                    for road in crate::vector_tile::decode_roads(&data)? {
                        let Some(style) = road_style(&road) else { continue };
                        let level = if road.bridge { 1 } else if road.tunnel { -1 } else { 0 };
                        let n = road.points.len() - 1;
                        let to = |p: [f64; 2]| [(p[0] / f64::from(road.extent) + x as f64) / scale, (p[1] / f64::from(road.extent) + y as f64) / scale];
                        for i in 0..n {
                            segments.push(Segment {
                                start: to(road.points[i]),
                                end: to(road.points[i + 1]),
                                style: style as u8,
                                level,
                                caps: [i == 0, i == n - 1],
                            });
                        }
                    }
                    Some(segments)
                }
            };
            if self.order.len() >= 128 {
                if let Some(old) = self.order.pop_front() {
                    self.cache.remove(&old);
                }
            }
            self.order.push_back((x, y));
            self.cache.insert((x, y), segments);
        }
        Ok(self.cache[&(x, y)].as_ref())
    }

    /// Tiles covering the view for every heading, and whether all exist on the map.
    pub fn view_tiles(longitude: f64, latitude: f64) -> (Vec<(u64, u64)>, bool) {
        let origin = mercator(longitude, latitude);
        let radius = (SIZE as f64 * METERS_PER_PIXEL / 2f64.sqrt() + FORWARD_OFFSET + 16.)
            / (EARTH_CIRCUMFERENCE * latitude.to_radians().cos());
        let count = 1i64 << ZOOM;
        let low = [((origin[0] - radius) * count as f64).floor() as i64, ((origin[1] - radius) * count as f64).floor() as i64];
        let high = [((origin[0] + radius) * count as f64).floor() as i64, ((origin[1] + radius) * count as f64).floor() as i64];
        let mut tiles = Vec::new();
        for y in low[1]..=high[1] {
            for x in low[0]..=high[0] {
                if (0..count).contains(&y) {
                    tiles.push((x.rem_euclid(count) as u64, y as u64));
                }
            }
        }
        let expected = ((high[0] - low[0] + 1) * (high[1] - low[1] + 1)) as usize;
        let complete = tiles.len() == expected;
        (tiles, complete)
    }

    pub fn render(&mut self, source: &mut dyn TileSource, position: Position, route: Option<&[[f64; 2]]>) -> Result<RenderResult, Error> {
        let mut image = vec![0u8; SIZE * SIZE];
        let (wanted, mut complete) = Self::view_tiles(position.longitude, position.latitude);
        let origin = mercator(position.longitude, position.latitude);
        let style_zoom = (position.latitude.to_radians().cos() * EARTH_CIRCUMFERENCE / (256. * METERS_PER_PIXEL)).log2() - 1.;
        let widths: Vec<[f64; 2]> = STYLES.iter().map(|(i, c)| [interpolate_width(style_zoom, i), interpolate_width(style_zoom, c)]).collect();
        let mut projected: Vec<([f64; 2], [f64; 2], f64, i8, [bool; 2])> = Vec::new();
        for (x, y) in wanted {
            match self.roads(source, x, y)? {
                None => complete = false,
                Some(segments) => {
                    for s in segments {
                        let width = widths[s.style as usize][0] + if s.level < 0 { 0. } else { 2. * widths[s.style as usize][1] };
                        projected.push((project(s.start, origin, &position), project(s.end, origin, &position), width, s.level, s.caps));
                    }
                }
            }
        }
        let ego = [SIZE as f64 / 2., SIZE as f64 / 2. + FORWARD_OFFSET / METERS_PER_PIXEL];
        let lines: Vec<_> = projected.iter().map(|p| (p.0, p.1)).collect();
        let road_distance = nearest_distance(ego, &lines) * METERS_PER_PIXEL;
        let min_level = projected.iter().map(|p| p.3).min().unwrap_or(0);
        for level in [-1i8, 0, 1] {
            let chosen: Vec<_> = projected.iter().filter(|p| p.3 == level).collect();
            if chosen.is_empty() {
                continue;
            }
            if level > min_level {
                for p in &chosen {
                    draw_segment(&mut image, p.0, p.1, (p.2 + CUT_WIDTH) / 2., 0, true, [!p.4[0], !p.4[1]]);
                }
            }
            for p in &chosen {
                draw_segment(&mut image, p.0, p.1, p.2 / 2., ROAD_INTENSITY, false, [true, true]);
            }
        }
        let route_present = route.is_some_and(|r| r.len() >= 2);
        if let Some(route) = route.filter(|r| r.len() >= 2) {
            let points: Vec<_> = route.iter().map(|p| project(mercator(p[0], p[1]), origin, &position)).collect();
            for pair in points.windows(2) {
                draw_segment(&mut image, pair[0], pair[1], ROUTE_WIDTH / 2., ROUTE_INTENSITY, true, [true, true]);
            }
        }
        draw_vehicle(&mut image, ego);
        Ok(RenderResult { image, complete, route_present, road_distance })
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    struct Empty;
    impl TileSource for Empty {
        fn tile(&mut self, _: u8, _: u64, _: u64) -> Result<Option<Vec<u8>>, Error> {
            Ok(Some(Vec::new()))
        }
    }

    #[test]
    fn renders_the_route_and_ego_arrow_on_an_empty_map() {
        let mut renderer = MapRenderer::default();
        let position = Position { latitude: 37.5, longitude: 127.0, bearing: 0. };
        let route = [[127.0, 37.499], [127.0, 37.501]];
        let result = renderer.render(&mut Empty, position, Some(&route)).unwrap();
        assert!(result.complete && result.route_present);
        assert!(!result.valid(), "no road near the ego");
        // The route runs straight up the centre column; the arrow is white.
        assert_eq!(result.image[100 * SIZE + 128], ROUTE_INTENSITY);
        assert_eq!(result.image[(128 + 64) * SIZE + 128], 255);
        assert_eq!(result.image[10 * SIZE + 10], 0);
    }

    #[test]
    fn projection_puts_the_ego_three_quarters_down() {
        let position = Position { latitude: 37.5, longitude: 127.0, bearing: 90. };
        let origin = mercator(127.0, 37.5);
        let p = project(origin, origin, &position);
        assert!((p[0] - 128.).abs() < 1e-9 && (p[1] - 192.).abs() < 1e-9);
    }
}
