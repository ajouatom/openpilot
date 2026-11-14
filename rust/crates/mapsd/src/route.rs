//! Route-relative map pose and progress (carrot `route.py`): the map is
//! rendered on the route, centred on the ego's projection and headed along
//! the route's +-50 m chord, as the training renderer did.
pub const MAX_LATITUDE: f64 = 85.051_128_78;
const WINDOW: (f64, f64) = (-30., 100.);
const HEADING_HALF_WINDOW: f64 = 50.;

pub fn cumulative_distance(route: &[[f64; 2]]) -> Vec<f64> {
    let mut out = vec![0.; route.len()];
    for i in 1..route.len() {
        let (lon1, lat1) = (route[i - 1][0].to_radians(), route[i - 1][1].to_radians());
        let (lon2, lat2) = (route[i][0].to_radians(), route[i][1].to_radians());
        let h = ((lat2 - lat1) / 2.).sin().powi(2)
            + lat1.cos() * lat2.cos() * ((lon2 - lon1) / 2.).sin().powi(2);
        out[i] = out[i - 1] + 2. * 6_371_008.8 * h.sqrt().atan2((1. - h).sqrt());
    }
    out
}

fn interp(x: f64, xs: &[f64], ys: &[f64]) -> f64 {
    if x <= xs[0] {
        return ys[0];
    }
    let last = xs.len() - 1;
    if x >= xs[last] {
        return ys[last];
    }
    let i = xs.partition_point(|&v| v <= x).max(1);
    let (x0, x1) = (xs[i - 1], xs[i]);
    if x1 == x0 {
        return ys[i];
    }
    ys[i - 1] + (ys[i] - ys[i - 1]) * (x - x0) / (x1 - x0)
}

pub struct RouteTracker {
    route: Vec<[f64; 2]>,
    distance: Vec<f64>,
    progress: f64,
}

impl RouteTracker {
    /// `route`: (longitude, latitude) rows, at least two.
    pub fn new(route: Vec<[f64; 2]>) -> Option<Self> {
        (route.len() >= 2 && route.len() <= 4096 && route.iter().flatten().all(|v| v.is_finite()))
            .then(|| Self { distance: cumulative_distance(&route), route, progress: f64::NEG_INFINITY })
    }

    fn project(&mut self, longitude: f64, latitude: f64) -> f64 {
        let origin = self.route[0];
        let scale = [111_320. * origin[1].to_radians().cos(), 110_540.];
        let to = |p: [f64; 2]| [(p[0] - origin[0]) * scale[0], (p[1] - origin[1]) * scale[1]];
        let point = to([longitude, latitude]);
        let n = self.route.len() - 1;
        let near: Vec<usize> = (0..n)
            .filter(|&i| {
                self.distance[i + 1] >= self.progress + WINDOW.0 && self.distance[i] <= self.progress + WINDOW.1
            })
            .collect();
        let near = if near.is_empty() { (0..n).collect() } else { near };
        let mut best = (f64::INFINITY, near[0], 0.);
        for &i in &near {
            let (a, b) = (to(self.route[i]), to(self.route[i + 1]));
            let ab = [b[0] - a[0], b[1] - a[1]];
            let along = (((point[0] - a[0]) * ab[0] + (point[1] - a[1]) * ab[1])
                / (ab[0] * ab[0] + ab[1] * ab[1]).max(1e-12))
            .clamp(0., 1.);
            let d = (a[0] + along * ab[0] - point[0]).powi(2) + (a[1] + along * ab[1] - point[1]).powi(2);
            if d < best.0 {
                best = (d, i, along);
            }
        }
        let (_, segment, along) = best;
        let reached = self.distance[segment] + along * (self.distance[segment + 1] - self.distance[segment]);
        self.progress = self.progress.max(reached);
        self.progress
    }

    /// (longitude, latitude, bearing) on the route at `progress`.
    pub fn pose(&self, progress: f64) -> (f64, f64, f64) {
        let end = *self.distance.last().expect("two points");
        let ends = [progress - HEADING_HALF_WINDOW, progress, progress + HEADING_HALF_WINDOW].map(|v| v.clamp(0., end));
        let lon: Vec<f64> = self.route.iter().map(|p| p[0]).collect();
        let lat: Vec<f64> = self.route.iter().map(|p| p[1]).collect();
        let lons = ends.map(|e| interp(e, &self.distance, &lon));
        let lats = ends.map(|e| interp(e, &self.distance, &lat));
        let bearing = ((lons[2] - lons[0]) * lats[0].to_radians().cos())
            .atan2(lats[2] - lats[0])
            .to_degrees()
            .rem_euclid(360.);
        (lons[1], lats[1], bearing)
    }

    /// Commit progress only when the pose stays within `max_distance` m of the fix.
    pub fn update_pose(&mut self, longitude: f64, latitude: f64, advance: f64, max_distance: f64) -> Option<(f64, f64, f64)> {
        let previous = self.progress;
        let progress = self.project(longitude, latitude);
        let pose = self.pose(progress + advance);
        let distance = ((pose.0 - longitude) * 111_320. * pose.1.to_radians().cos())
            .hypot((pose.1 - latitude) * 110_540.);
        if !distance.is_finite() || distance > max_distance {
            self.progress = previous;
            return None;
        }
        Some(pose)
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn tracks_progress_along_a_northbound_route() {
        let mut tracker = RouteTracker::new(vec![[127.0, 37.0], [127.0, 37.01]]).unwrap();
        let (lon, lat, bearing) = tracker.update_pose(127.0001, 37.005, 0., 50.).unwrap();
        assert!((lon - 127.0).abs() < 1e-9 && (lat - 37.005).abs() < 1e-5);
        assert!(bearing < 1. || bearing > 359.);
        // A fix far off the route is rejected and does not move progress.
        assert!(tracker.update_pose(128.0, 37.005, 0., 50.).is_none());
    }
}
