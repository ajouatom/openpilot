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

pub struct RouteTracker {
    route: Vec<[f64; 2]>,
    distance: Vec<f64>,
    progress: f64,
}

impl RouteTracker {
    /// `route`: (longitude, latitude) rows, at least two.
    pub fn new(route: Vec<[f64; 2]>) -> Option<Self> {
        (route.len() >= 2 && route.len() <= 4096 && route.iter().flatten().all(|v| v.is_finite()))
            .then(|| Self {
                distance: cumulative_distance(&route),
                route,
                progress: f64::NEG_INFINITY,
            })
    }

    fn project(&self, longitude: f64, latitude: f64, segments: std::ops::Range<usize>) -> f64 {
        let origin = self.route[0];
        let scale = [111_320. * origin[1].to_radians().cos(), 110_540.];
        let to = |p: [f64; 2]| [(p[0] - origin[0]) * scale[0], (p[1] - origin[1]) * scale[1]];
        let point = to([longitude, latitude]);
        let mut best = (f64::INFINITY, segments.start, 0.);
        for i in segments {
            let (a, b) = (to(self.route[i]), to(self.route[i + 1]));
            let ab = [b[0] - a[0], b[1] - a[1]];
            let along = (((point[0] - a[0]) * ab[0] + (point[1] - a[1]) * ab[1])
                / (ab[0] * ab[0] + ab[1] * ab[1]).max(1e-12))
            .clamp(0., 1.);
            let d = (a[0] + along * ab[0] - point[0]).powi(2)
                + (a[1] + along * ab[1] - point[1]).powi(2);
            if d < best.0 {
                best = (d, i, along);
            }
        }
        let (_, segment, along) = best;
        let reached =
            self.distance[segment] + along * (self.distance[segment + 1] - self.distance[segment]);
        self.progress.max(reached)
    }

    /// (longitude, latitude, bearing) on the route at `progress`.
    pub fn pose(&self, progress: f64) -> (f64, f64, f64) {
        let end = *self.distance.last().expect("two points");
        let ends = [
            progress - HEADING_HALF_WINDOW,
            progress,
            progress + HEADING_HALF_WINDOW,
        ]
        .map(|v| v.clamp(0., end));
        let [behind, center, ahead] = ends.map(|distance| {
            let right = self.distance.partition_point(|&v| v <= distance);
            if right == self.route.len() {
                return self.route[right - 1];
            }
            if right == 0 {
                return self.route[0];
            }
            let left = right - 1;
            let fraction =
                (distance - self.distance[left]) / (self.distance[right] - self.distance[left]);
            std::array::from_fn(|axis| {
                self.route[left][axis]
                    + fraction * (self.route[right][axis] - self.route[left][axis])
            })
        });
        let bearing = ((ahead[0] - behind[0]) * behind[1].to_radians().cos())
            .atan2(ahead[1] - behind[1])
            .to_degrees()
            .rem_euclid(360.);
        (center[0], center[1], bearing)
    }

    /// Commit progress only when the pose stays within `max_distance` m of the fix.
    pub fn update_pose(
        &mut self,
        longitude: f64,
        latitude: f64,
        advance: f64,
        max_distance: f64,
    ) -> Option<(f64, f64, f64)> {
        if !longitude.is_finite() || !latitude.is_finite() || !advance.is_finite() {
            return None;
        }
        let end = self.route.len() - 1;
        let start = self.distance[1..].partition_point(|d| *d < self.progress + WINDOW.0);
        let stop = self.distance[..end].partition_point(|d| *d <= self.progress + WINDOW.1);
        let nearby = start..stop;
        // Prefer the local road at crossings. Reacquire further along the same route
        // after a gap, applying the same 50 m caller limit to both candidates.
        for segments in [nearby, start..end] {
            if segments.is_empty() {
                continue;
            }
            let progress = self.project(longitude, latitude, segments);
            let pose = self.pose(progress + advance);
            let distance = ((pose.0 - longitude) * 111_320. * pose.1.to_radians().cos())
                .hypot((pose.1 - latitude) * 110_540.);
            if distance <= max_distance {
                self.progress = progress;
                return Some(pose);
            }
        }
        None
    }
}

/// Which `navRoute` the map follows.
///
/// carrot-nav's carrot_man stamps every navRoute with a `carrotRouteId` that
/// changes only when the selected route is replaced, and mapsd.py follows
/// that id. Our carrot_man is carrot-wip's: it never sets the id (always 0)
/// and interleaves the navd route (empty every ~5 s without a Mapbox route)
/// with the external navigation's route (~1 Hz, with gaps up to ~20 s).
/// Without an id the geometry is the identity and an empty route is ignored;
/// the fresh-instruction gate and the 50 m on-route check already turn the
/// map black when guidance ends or the car leaves the route.
#[derive(Default)]
pub struct RouteSelection {
    id: u64,
    stamped: bool,
}

impl RouteSelection {
    /// The route to switch to, if this message replaces the current one.
    pub fn update(
        &mut self,
        carrot_route_id: u64,
        route: &[[f64; 2]],
    ) -> Option<(u64, Vec<[f64; 2]>)> {
        if carrot_route_id != 0 {
            self.stamped = true;
            if carrot_route_id == self.id {
                return None;
            }
            self.id = carrot_route_id;
            return Some((carrot_route_id, route.to_vec()));
        }
        if self.stamped || route.is_empty() {
            return None;
        }
        let id = geometry_id(route);
        if id == self.id {
            return None;
        }
        self.id = id;
        Some((id, route.to_vec()))
    }
}

/// A nonzero identity of a route geometry.
fn geometry_id(route: &[[f64; 2]]) -> u64 {
    use std::hash::{Hash, Hasher};
    let mut hasher = std::collections::hash_map::DefaultHasher::new();
    for point in route {
        point[0].to_bits().hash(&mut hasher);
        point[1].to_bits().hash(&mut hasher);
    }
    hasher.finish() | 1
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn unstamped_routes_follow_geometry_and_ignore_empty_heartbeats() {
        let a = [[127.0, 37.0], [127.0, 37.01]];
        let b = [[127.0, 37.0], [127.01, 37.0]];
        let mut selection = RouteSelection::default();
        let (id_a, route) = selection.update(0, &a).unwrap();
        assert!(id_a != 0 && route == a);
        assert!(selection.update(0, &a).is_none(), "same geometry");
        assert!(selection.update(0, &[]).is_none(), "navd's empty route");
        assert!(selection.update(0, &a).is_none());
        let (id_b, _) = selection.update(0, &b).unwrap();
        assert_ne!(id_a, id_b);
    }

    #[test]
    fn stamped_routes_follow_the_id_as_carrot_nav() {
        let a = [[127.0, 37.0], [127.0, 37.01]];
        let mut selection = RouteSelection::default();
        assert_eq!(selection.update(7, &a).unwrap().0, 7);
        assert!(selection.update(7, &a).is_none());
        // A new id replaces the route, an empty one too (carrot-nav clears).
        assert!(selection.update(8, &[]).unwrap().1.is_empty());
        // Once stamped, unstamped messages do not take over.
        assert!(selection.update(0, &a).is_none());
    }

    #[test]
    fn reacquires_after_a_gap_on_the_same_route() {
        let route = (0..100)
            .map(|i| [127., 37. + f64::from(i) * 0.0001])
            .collect();
        let mut tracker = RouteTracker::new(route).unwrap();
        tracker.update_pose(127., 37.001, 0., 50.).unwrap();
        assert!(tracker.update_pose(128., 37.004, 0., 50.).is_none());
        let (_, lat, _) = tracker.update_pose(127., 37.004, 0., 50.).unwrap();
        assert!((lat - 37.004).abs() < 1e-6);
    }

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
