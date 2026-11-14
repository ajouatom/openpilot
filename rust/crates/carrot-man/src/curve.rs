//! Source: curve_speed.py, geometry envelope and fresh-frame release policy.
use serde::{Deserialize, Serialize};
use std::collections::VecDeque;

pub const NO_LIMIT_KPH: f64 = 250.;
/// Fastest descent (m/s2) of the published curve target.
pub const FALL_RATE: f64 = 1.5;

#[derive(Clone, Debug, Default, Deserialize, Serialize)]
pub struct ModelPath {
    pub x: Vec<f64>,
    pub y: Vec<f64>,
    pub z: Vec<f64>,
    pub velocity: Vec<f64>,
    pub yaw_rate: Vec<f64>,
}

#[derive(Clone, Copy, Debug, Deserialize, Serialize)]
pub struct CurveInput {
    pub v_ego: f64,
    pub sensitivity: f64,
    pub lower_limit_kph: f64,
    pub speed_ratio: f64,
    pub a_ego: f64,
}

#[derive(Clone, Copy, Debug, Deserialize, Serialize)]
pub struct CurveSpeed {
    pub approach_kph: f64,
    pub curve_kph: f64,
    pub distance: f64,
    pub direction: f64,
}

impl Default for CurveSpeed {
    fn default() -> Self {
        Self {
            approach_kph: NO_LIMIT_KPH,
            curve_kph: NO_LIMIT_KPH,
            distance: 0.,
            direction: 1.,
        }
    }
}

pub fn curve_speed(model: &ModelPath, input: CurveInput) -> Option<CurveSpeed> {
    let CurveInput {
        v_ego,
        sensitivity,
        lower_limit_kph,
        speed_ratio,
        a_ego,
    } = input;
    if [v_ego, sensitivity, lower_limit_kph, speed_ratio, a_ego]
        .iter()
        .any(|v| !v.is_finite())
        || sensitivity <= 0.
        || v_ego < 0.
        || [
            &model.x,
            &model.y,
            &model.z,
            &model.velocity,
            &model.yaw_rate,
        ]
        .iter()
        .any(|v| v.len() != 33)
    {
        return None;
    }
    let ratio = if speed_ratio > 0.5 && speed_ratio <= 1.2 {
        speed_ratio
    } else {
        1.
    };
    let end = (0_u32..33)
        .take_while(|i| f64::from(*i).powi(2) / 1024. * 10. <= 6.)
        .count();
    if [&model.x, &model.y, &model.z]
        .iter()
        .any(|v| v[..end].iter().any(|x| !x.is_finite()))
    {
        return None;
    }
    let mut distance = vec![0.; end];
    for i in 1..end {
        let sum = (model.x[i] - model.x[i - 1]).powi(2)
            + (model.y[i] - model.y[i - 1]).powi(2)
            + (model.z[i] - model.z[i - 1]).powi(2);
        distance[i] = distance[i - 1] + sum.sqrt();
    }
    let valid: Vec<_> = (0..end)
        .map(|i| {
            model.velocity[i].is_finite()
                && model.yaw_rate[i].is_finite()
                && model.velocity[i] >= 3.
        })
        .collect();
    let curvature: Vec<_> = (0..end)
        .map(|i| {
            if valid[i] {
                model.yaw_rate[i] / model.velocity[i]
            } else {
                0.
            }
        })
        .collect();
    let max_distance = 180_f64.min(30_f64.max(v_ego * 6.));
    // carrot reserved 1.6-2.4 s of travel (v * (1 + (a + 1) / 1.6)) before
    // the 1 m/s2 approach: at 76 km/h it ignored the first ~36 m of a curve
    // it already saw and left the rest to a late, steep drop. One second
    // covers the planner's response.
    let _ = a_ego;
    let response_distance = v_ego;
    let floor = lower_limit_kph.max(5.) * ratio / 3.6;
    let lateral_budget = 1.9 / sensitivity.clamp(0.5, 3.);
    let mut best = CurveSpeed::default();
    let mut found = false;
    for i in 0..end {
        let start = i.saturating_sub(1).min(end - 3);
        if distance[i] > max_distance
            || valid[start..start + 3].iter().any(|v| !v)
            || distance[start + 2] - distance[start] < 0.05
        {
            continue;
        }
        found = true;
        let mut nodes = [curvature[start], curvature[start + 1], curvature[start + 2]];
        nodes.sort_by(f64::total_cmp);
        let curve = nodes[1];
        if curve.abs() < 1e-6 {
            continue;
        }
        let curve_ms = floor.max((lateral_budget / curve.abs()).sqrt());
        let approach = (curve_ms.powi(2) + 2. * (distance[i] - response_distance).max(0.)).sqrt();
        let approach_kph = NO_LIMIT_KPH.min(approach * 3.6 / ratio);
        if approach_kph < best.approach_kph {
            best = CurveSpeed {
                approach_kph,
                curve_kph: NO_LIMIT_KPH.min(curve_ms * 3.6 / ratio),
                distance: distance[i],
                direction: 1_f64.copysign(curve),
            };
        }
    }
    found.then_some(best)
}

#[derive(Clone, Debug, Serialize)]
pub struct VisionCurveSpeed {
    pub speed: f64,
    pub direction: f64,
    pub last_time: Option<f64>,
    pub release_since: Option<f64>,
    pub geometry: VecDeque<(f64, f64)>,
    pub last_model_time: Option<f64>,
    /// Published target after the fall-rate limit.
    pub output: f64,
    /// Current speed (km/h) the limited target descends from; None skips it.
    #[serde(skip)]
    pub ego_kph: Option<f64>,
    #[serde(skip)]
    last_model_nanos: Option<u64>,
}

impl Default for VisionCurveSpeed {
    fn default() -> Self {
        Self {
            speed: NO_LIMIT_KPH,
            direction: 1.,
            last_time: None,
            release_since: None,
            geometry: VecDeque::new(),
            last_model_time: None,
            output: NO_LIMIT_KPH,
            ego_kph: None,
            last_model_nanos: None,
        }
    }
}

impl VisionCurveSpeed {
    pub fn update(&mut self, result: Option<CurveSpeed>, now: f64, model_time: Option<f64>) -> f64 {
        let stamp = model_time.unwrap_or(now);
        let fresh = !self
            .last_model_time
            .is_some_and(|previous| stamp <= previous);
        if fresh && now.is_finite() && result.is_some_and(|r| r.approach_kph.is_finite()) {
            self.last_model_time = Some(stamp);
        }
        self.update_frame(result, now, fresh)
    }
    pub fn update_nanos(
        &mut self,
        result: Option<CurveSpeed>,
        now: f64,
        model_time: Option<u64>,
    ) -> f64 {
        let Some(stamp) = model_time else {
            return self.update_frame(result, now, true);
        };
        let fresh = self
            .last_model_nanos
            .is_none_or(|previous| stamp > previous);
        if fresh && now.is_finite() && result.is_some_and(|r| r.approach_kph.is_finite()) {
            self.last_model_nanos = Some(stamp);
        }
        self.update_frame(result, now, fresh)
    }
    /// The model sees a far curve as gentler than it is and sharpens it as the
    /// car closes in, so the raw target fell in steps of 7-10 m/s2 just before
    /// the curve (route 00000cb1), which the planner followed as a hard, late
    /// brake. The published target descends from the current speed at most
    /// FALL_RATE; it still rises at once.
    fn update_frame(&mut self, result: Option<CurveSpeed>, now: f64, fresh: bool) -> f64 {
        let previous = self.last_time;
        self.target_frame(result, now, fresh);
        let target = self.speed;
        let dt = previous.map_or(0., |p| (now - p).clamp(0., 0.2));
        self.output = match self.ego_kph.filter(|v| v.is_finite()) {
            Some(ego) if now.is_finite() => {
                let from = self.output.min(ego + 3.);
                if target >= from {
                    target
                } else {
                    target.max(from - FALL_RATE * 3.6 * dt)
                }
            }
            _ => target,
        };
        self.output * self.direction
    }

    fn target_frame(&mut self, result: Option<CurveSpeed>, now: f64, fresh: bool) -> f64 {
        if !now.is_finite() {
            return self.speed * self.direction;
        }
        let dt = self
            .last_time
            .map_or(0., |previous| (now - previous).clamp(0., 0.2));
        self.last_time = Some(now);
        let result = result.filter(|r| r.approach_kph.is_finite());
        if let Some(result) = result {
            if !fresh {
                if result.approach_kph < self.speed {
                    self.speed = result.approach_kph;
                    self.direction = result.direction;
                    self.geometry.clear();
                }
                return self.speed * self.direction;
            }
            if self
                .geometry
                .back()
                .is_some_and(|(last, _)| now < *last || now - last > 0.20)
            {
                self.geometry.clear();
            }
            self.geometry.push_back((now, result.approach_kph));
            while self
                .geometry
                .front()
                .is_some_and(|(first, _)| now - first > 0.25 + 1e-9)
            {
                self.geometry.pop_front();
            }
            let confirmed = self.geometry.len() >= 3
                && self
                    .geometry
                    .front()
                    .is_some_and(|(first, _)| now - first >= 0.20 - 1e-9);
            if result.approach_kph <= self.speed {
                self.speed = result.approach_kph;
                self.direction = result.direction;
            } else if confirmed {
                self.speed = self
                    .geometry
                    .iter()
                    .map(|(_, v)| *v)
                    .fold(f64::INFINITY, f64::min);
            }
            self.release_since = None;
        } else {
            self.geometry.clear();
            let since = *self.release_since.get_or_insert(now);
            if now - since >= 0.35 {
                self.speed = NO_LIMIT_KPH.min(self.speed + 7.2 * dt);
            }
        }
        self.speed * self.direction
    }
}

#[cfg(test)]
mod tests {
    use super::{CurveSpeed, VisionCurveSpeed, FALL_RATE, NO_LIMIT_KPH};

    fn curve(kph: f64) -> Option<CurveSpeed> {
        Some(CurveSpeed {
            approach_kph: kph,
            curve_kph: kph,
            distance: 50.,
            direction: 1.,
        })
    }

    #[test]
    fn the_published_target_descends_from_the_current_speed_at_the_fall_rate() {
        let mut vision = VisionCurveSpeed::default();
        vision.ego_kph = Some(76.);
        // A step from no limit to 60 km/h starts at the current speed (+3).
        let first = vision.update_nanos(curve(60.), 0.0, Some(1));
        assert!((first - 79.).abs() < 1e-9, "{first}");
        let second = vision.update_nanos(curve(60.), 0.05, Some(2));
        assert!((second - (79. - FALL_RATE * 3.6 * 0.05)).abs() < 1e-9, "{second}");
        // It never falls below the raw target and rises again at once.
        for i in 3..200 {
            vision.update_nanos(curve(60.), f64::from(i) * 0.05, Some(i as u64));
        }
        assert!((vision.output - 60.).abs() < 1e-9);
        let up = vision.update_nanos(None, 10.0, Some(300));
        assert!(up >= 60.);
        // Without a current speed the raw target is published unchanged.
        let mut raw = VisionCurveSpeed::default();
        assert_eq!(raw.update_nanos(curve(60.), 0.0, Some(1)), 60.);
        assert_eq!(VisionCurveSpeed::default().output, NO_LIMIT_KPH);
    }
}
