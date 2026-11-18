//! Geometry shared by the lamp-only observers: carrot's
//! `desire_lib/boundary_lane.py` and the helpers of `indicator_observer.py`.
use crate::types::{Car, Model};

/// Minimum speed for the Ioniq 5 PE driver-initiated, lamp-only indications.
pub const MANUAL_INDICATOR_MIN_SPEED: f64 = 5. / 3.6;
pub const LINE_CONFIDENCE: f64 = 0.10;

pub fn finite(values: &[f64]) -> bool {
    values.iter().all(|v| v.is_finite())
}

/// Directional boundary exclusion for manual indication only, never steering.
pub fn adjacent_lane(car: &Car, model: &Model, direction: u8, now: f64) -> bool {
    match direction {
        1 => {
            // Only the left side uses OEM marking classification. Unknown or
            // stale classification cannot establish this is not a centre line.
            let stamp = car.lane_line_info_mono_time as f64 / 1e9;
            stamp > 0.
                && (0. ..=0.25).contains(&(now - stamp))
                && matches!(car.left_lane_line, 10 | 11 | 14 | 15 | 17)
        }
        2 => {
            // Geometric road-edge clearance only: not a free-space check.
            [0, 4, 8].iter().all(|&i| {
                let (Some(&boundary), Some(&edge)) =
                    (model.lane_lines[2].get(i), model.road_edges[1].get(i))
                else {
                    return false;
                };
                finite(&[boundary, edge]) && edge - boundary >= 2.5
            })
        }
        _ => false,
    }
}

/// Linear interpolation on strictly increasing `xs`; `None` outside range.
pub fn interpolate(x: f64, xs: &[f64], ys: &[f64]) -> Option<f64> {
    if xs.len() != ys.len() || xs.len() < 2 || !finite(xs) || !finite(ys) || !x.is_finite() {
        return None;
    }
    if xs.windows(2).any(|w| w[1] <= w[0]) || x < xs[0] || x > xs[xs.len() - 1] {
        return None;
    }
    for i in 1..xs.len() {
        if x <= xs[i] {
            let f = (x - xs[i - 1]) / (xs[i] - xs[i - 1]);
            return Some(ys[i - 1] + f * (ys[i] - ys[i - 1]));
        }
    }
    None
}

fn line_curvature(model: &Model, index: usize) -> Option<f64> {
    let near = [5., 10., 15.]
        .iter()
        .map(|&x| interpolate(x, &model.lane_line_x[index], &model.lane_lines[index]))
        .collect::<Option<Vec<_>>>()?;
    let slope = (near[2] - near[0]) / 10.;
    Some((near[2] - 2. * near[1] + near[0]) / 25. / (1. + slope * slope).powf(1.5))
}

/// Compares curved boundary and centre path at identical forward positions.
pub fn path_direction(model: &Model, curvature: f64) -> u8 {
    path_direction_inner(model, curvature).unwrap_or(0)
}

fn path_direction_inner(model: &Model, curvature: f64) -> Option<u8> {
    if !curvature.is_finite() {
        return Some(0);
    }
    for (direction, index, sign) in [(1u8, 1usize, -1.), (2, 2, 1.)] {
        let probability = model.lane_line_probs[index];
        if !probability.is_finite() || probability < LINE_CONFIDENCE {
            continue;
        }
        let lane = line_curvature(model, index)?;
        if (curvature - lane) * sign <= 0.0001 {
            continue;
        }
        let mut margins = Vec::with_capacity(5);
        for t in [1., 1.5, 2., 2.5, 3.] {
            let x = interpolate(t, &model.position_t, &model.position_x)?;
            let y = interpolate(t, &model.position_t, &model.position_y)?;
            let line = interpolate(x, &model.lane_line_x[index], &model.lane_lines[index])?;
            margins.push(sign * (y - line));
        }
        // The centre path starts inside, then crosses and stays across.
        if margins[0] < 0.
            && margins[4] >= 0.15
            && margins[3] >= 0.05
            && margins[4] > margins[0] + 0.3
        {
            return Some(direction);
        }
    }
    Some(0)
}

/// Recognised curved lane geometry agreeing with the vehicle's current arc.
pub fn following_road_curve(model: &Model, speed: f64, yaw_rate: Option<f64>) -> bool {
    let Some(yaw_rate) = yaw_rate else {
        return false;
    };
    // carrot 9eb26059a: a NaN probability fails too.
    if speed < 1.
        || ![1, 2]
            .iter()
            .all(|&i| (0.5..=1.).contains(&model.lane_line_probs[i]))
    {
        return false;
    }
    let curvature = yaw_rate / speed;
    if !curvature.is_finite() || curvature.abs() < 0.002 {
        return false;
    }
    [1, 2].iter().all(|&i| {
        line_curvature(model, i).is_some_and(|c| {
            c * curvature > 0. && (c - curvature).abs() <= 0.002f64.max(curvature.abs() * 0.5)
        })
    })
}

/// `(time, left, right, left prob, right prob)` of the nearest boundaries.
pub type Sample = (f64, f64, f64, f64, f64);

/// Confirms the two nearest forecast rows against measured motion of both
/// lane boundaries.
/// carrot `boundary_approach` (569f3191e): both measured boundaries moved at
/// least `minimum` toward the indicated crossing over 0.30..0.55 s, the near
/// boundary is within 1.5 m and that line was seen with confidence.
pub fn boundary_approach(samples: &[Sample], direction: u8, minimum: f64) -> bool {
    let (Some(first), Some(last)) = (samples.first(), samples.last()) else {
        return false;
    };
    if samples.len() < 2 {
        return false;
    }
    let (old_t, old_l, old_r, _, _) = *first;
    let (t, left, right, _, _) = *last;
    let sign = if direction == 1 { -1. } else { 1. };
    let near = if direction == 1 { -left } else { right };
    (0.30..=0.55).contains(&(t - old_t))
        && 0. < near
        && near <= 1.5
        && (sign * (old_l - left)).min(sign * (old_r - right)) >= minimum
        && samples
            .iter()
            .any(|s| (if direction == 1 { s.3 } else { s.4 }) >= LINE_CONFIDENCE)
}

pub fn anticipated_lane(model: &Model, samples: &[Sample]) -> u8 {
    let forecast = &model.desire_prediction;
    if forecast.len() != 32 {
        return 0;
    }
    for (side, index) in [(1u8, 3usize), (2, 4)] {
        if boundary_approach(samples, side, 0.06) {
            let rows_ok = [0, 8].iter().all(|&j| {
                let row = &forecast[j..j + 8];
                let max = row.iter().copied().fold(f64::NEG_INFINITY, f64::max);
                row.iter().all(|x| x.is_finite() && (0. ..=1.).contains(x))
                    && row[index] >= 0.65
                    && row[index] == max
            });
            if rows_ok {
                return side;
            }
        }
    }
    0
}
