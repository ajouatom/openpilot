//! Laneless lane centring (`LanelessCentering`, 0 = off, 100 = the gains below).
//!
//! In laneless mode controlsd follows the model's own curvature. On drive
//! 00000ccb N16 held its path about 0.19 m left of the lane centre whenever no
//! lead car was ahead (p90 0.34 m) and was centred behind a lead; controls
//! tracked the model's curvature to within 1e-5, so steer ratio and actuator
//! delay were not the cause and lane mode (`UseLaneLineSpeed`) is too rigid in
//! the city. This adds a small, bounded curvature on top of the model's:
//!
//! * P: the pure-pursuit curvature that moves the model's path point at
//!   lookahead `L` (2 s of travel, 15..50 m) onto the lane centre at `L`,
//!   `2 (centre - path) / L²`. Path and centre bend together in a curve, so
//!   only their offset counts, not the road's curvature.
//! * I: the car's own offset from the lane centre (5 m ahead), integrated, to
//!   remove what the model pulls back.
//!
//! Both work in lateral acceleration (curvature × v²) so a setting feels the
//! same at any speed; the sum is capped at 0.15 m/s² at 100 % (0.30 at the
//! 200 % maximum) on a straight road. How much offset that removes depends on
//! how hard the model pulls back to its own line, which only a drive shows:
//! start at 100 and raise it if the car stays off centre. It ramps in over 1 s
//! and out over 0.5 s, and only acts above 30 km/h with both ego lane lines
//! seen (probability ≥ 0.5, 2.6..4.3 m apart), no lane change, turn or ATC, no
//! driver steering and a road gentler than an 80 m radius.
//!
//! Curves. The driving models hold the inside of a firm curve: on drive
//! 00000cdf (IONIQ 5, 11/23) N16 ran 0.32 m inside the lane centre above
//! 1 m/s² of lateral acceleration, 0.37 m on segment 11 with its path still
//! 0.28 m inside 30 m ahead, and exactly so on the same frames run on a CPU,
//! so it is the model's own line, not the accelerator's; Cheddar held 0.33 m
//! on 00000ce3. Until this change centring stopped at a 200 m radius, where
//! those curves begin. Now it works up to an 80 m radius, and as the road's
//! lateral acceleration grows (the model's curvature × v², up to 2 m/s²) so
//! do its cap, by 0.10 m/s² per m/s² (0.30 at 1.5 m/s², at 100 %), and the
//! path offset it accepts as real, from 0.6 m to 0.9 m by 1.5 m/s²: in
//! segment 11 a tenth of the curve frames had the path 0.6 m or more inside.

const DT: f64 = 0.01;
const MIN_SPEED: f64 = 30. / 3.6;
const LOOKAHEAD_S: f64 = 2.;
const LOOKAHEAD_MIN: f64 = 15.;
const LOOKAHEAD_MAX: f64 = 50.;
const CAR_POINT: f64 = 5.;
/// m/s² of integral per metre of car offset per second, at 100 %.
const INTEGRAL_GAIN: f64 = 0.02;
const INTEGRAL_MAX: f64 = 0.08;
const INTEGRAL_LEAK_S: f64 = 2.;
/// At 100 %; scales with the setting up to `STRENGTH_MAX`.
const ACCEL_MAX: f64 = 0.15;
const STRENGTH_MAX: f64 = 2.;
const RAMP_IN_S: f64 = 1.;
const RAMP_OUT_S: f64 = 0.5;
const LINE_PROB_MIN: f64 = 0.5;
const WIDTH_MIN: f64 = 2.6;
const WIDTH_MAX: f64 = 4.3;
const CAR_OFFSET_MAX: f64 = 0.8;
const PATH_OFFSET_MAX: f64 = 0.6;
/// The path offset accepted at a lateral acceleration of `CURVE_FULL` or more.
const PATH_OFFSET_MAX_CURVE: f64 = 0.9;
/// m/s² of cap added per m/s² of the road's lateral acceleration, at 100 %.
const CURVE_CAP_GAIN: f64 = 0.10;
/// The road's lateral acceleration past which the curve allowances stop growing.
const CURVE_ACCEL_MAX: f64 = 2.;
/// Where the path-offset allowance reaches `PATH_OFFSET_MAX_CURVE`.
const CURVE_FULL: f64 = 1.5;
const CURVATURE_MAX: f64 = 1. / 80.;
/// The correction's time constant: it acts over seconds, the model's frames
/// jitter within 0.1-0.2 s.
const ACCEL_TAU_S: f64 = 0.5;
/// m/s² per second the correction may change at 100 %; scales with the setting.
const ACCEL_RATE_MAX: f64 = 0.4;

/// The road's lateral acceleration as the model drives it, capped for the allowances.
fn curve_accel(sample: &Sample<'_>) -> f64 {
    (sample.model_curvature * sample.speed.powi(2))
        .abs()
        .min(CURVE_ACCEL_MAX)
}

/// What one control step needs from modelV2, carState and the lateral state.
pub struct Sample<'a> {
    /// `LanelessCentering` / 100; 0 switches it off.
    pub strength: f64,
    /// Laneless lateral control is active and nothing else owns the path
    /// (no lane change, turn desire or ATC, no driver steering).
    pub allowed: bool,
    pub speed: f64,
    pub model_curvature: f64,
    /// Ego lane lines (laneLines[1] left, [2] right): shared x, each y.
    pub lane_x: &'a [f64],
    pub left_y: &'a [f64],
    pub right_y: &'a [f64],
    pub left_prob: f64,
    pub right_prob: f64,
    /// modelV2.position.
    pub path_x: &'a [f64],
    pub path_y: &'a [f64],
}

#[derive(Default)]
pub struct Centering {
    blend: f64,
    integral: f64,
    /// Last lateral acceleration added (m/s²), for tests and diagnostics.
    pub accel: f64,
}

impl Centering {
    /// The curvature to add to the model's this step.
    pub fn update(&mut self, sample: &Sample<'_>) -> f64 {
        let offsets = (sample.strength > 0.).then(|| offsets(sample)).flatten();
        match offsets {
            Some((car, path)) => {
                self.blend = (self.blend + DT / RAMP_IN_S).min(1.);
                // Above 100 % the limits grow too (to 2×): a model that pulls
                // back to its own line hard needs more than 0.15 m/s².
                let scale = sample.strength.min(STRENGTH_MAX);
                let limit = INTEGRAL_MAX * scale;
                self.integral =
                    (self.integral + INTEGRAL_GAIN * scale * car * DT).clamp(-limit, limit);
                let lookahead = lookahead(sample.speed);
                let pursuit = 2. * path * sample.speed.powi(2) / lookahead.powi(2);
                let cap = (ACCEL_MAX + CURVE_CAP_GAIN * curve_accel(sample)) * scale;
                let target = (scale * pursuit + self.integral).clamp(-cap, cap);
                // The lane centre and the path 25-30 m ahead move a little
                // from one model frame to the next; taken as they come the
                // correction shook at 2-3 Hz (PHEV 0000000f seg 14 at 150 %:
                // 0.17, 0.26, 0.12, 0.20 m/s² 0.2 s apart), which drivers
                // felt as a tapping wheel. Smoothed over 0.5 s and limited in
                // how fast it may change, it follows the road, not the noise.
                let filtered = self.accel + (target - self.accel) * DT / ACCEL_TAU_S;
                let step = ACCEL_RATE_MAX * scale * DT;
                self.accel += (filtered - self.accel).clamp(-step, step);
            }
            None => {
                self.blend = (self.blend - DT / RAMP_OUT_S).max(0.);
                self.integral *= (-DT / INTEGRAL_LEAK_S).exp();
                if sample.strength <= 0. {
                    self.integral = 0.;
                }
            }
        }
        if self.blend == 0. {
            self.accel = 0.;
            return 0.;
        }
        self.blend * self.accel / sample.speed.max(MIN_SPEED).powi(2)
    }
}

fn lookahead(speed: f64) -> f64 {
    (speed * LOOKAHEAD_S).clamp(LOOKAHEAD_MIN, LOOKAHEAD_MAX)
}

/// The car's offset from the lane centre and the path's offset from it at the
/// lookahead (both + when the centre is to the left), or None when the
/// geometry or the conditions do not allow centring.
fn offsets(sample: &Sample<'_>) -> Option<(f64, f64)> {
    if !sample.allowed
        || sample.speed < MIN_SPEED
        || sample.model_curvature.abs() > CURVATURE_MAX
        || sample.left_prob < LINE_PROB_MIN
        || sample.right_prob < LINE_PROB_MIN
    {
        return None;
    }
    let centre = |x: f64| -> Option<(f64, f64)> {
        let left = at(sample.lane_x, sample.left_y, x)?;
        let right = at(sample.lane_x, sample.right_y, x)?;
        Some(((left + right) / 2., right - left))
    };
    // The centre 5 m ahead, less the bend the car's own curvature puts there:
    // on a curve the centre at 5 m is already κ·5²/2 to the inside (8 cm at a
    // 150 m radius), which is not an offset of the car.
    let (car, width) = centre(CAR_POINT)?;
    let car = car - sample.model_curvature * CAR_POINT * CAR_POINT / 2.;
    let lookahead = lookahead(sample.speed);
    let (far, _) = centre(lookahead)?;
    let path = far - at(sample.path_x, sample.path_y, lookahead)?;
    let path_max = PATH_OFFSET_MAX
        + (PATH_OFFSET_MAX_CURVE - PATH_OFFSET_MAX) * (curve_accel(sample) / CURVE_FULL).min(1.);
    ((WIDTH_MIN..=WIDTH_MAX).contains(&width)
        && car.abs() <= CAR_OFFSET_MAX
        && path.abs() <= path_max)
        .then_some((car, path))
}

/// Linear interpolation of y at x along increasing xs; None outside the data.
fn at(xs: &[f64], ys: &[f64], x: f64) -> Option<f64> {
    if xs.len() < 2 || xs.len() != ys.len() || !x.is_finite() {
        return None;
    }
    if x < xs[0] || x > xs[xs.len() - 1] {
        return None;
    }
    let index = xs.windows(2).position(|w| x <= w[1])?;
    let (x0, x1) = (xs[index], xs[index + 1]);
    let t = if x1 > x0 { (x - x0) / (x1 - x0) } else { 0. };
    let value = ys[index] + t * (ys[index + 1] - ys[index]);
    value.is_finite().then_some(value)
}

#[cfg(test)]
mod tests {
    use super::{Centering, Sample, ACCEL_MAX, CURVE_CAP_GAIN};

    const XS: [f64; 33] = {
        let mut xs = [0.; 33];
        let mut i = 0;
        while i < 33 {
            // modelV2's x grid is quadratic in index up to ~192 m.
            xs[i] = 192. * (i as f64 / 32.) * (i as f64 / 32.);
            i += 1;
        }
        xs
    };

    /// A straight lane 3.5 m wide whose centre is `centre` m left of the car,
    /// with the model's path `path` m left of the car everywhere.
    fn straight(centre: f64, path: f64) -> ([f64; 33], [f64; 33], [f64; 33]) {
        (
            XS.map(|_| centre - 1.75),
            XS.map(|_| centre + 1.75),
            XS.map(|_| path),
        )
    }

    fn sample<'a>(speed: f64, lines: &'a ([f64; 33], [f64; 33], [f64; 33])) -> Sample<'a> {
        Sample {
            strength: 1.,
            allowed: true,
            speed,
            model_curvature: 0.,
            lane_x: &XS,
            left_y: &lines.0,
            right_y: &lines.1,
            left_prob: 0.9,
            right_prob: 0.9,
            path_x: &XS,
            path_y: &lines.2,
        }
    }

    fn settle(centering: &mut Centering, sample: &Sample<'_>, steps: usize) -> f64 {
        (0..steps)
            .map(|_| centering.update(sample))
            .last()
            .unwrap_or(0.)
    }

    #[test]
    fn steers_toward_the_centre_when_the_model_holds_left_of_it() {
        // Drive 00000ccb without a lead: centre 0.19 m right of the car and of
        // the path. Curvature + is left, so the correction is negative.
        let lines = straight(-0.19, 0.);
        let mut c = Centering::default();
        let curvature = settle(&mut c, &sample(70. / 3.6, &lines), 150);
        assert!(curvature < 0., "{curvature}");
        // 2 s lookahead at 70 km/h: 2 × 0.19 / 2² ≈ 0.095 m/s², plus a little integral.
        assert!((0.09..0.12).contains(&-c.accel), "{}", c.accel);
    }

    /// A curve of `curvature` (+ the same side as + y): lane centre and car on
    /// it, the model's path `inside` m to the curve's inside all the way.
    fn curve(curvature: f64, inside: f64) -> ([f64; 33], [f64; 33], [f64; 33]) {
        let centre = |x: f64| curvature * x * x / 2.;
        (
            XS.map(|x| centre(x) - 1.75),
            XS.map(|x| centre(x) + 1.75),
            XS.map(|x| centre(x) + inside * curvature.signum() * (x / 10.).min(1.)),
        )
    }

    #[test]
    fn steers_out_of_a_curve_the_model_cuts_and_past_the_straight_cap() {
        // 50 km/h on a 150 m radius (1.29 m/s²), the path 0.5 m inside, as
        // N16's sharpest frames on 00000cdf segment 11.
        let speed = 50. / 3.6;
        for curvature in [1. / 150., -1. / 150.] {
            let lines = curve(curvature, 0.5);
            let s = Sample {
                model_curvature: curvature,
                ..sample(speed, &lines)
            };
            let mut c = Centering::default();
            let correction = settle(&mut c, &s, 300);
            // toward the outside: against the curve's own sign
            assert!(correction * curvature < 0., "{curvature}: {correction}");
            let road = curvature.abs() * speed * speed;
            assert!(
                c.accel.abs() > ACCEL_MAX,
                "no more than the straight cap: {}",
                c.accel
            );
            assert!(
                c.accel.abs() <= ACCEL_MAX + CURVE_CAP_GAIN * road + 1e-12,
                "{}",
                c.accel
            );
        }
    }

    #[test]
    fn a_path_that_jitters_from_frame_to_frame_moves_the_correction_slowly() {
        // A curve the model cuts 0.4 m inside, its path 30 m ahead jumping
        // ±0.12 m from one 20 Hz model frame to the next as on PHEV 0000000f
        // seg 14; controls run at 100 Hz, so each frame holds 5 steps.
        let speed = 46. / 3.6;
        let curvature = 1. / 120.;
        let strength = 1.5;
        let mut c = Centering::default();
        let mut seed: u32 = 12345;
        let mut accels = vec![];
        for frame in 0..400 {
            seed = seed.wrapping_mul(1_103_515_245).wrapping_add(12345);
            let jitter = (f64::from(seed >> 16) / 65535. - 0.5) * 0.24;
            let lines = curve(curvature, 0.4 + jitter);
            let s = Sample {
                strength,
                model_curvature: curvature,
                ..sample(speed, &lines)
            };
            for _ in 0..5 {
                c.update(&s);
                if frame >= 200 {
                    accels.push(c.accel);
                }
            }
        }
        let steps = accels.windows(2).map(|w| (w[1] - w[0]).abs() / super::DT);
        let fastest = steps.fold(0., f64::max);
        assert!(
            fastest <= super::ACCEL_RATE_MAX * strength + 1e-9,
            "{fastest} m/s² per s"
        );
        // and it still pulls outward as hard as the steady path asks, on average
        let mean = accels.iter().sum::<f64>() / accels.len() as f64;
        let mut steady = Centering::default();
        let lines = curve(curvature, 0.4);
        settle(
            &mut steady,
            &Sample {
                strength,
                model_curvature: curvature,
                ..sample(speed, &lines)
            },
            2000,
        );
        assert!(
            (mean - steady.accel).abs() < 0.03,
            "{mean} vs {}",
            steady.accel
        );
    }

    #[test]
    fn a_curve_driven_on_its_centre_gets_next_to_nothing() {
        // Only the lane grid's linear interpolation of the bend at 5 m is
        // left (a few mm), well under a twentieth of the straight cap. Without
        // taking the car's own bend off the 5 m point the integral would lean
        // the car 8 cm into this curve.
        let lines = curve(1. / 150., 0.);
        let s = Sample {
            model_curvature: 1. / 150.,
            ..sample(50. / 3.6, &lines)
        };
        let mut c = Centering::default();
        settle(&mut c, &s, 300);
        assert!(c.accel.abs() < ACCEL_MAX / 20., "{}", c.accel);
    }

    #[test]
    fn a_path_far_off_the_lane_on_a_straight_is_still_taken_for_bad_lines() {
        // 0.8 m off is accepted in a firm curve, not on a straight road
        let lines = straight(-0.8, 0.);
        let mut c = Centering::default();
        assert_eq!(settle(&mut c, &sample(70. / 3.6, &lines), 300), 0.);
    }

    #[test]
    fn a_centred_path_and_car_get_nothing() {
        let lines = straight(0., 0.);
        let mut c = Centering::default();
        assert_eq!(settle(&mut c, &sample(70. / 3.6, &lines), 300), 0.);
    }

    #[test]
    fn the_correction_never_exceeds_its_lateral_acceleration_cap() {
        let lines = straight(-0.55, 0.);
        let mut c = Centering::default();
        let speed = 50. / 3.6;
        let curvature = settle(&mut c, &sample(speed, &lines), 3000);
        assert!(c.accel.abs() <= ACCEL_MAX + 1e-12, "{}", c.accel);
        assert!((curvature * speed * speed).abs() <= ACCEL_MAX + 1e-12);
    }

    #[test]
    fn ramps_in_over_a_second_and_out_when_conditions_end() {
        let lines = straight(-0.19, 0.);
        let mut c = Centering::default();
        let first = c.update(&sample(70. / 3.6, &lines));
        let full = settle(&mut c, &sample(70. / 3.6, &lines), 100);
        assert!(first.abs() < full.abs() * 0.05, "{first} vs {full}");
        let mut off = sample(70. / 3.6, &lines);
        off.allowed = false;
        let half = settle(&mut c, &off, 25);
        assert!(half.abs() < full.abs() * 0.6 && half.abs() > 0.);
        assert_eq!(settle(&mut c, &off, 30), 0.);
    }

    #[test]
    fn does_nothing_when_switched_off_slow_curved_or_unsure_of_the_lane() {
        let lines = straight(-0.19, 0.);
        let base = sample(70. / 3.6, &lines);
        let wide = straight(-0.19, 0.);
        let wide = ([0.; 33].map(|_| -3.), wide.1, wide.2);
        for (why, s) in [
            (
                "off",
                Sample {
                    strength: 0.,
                    ..sample(70. / 3.6, &lines)
                },
            ),
            ("slow", sample(25. / 3.6, &lines)),
            (
                "a curve tighter than 80 m",
                Sample {
                    model_curvature: 0.02,
                    ..sample(50. / 3.6, &lines)
                },
            ),
            (
                "faint line",
                Sample {
                    left_prob: 0.3,
                    ..sample(70. / 3.6, &lines)
                },
            ),
            ("lane too wide", sample(70. / 3.6, &wide)),
            (
                "not allowed",
                Sample {
                    allowed: false,
                    ..sample(70. / 3.6, &lines)
                },
            ),
        ] {
            let mut c = Centering::default();
            assert_eq!(settle(&mut c, &s, 300), 0., "{why}");
        }
        let _ = base;
    }

    /// A model that steers by pure pursuit toward its own line 0.19 m left
    /// of the lane centre, looking `model_seconds` ahead, a first-order
    /// steering lag of 0.25 s, 70 km/h. Returns the car's offset from the
    /// centre (+ left) after `seconds` and the largest overshoot to the right.
    fn simulate(strength: f64, model_seconds: f64, seconds: f64) -> (f64, f64) {
        let v = 70. / 3.6;
        let preferred = 0.19;
        let model_lookahead = v * model_seconds;
        let (mut y, mut psi, mut curvature) = (preferred, 0., 0.);
        let mut c = Centering::default();
        let mut overshoot: f64 = 0.;
        for _ in 0..(seconds / super::DT) as usize {
            // The lane and the model's path as the car sees them (car frame).
            let left = XS.map(|x| -y - psi * x - 1.75);
            let right = XS.map(|x| -y - psi * x + 1.75);
            let path = XS.map(|x| {
                let toward = (preferred - y) * (x / model_lookahead).min(1.);
                toward - psi * x * (1. - (x / model_lookahead).min(1.))
            });
            let model = 2. * ((preferred - y) - psi * model_lookahead) / model_lookahead.powi(2);
            let sample = Sample {
                strength,
                allowed: true,
                speed: v,
                model_curvature: model,
                lane_x: &XS,
                left_y: &left,
                right_y: &right,
                left_prob: 0.9,
                right_prob: 0.9,
                path_x: &XS,
                path_y: &path,
            };
            let target = model + c.update(&sample);
            curvature += (target - curvature) * super::DT / 0.25;
            psi += v * curvature * super::DT;
            y += v * psi * super::DT;
            overshoot = overshoot.max(-y);
        }
        (y, overshoot)
    }

    /// A model that cuts a curve: on a road of `curvature` (+ left) it steers
    /// by pure pursuit, `model_seconds` ahead, toward its own line `inside` m
    /// to the curve's inside, with a first-order steering lag of `lag` s (what
    /// lagd measures, 0.30-0.34 s on the cars logged). Returns the car's
    /// offset from the lane centre toward the inside after `seconds`, and the
    /// furthest it went toward the outside.
    fn simulate_curve(
        strength: f64,
        curvature: f64,
        speed: f64,
        inside: f64,
        model_seconds: f64,
        lag: f64,
        seconds: f64,
    ) -> (f64, f64) {
        let preferred = inside * curvature.signum();
        let model_lookahead = speed * model_seconds;
        // y: the car's offset from the centre (+ left), psi: its heading
        // against the road's tangent, starting on the model's own line
        let (mut y, mut psi, mut steer) = (preferred, 0., curvature);
        let mut c = Centering::default();
        let mut outside: f64 = 0.;
        for _ in 0..(seconds / super::DT) as usize {
            // the lane centre and the model's path in the car's frame
            let centre = |x: f64| -y - psi * x + curvature * x * x / 2.;
            let left = XS.map(|x| centre(x) - 1.75);
            let right = XS.map(|x| centre(x) + 1.75);
            let path = XS.map(|x| centre(x) + y + (preferred - y) * (x / model_lookahead).min(1.));
            let model = curvature
                + 2. * ((preferred - y) - psi * model_lookahead) / model_lookahead.powi(2);
            let sample = Sample {
                strength,
                allowed: true,
                speed,
                model_curvature: model,
                lane_x: &XS,
                left_y: &left,
                right_y: &right,
                left_prob: 0.9,
                right_prob: 0.9,
                path_x: &XS,
                path_y: &path,
            };
            let target = model + c.update(&sample);
            steer += (target - steer) * super::DT / lag;
            psi += speed * (steer - curvature) * super::DT;
            y += speed * psi * super::DT;
            outside = outside.max(-y * curvature.signum());
        }
        (y * curvature.signum(), outside)
    }

    #[test]
    fn a_model_that_cuts_a_firm_curve_is_drawn_toward_the_centre() {
        // 00000cdf segment 11: 50 km/h, a 150 m radius (1.29 m/s²), N16
        // 0.35 m inside. 30 s of the same curve, both directions.
        let speed = 50. / 3.6;
        for curvature in [1. / 150., -1. / 150.] {
            let (off, _) = simulate_curve(0., curvature, speed, 0.35, 1.5, 0.3, 30.);
            assert!((off - 0.35).abs() < 0.02, "uncorrected {off}");
            let (on, outside) = simulate_curve(1., curvature, speed, 0.35, 1.5, 0.3, 30.);
            let (raised, outside_raised) =
                simulate_curve(2., curvature, speed, 0.35, 1.5, 0.3, 30.);
            assert!(on < 0.6 * off, "100 %: still {on} m inside");
            assert!(raised < on, "200 %: {raised} m vs {on} m");
            // 100 % stays inside the centre; 200 % may lean a few cm past it
            assert!(outside < 0.05, "100 % swung out {outside} m");
            assert!(outside_raised < 0.12, "200 % swung out {outside_raised} m");
        }
    }

    #[test]
    fn a_slower_or_quicker_steering_lag_changes_little() {
        // lagd's estimate moves with the car and the road; the correction
        // looks 2 s ahead and integrates over seconds, so 0.2-0.6 s of lag
        // must not undo it or swing the car out of the curve
        let speed = 50. / 3.6;
        for lag in [0.2, 0.3, 0.45, 0.6] {
            let (on, outside) = simulate_curve(1., 1. / 150., speed, 0.35, 1.5, lag, 30.);
            assert!(
                on < 0.15 && outside < 0.05,
                "lag {lag}: {on} m inside, {outside} m out"
            );
        }
    }

    /// The same curve at each setting, for the drivers' guide:
    /// `cargo test -p openpilot-controlsd curve_settings -- --nocapture --ignored`.
    #[test]
    #[ignore]
    fn curve_settings() {
        for (kmh, radius) in [(50., 150.), (60., 200.), (40., 100.)] {
            let speed = kmh / 3.6;
            for lag in [0.3, 0.45, 0.6] {
                for strength in [0., 1., 1.5, 2.] {
                    let (on, outside) =
                        simulate_curve(strength, 1. / radius, speed, 0.35, 1.5, lag, 30.);
                    eprintln!("{kmh} km/h, {radius} m radius, lag {lag} s, {:>3.0} %: {on:+.2} m inside after 30 s, at most {outside:.2} m outside", strength * 100.);
                }
            }
        }
    }

    #[test]
    fn a_left_holding_model_is_brought_to_the_centre_without_swinging_past() {
        let (uncorrected, _) = simulate(0., 1.5, 30.);
        assert!((uncorrected - 0.19).abs() < 0.01, "{uncorrected}");
        let (corrected, overshoot) = simulate(1., 1.5, 30.);
        assert!(corrected.abs() < 0.05, "still {corrected} m off after 30 s");
        assert!(overshoot < 0.05, "swung {overshoot} m past the centre");
    }

    #[test]
    fn a_stiffer_model_needs_and_gets_more_from_a_higher_setting() {
        let (default, _) = simulate(1., 1., 30.);
        let (raised, overshoot) = simulate(2., 1., 30.);
        assert!(raised.abs() < default.abs(), "{raised} vs {default}");
        assert!(raised < 0.12 && overshoot < 0.05, "{raised}, {overshoot}");
    }
}
