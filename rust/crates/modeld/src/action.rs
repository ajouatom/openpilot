//! Action calculation matching modeld.py and controls/lib/drive_helpers.py.

use serde::{Deserialize, Serialize};

/// Smoothed model action, with SI units matching cereal ModelDataV2.Action.
#[derive(Default, Debug, Clone, Copy, Serialize, Deserialize)]
pub struct Action {
    pub desired_curvature: f64,
    pub desired_acceleration: f64,
    pub should_stop: bool,
    pub desired_velocity: f64,
}

/// Timing, speed and configured thresholds for the current model action.
#[derive(Debug, Clone, Copy, Serialize, Deserialize)]
pub struct ActionInputs {
    pub lat_action_t: f64,
    pub long_action_t: f64,
    pub v_ego: f64,
    pub lat_smooth_seconds: f64,
    pub v_ego_stopping: f64,
}

/// Selected plan and optional direct lateral-acceleration/acceleration action.
#[derive(Debug, Clone, Copy)]
pub struct PlanActionInput<'a> {
    pub plan: &'a [[f32; 15]; 33],
    pub direct_action: Option<[f32; 2]>,
}

/// Preserves the source's failed four-dimensional plan_stds lookup (y_std = 0).
#[must_use]
pub fn dynamic_lat_smooth_seconds(base: f64) -> f64 {
    if base <= 0.0 {
        0.0
    } else {
        base.clamp(0.0, 0.60)
    }
}

/// Calculates an action without changing the source's smoothing or stop policy.
/// Action times use the caller's positive actuator-delay intervals.
#[must_use]
pub fn from_plan(output: PlanActionInput<'_>, previous: Action, inputs: ActionInputs) -> Action {
    let plan = output.plan;
    let target_speed = interpolate(plan, 3, inputs.long_action_t);
    let future_speed = interpolate(plan, 3, inputs.long_action_t + 1.0);
    let should_stop = target_speed < inputs.v_ego_stopping && future_speed < inputs.v_ego_stopping;
    // np.max retains f32 and propagates NaN; f32::max alone would discard NaN.
    let max_speed = plan.iter().fold(plan[0][3], |maximum, row| {
        if maximum.is_nan() || row[3].is_nan() {
            f32::NAN
        } else {
            maximum.max(row[3])
        }
    });
    let (acceleration, curvature) = match output.direct_action {
        Some([lateral_acceleration, acceleration]) => {
            // NumPy 2 divides the f32 action by a weak Python scalar in f32.
            // This narrowing preserves its rounded denominator before division.
            // carrot a47fd16e5 (commaai 0c40995): low-speed floor 2.5 m/s.
            let speed_squared = inputs.v_ego.max(2.5).powi(2) as f32;
            (
                f64::from(acceleration),
                f64::from(lateral_acceleration / speed_squared),
            )
        }
        None => {
            let acceleration = 2.0 * (target_speed - f64::from(plan[0][3])) / inputs.long_action_t
                - f64::from(plan[0][6]);
            let yaw = interpolate(plan, 11, inputs.lat_action_t);
            // np.clip preserves NaN, unlike f64::max.
            let speed = if inputs.v_ego < 1.0 {
                1.0
            } else {
                inputs.v_ego
            };
            let curvature =
                2.0 * (yaw / (speed * inputs.lat_action_t)) - f64::from(plan[0][14]) / speed;
            (acceleration, curvature)
        }
    };
    Action {
        desired_curvature: if inputs.v_ego > 0.3 {
            smooth(
                curvature,
                previous.desired_curvature,
                inputs.lat_smooth_seconds,
            )
        } else {
            previous.desired_curvature
        },
        desired_acceleration: smooth(acceleration, previous.desired_acceleration, 0.3),
        should_stop,
        desired_velocity: smooth(f64::from(max_speed), previous.desired_velocity, 0.3),
    }
}

fn smooth(value: f64, previous: f64, tau: f64) -> f64 {
    let alpha = if tau > 0.0 {
        1.0 - (-0.05 / tau).exp()
    } else {
        1.0
    };
    alpha * value + (1.0 - alpha) * previous
}

fn interpolate(plan: &[[f32; 15]; 33], column: usize, time: f64) -> f64 {
    if time <= 0.0 {
        return f64::from(plan[0][column]);
    }
    let mut previous_time = 0.0;
    for (index, pair) in (1_u32..=32).zip(plan.windows(2)) {
        let next_time = 10.0 * (f64::from(index) / 32.0).powi(2);
        if time <= next_time {
            let previous_value = f64::from(pair[0][column]);
            let next_value = f64::from(pair[1][column]);
            if time == next_time {
                return next_value;
            }
            let slope = (next_value - previous_value) / (next_time - previous_time);
            let mut value = previous_value + slope * (time - previous_time);
            // np.interp retries from the other endpoint for non-finite samples.
            if value.is_nan() {
                value = next_value + slope * (time - next_time);
                if value.is_nan() && previous_value == next_value {
                    value = previous_value;
                }
            }
            return value;
        }
        previous_time = next_time;
    }
    if time.is_nan() {
        f64::NAN
    } else {
        f64::from(plan[32][column])
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    fn direct(v_ego: f64) -> f64 {
        let plan = [[0.0_f32; 15]; 33];
        from_plan(
            PlanActionInput {
                plan: &plan,
                direct_action: Some([1.0, 0.0]),
            },
            Action::default(),
            ActionInputs {
                lat_action_t: 0.2,
                long_action_t: 0.3,
                v_ego,
                lat_smooth_seconds: 0.0,
                v_ego_stopping: 0.5,
            },
        )
        .desired_curvature
    }

    #[test]
    fn direct_action_curvature_uses_the_2_5_m_s_floor() {
        assert!((direct(1.0) - 1.0 / 6.25).abs() < 1e-7);
        assert!((direct(2.0) - 1.0 / 6.25).abs() < 1e-7);
        assert!((direct(5.0) - 1.0 / 25.0).abs() < 1e-7);
    }
}
