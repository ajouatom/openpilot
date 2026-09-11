"""Bounded, smoothly adaptive velocity/acceleration radar observer."""

import math


class RadarLeadFilter:
  """Use persistent prediction error to distinguish a motion change from jitter.

  Estimate motion, persistent residual and measurement jitter with scalar updates.
  Gains vary continuously; there are no acceleration thresholds, motion modes,
  matrix operations, or exponential calculations in the update.
  """

  def __init__(self, velocity: float, dt: float):
    if not math.isfinite(dt) or dt <= 0.0:
      raise ValueError("radar sample period must be finite and positive")
    self.dt = dt
    slow_v, slow_a = 0.10 / (0.10 + dt), 0.15 / (0.15 + dt)
    fast_v, fast_a = 0.05 / (0.05 + dt), 0.075 / (0.075 + dt)
    self.alpha_slow = 1.0 - slow_v * slow_a
    self.beta_slow = (1.0 - slow_v) * (1.0 - slow_a)
    self.alpha_range = 1.0 - fast_v * fast_a - self.alpha_slow
    self.beta_range = (1.0 - fast_v) * (1.0 - fast_a) - self.beta_slow
    self.residual_alpha = dt / (0.15 + dt)
    self.noise_alpha = dt / (0.50 + dt)
    self.accel_alpha_range = dt / (0.075 + dt) - self.residual_alpha
    # Speed uncertainty plus one sample of acceleration uncertainty, in m/s.
    self.residual_scale_squared = (0.10 + 0.8 * dt)**2
    # Preserve the historical post-filter innovation bound, including at other dt.
    self.max_accel_step = 3.0 * self.residual_alpha
    self.reset(velocity)

  def reset(self, velocity: float):
    self.velocity = velocity
    self.acceleration = 0.0
    self.mean_residual = 0.0
    self.previous_residual = 0.0
    self.residual_variance = 0.0
    self.response_weight = 0.0
    self.limited = False

  def update(self, velocity: float, *, stationary: bool = False) -> float:
    predicted_velocity = self.velocity + self.acceleration * self.dt
    residual = velocity - predicted_velocity
    # One bad sample must not fill the persistent-error state.
    residual_sample = 0.0 if stationary else max(-0.5, min(0.5, residual))
    evidence = max(0.0, self.mean_residual * residual_sample)
    noise_sample = 0.5 * (residual_sample - self.previous_residual)**2
    self.previous_residual = residual_sample
    self.residual_variance += self.noise_alpha * (noise_sample - self.residual_variance)
    self.mean_residual += self.residual_alpha * (residual_sample - self.mean_residual)
    self.response_weight = (evidence / (self.residual_scale_squared + 8.0 * self.residual_variance + evidence))**2
    alpha = self.alpha_slow + self.alpha_range * self.response_weight
    beta = self.beta_slow + self.beta_range * self.response_weight
    predicted_acceleration = self.acceleration + beta / self.dt * residual
    correction = (-self.residual_alpha * self.acceleration if stationary else beta / self.dt * residual)
    accel_alpha = self.residual_alpha + self.accel_alpha_range * self.response_weight
    correction = max((-10.0 - self.acceleration) * accel_alpha,
                     min((5.0 - self.acceleration) * accel_alpha, correction))
    self.limited = abs(correction) > self.max_accel_step
    correction = max(-self.max_accel_step, min(self.max_accel_step, correction))
    self.acceleration = max(-10.0, min(5.0, self.acceleration + correction))
    # Keep the velocity state consistent with a bounded acceleration correction.
    # At the slow gain, velocity = historical filtered_speed + .10*acceleration.
    # Omitting this correction changes the spike response even with adaptation off.
    speed_rc = 0.10 - 0.05 * self.response_weight
    self.velocity = predicted_velocity + alpha * residual + speed_rc * (self.acceleration - predicted_acceleration)
    return self.acceleration
