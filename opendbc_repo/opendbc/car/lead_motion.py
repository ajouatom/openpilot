"""Low-latency longitudinal lead-motion estimation.

This module intentionally uses scalar arithmetic only. Radar parsing runs over
many objects, so allocating NumPy matrices for every track and radar cycle costs
more than the small two-state filters themselves.

The estimator is an interacting multiple model (IMM) with two constant-
acceleration Kalman filters:

* ``steady`` rejects quantisation and ego/radar timestamp noise.
* ``maneuver`` accepts real acceleration changes with much less delay.

Both models observe radar velocity. A radar-native relative acceleration can be
fused as a second measurement after it is converted to lead acceleration by the
caller. The native measurement is robustly gated; velocity remains the
authoritative measurement when the two disagree.
"""

from __future__ import annotations

from dataclasses import dataclass
import math


MIN_DT_S = 0.015
MAX_DT_S = 0.20
DEFAULT_DT_S = 0.05

# Markov transition probabilities per nominal 50 ms radar cycle. They are
# rescaled for the measured dt in ``_transition_probabilities``.
STEADY_TO_MANEUVER = 0.010
MANEUVER_TO_STEADY = 0.20

# Continuous jerk-noise variances. The steady model is deliberately quiet;
# the maneuver model can move by several m/s^2 in two radar samples.
STEADY_JERK_VARIANCE = 0.04
MANEUVER_JERK_VARIANCE = 120.0

VELOCITY_MEASUREMENT_VARIANCE = 0.08 ** 2
# Native radar acceleration is an early event sensor, but its quantisation is
# too visible for the quiet model. The maneuver model trusts it immediately;
# the steady model only follows a persistent velocity change. Native
# acceleration does not directly score the models: it can promote maneuver
# mode only when the simultaneous velocity innovation corroborates it.
STEADY_NATIVE_ACCEL_MEASUREMENT_VARIANCE = 4.00 ** 2
MANEUVER_NATIVE_ACCEL_MEASUREMENT_VARIANCE = 0.20 ** 2
NATIVE_ACCEL_MAX_ABS_MPS2 = 8.0

VELOCITY_UPDATE_GATE_SIGMA = 6.0
VELOCITY_UPDATE_MIN_GATE_MPS = 0.35
ACCEL_UPDATE_GATE_SIGMA = 5.0
ACCEL_UPDATE_MIN_GATE_MPS2 = 1.0
NATIVE_ACCEL_DIRECT_FUSION_DELTA_MPS2 = 0.75
NATIVE_ACCEL_CORROBORATION_PRODUCT = 0.008

MIN_MODEL_PROBABILITY = 1e-4
MAX_ABS_ACCEL_MPS2 = 10.0
MAX_ABS_JERK_MPS3 = 8.0


@dataclass
class _KalmanModel:
  velocity: float
  acceleration: float
  p_vv: float
  p_va: float
  p_aa: float

  def copy(self) -> _KalmanModel:
    return _KalmanModel(
      self.velocity,
      self.acceleration,
      self.p_vv,
      self.p_va,
      self.p_aa,
    )


@dataclass(frozen=True)
class LeadMotionEstimate:
  acceleration: float
  acceleration_slow: float
  acceleration_fast: float
  jerk: float
  maneuver_probability: float
  velocity_innovation: float
  used_native_acceleration: bool


def _finite(value: float | None) -> bool:
  return value is not None and math.isfinite(float(value))


def _clamp(value: float, lower: float, upper: float) -> float:
  return min(upper, max(lower, value))


def _normal_likelihood(innovation: float, variance: float) -> float:
  variance = max(variance, 1e-9)
  exponent = min(50.0, 0.5 * innovation * innovation / variance)
  return math.exp(-exponent) / math.sqrt(variance)


def _transition_probability(nominal_probability: float, dt: float) -> float:
  # Preserve the same dwell time when a vehicle radar runs at a cadence other
  # than the common 20 Hz cycle.
  cycles = dt / DEFAULT_DT_S
  return 1.0 - (1.0 - nominal_probability) ** cycles


class LeadMotionIMM:
  """Two-model scalar IMM for one physical radar track."""

  def __init__(
    self,
    velocity: float,
    acceleration: float = 0.0,
  ) -> None:
    self.reset(velocity, acceleration)

  def reset(self, velocity: float, acceleration: float = 0.0) -> None:
    velocity = float(velocity) if _finite(velocity) else 0.0
    acceleration = (
      _clamp(float(acceleration), -MAX_ABS_ACCEL_MPS2, MAX_ABS_ACCEL_MPS2)
      if _finite(acceleration) else 0.0
    )
    initial = _KalmanModel(
      velocity=velocity,
      acceleration=acceleration,
      p_vv=VELOCITY_MEASUREMENT_VARIANCE,
      p_va=0.0,
      p_aa=2.0 ** 2,
    )
    self.steady = initial.copy()
    self.maneuver = initial.copy()
    self.steady_probability = 0.90
    self.maneuver_probability = 0.10
    self.acceleration = acceleration
    self.jerk = 0.0
    self.velocity_innovation = 0.0
    self.count = 0

  @staticmethod
  def _mix_model(
    first: _KalmanModel,
    second: _KalmanModel,
    first_weight: float,
    second_weight: float,
  ) -> _KalmanModel:
    velocity = first_weight * first.velocity + second_weight * second.velocity
    acceleration = (
      first_weight * first.acceleration
      + second_weight * second.acceleration
    )
    first_dv = first.velocity - velocity
    first_da = first.acceleration - acceleration
    second_dv = second.velocity - velocity
    second_da = second.acceleration - acceleration
    return _KalmanModel(
      velocity=velocity,
      acceleration=acceleration,
      p_vv=(
        first_weight * (first.p_vv + first_dv * first_dv)
        + second_weight * (second.p_vv + second_dv * second_dv)
      ),
      p_va=(
        first_weight * (first.p_va + first_dv * first_da)
        + second_weight * (second.p_va + second_dv * second_da)
      ),
      p_aa=(
        first_weight * (first.p_aa + first_da * first_da)
        + second_weight * (second.p_aa + second_da * second_da)
      ),
    )

  def _interact(self, dt: float) -> tuple[float, float]:
    steady_to_maneuver = _transition_probability(STEADY_TO_MANEUVER, dt)
    maneuver_to_steady = _transition_probability(MANEUVER_TO_STEADY, dt)
    p_ss = 1.0 - steady_to_maneuver
    p_sm = steady_to_maneuver
    p_ms = maneuver_to_steady
    p_mm = 1.0 - maneuver_to_steady

    steady_prior = (
      p_ss * self.steady_probability
      + p_ms * self.maneuver_probability
    )
    maneuver_prior = (
      p_sm * self.steady_probability
      + p_mm * self.maneuver_probability
    )
    steady_prior = max(steady_prior, MIN_MODEL_PROBABILITY)
    maneuver_prior = max(maneuver_prior, MIN_MODEL_PROBABILITY)

    steady_from_steady = p_ss * self.steady_probability / steady_prior
    steady_from_maneuver = p_ms * self.maneuver_probability / steady_prior
    maneuver_from_steady = p_sm * self.steady_probability / maneuver_prior
    maneuver_from_maneuver = p_mm * self.maneuver_probability / maneuver_prior

    old_steady = self.steady
    old_maneuver = self.maneuver
    self.steady = self._mix_model(
      old_steady,
      old_maneuver,
      steady_from_steady,
      steady_from_maneuver,
    )
    self.maneuver = self._mix_model(
      old_steady,
      old_maneuver,
      maneuver_from_steady,
      maneuver_from_maneuver,
    )
    return steady_prior, maneuver_prior

  @staticmethod
  def _predict(model: _KalmanModel, dt: float, jerk_variance: float) -> None:
    dt2 = dt * dt
    dt3 = dt2 * dt
    dt4 = dt2 * dt2
    model.velocity += model.acceleration * dt
    p_vv = (
      model.p_vv + 2.0 * dt * model.p_va + dt2 * model.p_aa
      + 0.25 * jerk_variance * dt4
    )
    p_va = model.p_va + dt * model.p_aa + 0.5 * jerk_variance * dt3
    p_aa = model.p_aa + jerk_variance * dt2
    model.p_vv = max(p_vv, 1e-9)
    model.p_va = p_va
    model.p_aa = max(p_aa, 1e-9)

  @staticmethod
  def _update_velocity(
    model: _KalmanModel,
    measured_velocity: float,
  ) -> tuple[float, float]:
    raw_innovation = measured_velocity - model.velocity
    innovation_variance = model.p_vv + VELOCITY_MEASUREMENT_VARIANCE
    gate = max(
      VELOCITY_UPDATE_MIN_GATE_MPS,
      VELOCITY_UPDATE_GATE_SIGMA * math.sqrt(innovation_variance),
    )
    innovation = _clamp(raw_innovation, -gate, gate)
    gain_v = model.p_vv / innovation_variance
    gain_a = model.p_va / innovation_variance
    old_p_vv = model.p_vv
    old_p_va = model.p_va
    model.velocity += gain_v * innovation
    model.acceleration += gain_a * innovation
    model.p_vv = max((1.0 - gain_v) * old_p_vv, 1e-9)
    model.p_va = (1.0 - gain_v) * old_p_va
    model.p_aa = max(model.p_aa - gain_a * old_p_va, 1e-9)
    return raw_innovation, _normal_likelihood(
      raw_innovation,
      innovation_variance,
    )

  @staticmethod
  def _update_acceleration(
    model: _KalmanModel,
    measured_acceleration: float,
    measurement_variance: float,
  ) -> None:
    raw_innovation = measured_acceleration - model.acceleration
    innovation_variance = model.p_aa + measurement_variance
    gate = max(
      ACCEL_UPDATE_MIN_GATE_MPS2,
      ACCEL_UPDATE_GATE_SIGMA * math.sqrt(innovation_variance),
    )
    innovation = _clamp(raw_innovation, -gate, gate)
    gain_v = model.p_va / innovation_variance
    gain_a = model.p_aa / innovation_variance
    old_p_vv = model.p_vv
    old_p_va = model.p_va
    old_p_aa = model.p_aa
    model.velocity += gain_v * innovation
    model.acceleration += gain_a * innovation
    model.p_vv = max(old_p_vv - gain_v * old_p_va, 1e-9)
    model.p_va = old_p_va - gain_v * old_p_aa
    model.p_aa = max((1.0 - gain_a) * old_p_aa, 1e-9)

  def update(
    self,
    velocity: float,
    dt: float = DEFAULT_DT_S,
    native_acceleration: float | None = None,
  ) -> LeadMotionEstimate:
    if not _finite(velocity):
      return self.estimate(False)
    dt = _clamp(
      float(dt) if _finite(dt) else DEFAULT_DT_S,
      MIN_DT_S,
      MAX_DT_S,
    )
    velocity = float(velocity)
    use_native_acceleration = (
      _finite(native_acceleration)
      and abs(float(native_acceleration)) <= NATIVE_ACCEL_MAX_ABS_MPS2
    )
    native_acceleration_value = (
      float(native_acceleration) if use_native_acceleration else 0.0
    )

    steady_prior, maneuver_prior = self._interact(dt)
    self._predict(self.steady, dt, STEADY_JERK_VARIANCE)
    self._predict(self.maneuver, dt, MANEUVER_JERK_VARIANCE)

    steady_innovation, steady_likelihood = self._update_velocity(
      self.steady, velocity,
    )
    maneuver_innovation, maneuver_likelihood = self._update_velocity(
      self.maneuver, velocity,
    )
    # Radar acceleration fields can contain the previous occupant of a reused
    # object slot for a few cycles. Fuse a large change only when the velocity
    # measurement moves in the same direction in this cycle. Small changes are
    # safe to use directly and keep the quiet-state estimate responsive.
    native_delta = native_acceleration_value - self.acceleration
    native_acceleration_corroborated = (
      abs(native_delta) <= NATIVE_ACCEL_DIRECT_FUSION_DELTA_MPS2
      or native_delta * steady_innovation > NATIVE_ACCEL_CORROBORATION_PRODUCT
    )
    fuse_native_acceleration = (
      use_native_acceleration and native_acceleration_corroborated
    )
    if fuse_native_acceleration:
      self._update_acceleration(
        self.steady,
        native_acceleration_value,
        STEADY_NATIVE_ACCEL_MEASUREMENT_VARIANCE,
      )
      self._update_acceleration(
        self.maneuver,
        native_acceleration_value,
        MANEUVER_NATIVE_ACCEL_MEASUREMENT_VARIANCE,
      )

    steady_posterior = steady_prior * max(steady_likelihood, 1e-12)
    maneuver_posterior = maneuver_prior * max(maneuver_likelihood, 1e-12)
    probability_sum = steady_posterior + maneuver_posterior
    if probability_sum <= 1e-20:
      self.steady_probability = steady_prior
      self.maneuver_probability = maneuver_prior
    else:
      self.steady_probability = steady_posterior / probability_sum
      self.maneuver_probability = maneuver_posterior / probability_sum
    self.steady_probability = _clamp(
      self.steady_probability,
      MIN_MODEL_PROBABILITY,
      1.0 - MIN_MODEL_PROBABILITY,
    )
    self.maneuver_probability = 1.0 - self.steady_probability

    # A native acceleration sample is allowed to trigger an immediate attack
    # only when the velocity residual moves in the same direction. This keeps
    # a one-frame native spike out of the published acceleration. Radars that
    # do not expose aRel still enter maneuver mode after two corroborating
    # velocity residuals.
    event_probability = 0.0
    if fuse_native_acceleration:
      if (
        abs(native_delta) > 0.50
        and native_delta * steady_innovation > NATIVE_ACCEL_CORROBORATION_PRODUCT
      ):
        event_probability = float(_clamp(
          0.15 + (abs(native_delta) - 0.50) * (0.80 / 1.00),
          0.15,
          0.95,
        ))
    if (
      self.count > 0
      and steady_innovation * self.velocity_innovation > 0.0
      and min(abs(steady_innovation), abs(self.velocity_innovation)) > 0.075
    ):
      residual_sum = abs(steady_innovation) + abs(self.velocity_innovation)
      velocity_event_probability = _clamp(
        0.15 + (residual_sum - 0.15) * (0.75 / 0.25),
        0.15,
        0.90,
      )
      event_probability = max(event_probability, velocity_event_probability)
    if event_probability > self.maneuver_probability:
      self.maneuver_probability = event_probability
      self.steady_probability = 1.0 - event_probability

    previous_acceleration = self.acceleration
    target_acceleration = _clamp(
      self.steady_probability * self.steady.acceleration
      + self.maneuver_probability * self.maneuver.acceleration,
      -MAX_ABS_ACCEL_MPS2,
      MAX_ABS_ACCEL_MPS2,
    )
    self.acceleration = target_acceleration
    raw_jerk = _clamp(
      (self.acceleration - previous_acceleration) / dt,
      -MAX_ABS_JERK_MPS3,
      MAX_ABS_JERK_MPS3,
    )
    # aLead carries the fast physical response. jLead is only an event cue for
    # DynamicTFollow and lead extrapolation, so open it on corroborated event
    # evidence rather than on every noisy change of IMM model probability.
    jerk_event = event_probability >= 0.15
    jerk_target = raw_jerk if jerk_event else 0.0
    jerk_alpha = 0.45 if jerk_event else 0.30
    self.jerk += jerk_alpha * (jerk_target - self.jerk)
    self.velocity_innovation = (
      self.steady_probability * steady_innovation
      + self.maneuver_probability * maneuver_innovation
    )
    self.count += 1
    return self.estimate(fuse_native_acceleration)

  def estimate(self, used_native_acceleration: bool = False) -> LeadMotionEstimate:
    return LeadMotionEstimate(
      acceleration=float(self.acceleration),
      acceleration_slow=float(self.steady.acceleration),
      acceleration_fast=float(self.maneuver.acceleration),
      jerk=float(self.jerk),
      maneuver_probability=float(self.maneuver_probability),
      velocity_innovation=float(self.velocity_innovation),
      used_native_acceleration=bool(used_native_acceleration),
    )

  @property
  def steady_velocity(self) -> float:
    return float(self.steady.velocity)


__all__ = (
  "LeadMotionEstimate",
  "LeadMotionIMM",
)
