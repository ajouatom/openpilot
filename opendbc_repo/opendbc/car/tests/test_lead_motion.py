import math
import random
import time

import pytest

from opendbc.car.lead_motion import LeadMotionIMM


def test_quiet_velocity_noise_is_rejected() -> None:
  rng = random.Random(7)
  estimator = LeadMotionIMM(20.0)
  estimates = []
  for _ in range(200):
    estimates.append(estimator.update(20.0 + rng.gauss(0.0, 0.035)))

  tail = estimates[50:]
  rms_acceleration = math.sqrt(sum(
    estimate.acceleration ** 2 for estimate in tail
  ) / len(tail))
  assert rms_acceleration < 0.12
  assert sum(
    estimate.maneuver_probability for estimate in tail
  ) / len(tail) < 0.20


def test_braking_step_is_seen_within_two_radar_cycles() -> None:
  estimator = LeadMotionIMM(20.0)
  velocity = 20.0
  for _ in range(30):
    estimator.update(velocity)

  first_detected = None
  for index in range(8):
    velocity += -2.0 * 0.05
    estimate = estimator.update(velocity)
    if estimate.acceleration < -0.30 and first_detected is None:
      first_detected = index

  assert first_detected is not None
  assert first_detected <= 1
  assert estimate.acceleration < -1.4


def test_native_acceleration_provides_first_cycle_attack() -> None:
  estimator = LeadMotionIMM(20.0)
  for _ in range(20):
    estimator.update(20.0, native_acceleration=0.0)

  estimate = estimator.update(19.95, native_acceleration=-2.0)

  assert estimate.used_native_acceleration
  assert estimate.acceleration < -0.35
  assert estimate.jerk < -0.5


def test_single_velocity_outlier_does_not_create_unbounded_acceleration() -> None:
  estimator = LeadMotionIMM(15.0)
  for _ in range(30):
    estimator.update(15.0)

  outlier = estimator.update(12.0)
  recovered = [estimator.update(15.0) for _ in range(10)]

  assert abs(outlier.acceleration) < 4.0
  assert abs(recovered[-1].acceleration) < 0.5


@pytest.mark.parametrize("dt", (0.02, 0.033, 0.05, 0.10))
def test_variable_radar_cadence_tracks_constant_acceleration(dt: float) -> None:
  estimator = LeadMotionIMM(10.0)
  velocity = 10.0
  estimate = estimator.estimate()
  for _ in range(max(20, int(2.0 / dt))):
    velocity += 1.0 * dt
    estimate = estimator.update(velocity, dt=dt)

  assert estimate.acceleration == pytest.approx(1.0, abs=0.20)


def test_scalar_imm_has_small_per_track_runtime() -> None:
  estimator = LeadMotionIMM(20.0)
  start = time.perf_counter()
  iterations = 20_000
  for index in range(iterations):
    estimator.update(20.0 + 0.05 * math.sin(index * 0.03))
  elapsed = time.perf_counter() - start

  # A generous cross-platform guard. Current desktop Python is normally well
  # below 20 us/update, leaving ample margin for dozens of tracks at 20 Hz.
  assert elapsed / iterations < 80e-6
