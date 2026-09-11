import math

import numpy as np
import pytest

from opendbc.car.radar_lead_filter import RadarLeadFilter


def historical_reference(velocity, dt):
  """Independent original RC=.10/.15 filter, without the old publication lag."""
  speed, acceleration = velocity[0], 0.0
  result = []
  for measured in velocity:
    previous_speed = speed
    speed += dt / (.10 + dt) * (measured - speed)
    sample = np.clip((speed - previous_speed) / dt, -10., 5.)
    sample = np.clip(sample, acceleration - 3., acceleration + 3.)
    acceleration += dt / (.15 + dt) * (sample - acceleration)
    result.append(acceleration)
  return np.array(result)


def estimate(velocity, dt):
  observer = RadarLeadFilter(float(velocity[0]), dt)
  return np.array([observer.update(float(v)) for v in velocity])


@pytest.mark.parametrize('dt', [.02, .05, .1])
@pytest.mark.parametrize('initial', [0., 1.5])
@pytest.mark.parametrize('braking', [-.5, -1., -3., -6.])
@pytest.mark.parametrize('phase', [0., .0125, .025, .0375])
def test_braking_is_not_later_than_original_filter_without_publication_delay(dt, initial, braking, phase):
  t = np.arange(int(8 / dt)) * dt
  onset = 2. + phase
  velocity = 80. + initial*t + (braking-initial)*np.maximum(0., t-onset)
  reference, observed = historical_reference(velocity, dt), estimate(velocity, dt)
  for fraction in [.1, .5, .9]:
    threshold = initial + fraction * (braking-initial)
    before = np.flatnonzero((t >= onset) & (reference <= threshold))[0]
    after = np.flatnonzero((t >= onset) & (observed <= threshold))[0]
    assert after <= before
  assert observed.min() >= braking - 1e-5
  assert observed[-1] == pytest.approx(braking, abs=1e-4)


@pytest.mark.parametrize('braking', [-3., -6.])
def test_clear_braking_settles_at_least_100ms_earlier(braking):
  dt = .05
  t = np.arange(160)*dt
  velocity = 80.+braking*np.maximum(0., t-2.)
  reference, observed = historical_reference(velocity, dt), estimate(velocity, dt)
  before = np.flatnonzero((t >= 2.) & (reference <= .9*braking))[0]
  after = np.flatnonzero((t >= 2.) & (observed <= .9*braking))[0]
  assert after <= before - 2


@pytest.mark.parametrize('dt', [.02, .05, .1])
def test_slow_observer_matches_original_even_when_corrections_saturate(dt):
  velocity = 20.+np.random.default_rng(391).normal(0., .5, 1000)
  velocity[40] += 3.
  observer = RadarLeadFilter(float(velocity[0]), dt)
  observer.residual_scale_squared = math.inf  # Disable adaptation for this equivalence check.
  observed = [observer.update(float(v)) for v in velocity]
  assert np.allclose(observed, historical_reference(velocity, dt), atol=1e-10)


@pytest.mark.parametrize('dt', [.02, .05, .1])
@pytest.mark.parametrize('acceleration', [0., 1.5, -3.])
@pytest.mark.parametrize('rho', [0., .5, .9])
@pytest.mark.parametrize('sigma', [.03, .1, .3])
def test_steady_motion_noise_stays_within_original_filter_level(dt, acceleration, rho, sigma):
  rng = np.random.default_rng(7291)
  noise = 0.
  velocity = []
  for i in range(3000):
    noise = rho*noise + math.sqrt(1.-rho*rho)*rng.normal(0., sigma)
    velocity.append(10000.+acceleration*i*dt+noise)
  reference = historical_reference(velocity, dt)[200:]
  observed = estimate(velocity, dt)[200:]
  assert np.std(observed) <= 1.05*np.std(reference)
  assert abs(observed.sum()/len(observed)-acceleration) < .03


@pytest.mark.parametrize('dt', [.02, .05, .1])
@pytest.mark.parametrize('spike', [-2., -.5, -.1, .1, .5, 2.])
def test_isolated_speed_spike_does_not_enable_fast_response_or_leave_a_tail(dt, spike):
  velocity = np.full(int(8/dt), 20.)
  at = int(2/dt)
  velocity[at] += spike
  observer = RadarLeadFilter(20., dt)
  result = []
  for i, value in enumerate(velocity):
    result.append(observer.update(float(value)))
    if i == at:
      assert observer.response_weight == 0.
  reference = historical_reference(velocity, dt)
  assert max(abs(np.array(result))) <= 1.05*max(abs(reference))
  assert abs(result[-1]) < 1e-5


@pytest.mark.parametrize('dt', [.02, .05, .1])
def test_reset_discards_motion_and_noise_history(dt):
  observer = RadarLeadFilter(20., dt)
  for i in range(50):
    observer.update(20.-2.*i*dt)
  assert observer.acceleration < -1.
  observer.reset(30.)
  for _ in range(30):
    assert observer.update(30.) == 0.
    assert observer.response_weight == 0.


def test_stationary_gate_decays_acceleration_without_positive_rebound():
  observer = RadarLeadFilter(20., .05)
  for i in range(60):
    observer.update(20.-i*.1)
  values = [observer.update(0., stationary=True) for _ in range(80)]
  assert all(a <= 0. for a in values)
  assert all(b >= a for a, b in zip(values[:-1], values[1:], strict=True))
  assert abs(values[-1]) < 1e-5


@pytest.mark.parametrize('dt', [0., -.05, float('nan'), float('inf')])
def test_invalid_sample_period_is_rejected(dt):
  with pytest.raises(ValueError):
    RadarLeadFilter(20., dt)
