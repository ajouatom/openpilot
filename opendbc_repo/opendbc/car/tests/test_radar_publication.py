from collections import deque

import numpy as np
import pytest

from opendbc.car import structs
from opendbc.car.interfaces import MyTrack, RadarInterfaceBase


def point(track_id=52, speed=20., *, measured=True):
  return structs.RadarData.RadarPoint(trackId=track_id, dRel=30., yRel=0., vRel=speed-20., vLead=speed,
                                     yvRel=0., aRel=-.2, measured=measured, radarSource='frontRadar', trackState=3)


class CopyingRadar(RadarInterfaceBase):
  def __init__(self):
    super().__init__(structs.CarParams(radarTimeStep=.05, radarDelay=0.))
    self.pts = {101: point(), 102: point(63), 103: point(49, measured=False)}

  def update(self, _can_packets):
    ret = structs.RadarData()
    ret.errors.canError = True
    # Cap'n Proto copies these values before the base updates MyTrack.
    ret.points = [p for p in reversed(list(self.pts.values())) if p.measured]
    return ret


def step(radar, index, acceleration=-2.):
  for p in radar.pts.values():
    p.vLead = 20. + acceleration * index * .05
    p.vRel = p.vLead-20.
  return radar.update_carrot(20., 0., index*.05, [])


def test_published_acceleration_and_jerk_are_from_the_current_track_update():
  radar = CopyingRadar()
  for i in range(20):
    for p in radar.pts.values():
      t = i*.05
      p.vLead = 20.-t*t
      p.vRel = p.vLead-20.
    result = radar.update_carrot(20., 0., i*.05, [])
    if result is None:
      continue
    for p in result.points:
      track = radar.tracks[p.trackId]
      assert p.aLead == pytest.approx(track.aLead if track.cnt >= 6 else 0., abs=1e-6)
      assert p.jLead == pytest.approx(track.jLead if track.cnt >= 6 else 0., abs=1e-6)
  assert result.points[0].aLead < -1.
  assert result.points[0].jLead < -.5


def test_publication_preserves_selected_points_order_raw_lateral_values_and_metadata():
  radar = CopyingRadar()
  for i in range(10):
    step(radar, i)
  for p in radar.pts.values():
    p.yRel, p.yvRel = 1., .6
  result = step(radar, 10)
  assert result.errors.canError
  assert [p.trackId for p in result.points] == [63, 52]
  for p in result.points:
    assert p.yRel == pytest.approx(1.)
    assert p.yvRel == pytest.approx(.6)
    assert p.vLead == pytest.approx(19.)
    assert p.vRel == pytest.approx(-1.)
    assert p.dRel == pytest.approx(30.)
    assert p.aRel == pytest.approx(-.2)
    assert p.trackState == 3 and p.measured and str(p.radarSource) == 'frontRadar'
    assert radar.tracks[p.trackId].yRel < p.yRel


def test_new_and_reacquired_tracks_do_not_publish_old_acceleration():
  radar = CopyingRadar()
  for i in range(12):
    result = step(radar, i)
  assert result.points[-1].aLead < -1.
  radar.pts[101].measured = False
  assert [p.trackId for p in step(radar, 12).points] == [63]
  radar.pts[101].measured = True
  for i in range(13, 17):
    p = step(radar, i).points[-1]
    assert p.trackId == 52
    assert p.aLead == p.jLead == 0.


def legacy_published(velocities, dt):
  """Reference: former 3-sample difference, RC=.05, and one-frame copy lag."""
  history = deque(maxlen=3)
  accel = 0.
  output = []
  for v in velocities:
    output.append(accel)
    history.append(v)
    raw = (history[-1]-history[0])/(2*dt) if len(history) == 3 else 0.
    sample = np.clip(raw, -10., 5.)
    sample = np.clip(sample, accel-3., accel+3.)
    alpha = dt/(.05+dt)
    accel += alpha*(sample-accel)
  return np.array(output)


def current_estimate(velocities, dt):
  p = point(speed=float(velocities[0]))
  track = MyTrack(52, p, dt)
  output = []
  for v in velocities:
    p.vLead = float(v)
    p.vRel = float(v)-20.
    track.update(p, 0.)
    track.write_acceleration(p)
    output.append(p.aLead)
  return np.array(output)


@pytest.mark.parametrize('initial', [0., 1.5])
@pytest.mark.parametrize('braking', [-.5, -1., -3., -6.])
@pytest.mark.parametrize('phase', [0., .0125, .025, .0375])
def test_braking_thresholds_are_not_later_than_the_previous_published_estimate(initial, braking, phase):
  dt = .05
  t = np.arange(120)*dt
  onset = 2.+phase
  speeds = 30.+initial*t+(braking-initial)*np.maximum(0., t-onset)
  before, after = legacy_published(speeds, dt), current_estimate(speeds, dt)
  for fraction in [.1, .5, .9]:
    threshold = initial+(braking-initial)*fraction
    before_index = np.flatnonzero((t >= onset) & (before <= threshold))[0]
    after_index = np.flatnonzero((t >= onset) & (after <= threshold))[0]
    assert after_index <= before_index


def test_constant_speed_measurement_noise_is_reduced_without_an_extra_filter():
  velocities = 20.+np.random.default_rng(410).normal(0., .03, 2000)
  before, after = legacy_published(velocities, .05)[100:], current_estimate(velocities, .05)[100:]
  assert np.std(after) < .8*np.std(before)


def test_scc_target_replacement_discriminator_retains_three_sample_response():
  p = point(track_id=0)
  p.radarSource = 'scc'
  track = MyTrack(0, p, .05)
  for _ in range(12):
    track.update(p, 0.)
  # The old 0.1 s difference detects this reusable-slot jump immediately;
  # a longer slope window must not hide it from the SCC reset condition.
  p.vLead = 19.6
  track.update(p, 0.)
  assert track.noisy
  assert track.cnt == 1
  track.write_acceleration(p)
  assert p.aLead == p.jLead == 0.


@pytest.mark.parametrize('acceleration', [-3., -.5, 0., 1., 3.])
@pytest.mark.parametrize('dt', [.02, .05, .1])
def test_constant_acceleration_remains_unbiased(acceleration, dt):
  t = np.arange(100)*dt
  estimates = current_estimate(40.+acceleration*t, dt)
  assert estimates[-1] == pytest.approx(acceleration, abs=1e-4)
