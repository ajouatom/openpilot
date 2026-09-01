from collections import deque
from types import SimpleNamespace

import pytest

from opendbc.car.interfaces import MyTrack, estimate_radar_jerk


def radar_point(v_lead: float, *, measured: bool = True, source: str = "frontRadar"):
  return SimpleNamespace(
    dRel=30.0,
    yRel=0.0,
    vRel=v_lead - 20.0,
    yvRel=0.0,
    vLead=v_lead,
    measured=measured,
    radarSource=source,
  )


def test_continuous_track_keeps_acceleration_during_hard_braking():
  dt = 0.05
  point = radar_point(20.0)
  track = MyTrack(37, point, dt)

  for _ in range(8):
    track.update(point, 0.0)

  assert track.cnt >= 6
  assert track.aLead == pytest.approx(0.0)

  track.update(radar_point(19.0), 0.0)

  assert track.noisy
  assert track.cnt >= 6
  assert track.aLead == pytest.approx(-1.5)

  for v_lead in (18.5, 18.0, 17.5, 17.0, 16.5):
    track.update(radar_point(v_lead), 0.0)

  assert track.cnt >= 6
  assert track.aLead < -2.0


def test_new_track_still_requires_acceleration_warmup():
  point = radar_point(20.0)
  track = MyTrack(37, point, 0.05)

  track.update(point, 0.0)

  assert track.cnt == 1
  assert track.aLead == pytest.approx(0.0)


def test_scc_single_slot_still_restarts_on_large_velocity_jump():
  dt = 0.05
  point = radar_point(20.0, source="scc")
  track = MyTrack(0, point, dt)

  for _ in range(8):
    track.update(point, 0.0)

  track.update(radar_point(19.0, source="scc"), 0.0)

  assert track.noisy
  assert track.cnt == 1


def test_quadratic_velocity_fit_recovers_jerk():
  dt = 0.05
  expected_jerk = -2.0
  history = deque(maxlen=11)
  for index in range(11):
    t = index * dt
    history.append(20.0 + 0.4 * t + 0.5 * expected_jerk * t ** 2)

  assert estimate_radar_jerk(history, dt) == pytest.approx(expected_jerk)


def test_jlead_tracks_quadratic_velocity_trend():
  dt = 0.05
  point = radar_point(20.0)
  track = MyTrack(37, point, dt)

  for _ in range(12):
    track.update(point, 0.0)

  expected_jerk = -2.0
  for index in range(1, 31):
    t = index * dt
    track.update(radar_point(20.0 + 0.5 * expected_jerk * t ** 2), 0.0)

  # The 0.20 m/s^3 soft deadband intentionally leaves a -1.8 m/s^3 target.
  assert track.jLead == pytest.approx(-1.8, abs=0.05)


def test_jlead_returns_to_zero_for_constant_acceleration():
  dt = 0.05
  point = radar_point(20.0)
  track = MyTrack(37, point, dt)

  for _ in range(12):
    track.update(point, 0.0)
  for index in range(1, 31):
    track.update(radar_point(20.0 - 0.5 * index * dt), 0.0)

  assert track.jLead == pytest.approx(0.0, abs=0.02)
