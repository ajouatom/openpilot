from types import SimpleNamespace

import pytest

from opendbc.car.interfaces import MyTrack


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
  assert track.aLead == pytest.approx(-0.75)

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
