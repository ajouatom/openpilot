# RadarCutInLoadShed (Joongyu Lab, 11-17): radard sheds cut-in work only while its own rate is below 18 Hz.
# IONIQ 9 7465eee5 rainy 8 km/h jam: guardrail/rain reflections + next-lane cars put ~27 points per frame through the
# cut-in detector, radard fell to 16-17 Hz and selfdrived disengaged (02_logs/measurements.md s65).
from types import SimpleNamespace

from openpilot.selfdrive.carrot.radar_motion.controller import CutInLoadShed, shed_cutin_points


def feed(shed, hz, seconds, t0):
  t = t0
  for _ in range(int(hz * seconds)):
    t += 1.0 / hz
    shed.update(t)
  return t


def test_off_at_normal_rate_and_when_disabled():
  shed = CutInLoadShed(True)
  feed(shed, 20.0, 5, 0.)
  assert not shed.active  # normal logs: 19.0 Hz at worst
  off = CutInLoadShed(False)
  feed(off, 10.0, 5, 0.)
  assert not off.active  # the setting is off: never


def test_on_below_18_hz_and_off_after_3_s_back_at_rate():
  shed = CutInLoadShed(True)
  t = feed(shed, 16.5, 2, 0.)  # the rainy jam ran a 16.6 Hz median
  assert shed.active
  t = feed(shed, 20.0, 2.5, t)
  assert shed.active  # 19.5 Hz or more for less than 3 s
  t = feed(shed, 18.5, 1, t)  # a dip resets the hold
  t = feed(shed, 20.0, 2.5, t)
  assert shed.active
  feed(shed, 20.0, 2, t)
  assert not shed.active


def point(track_id, d_rel, v_lead):
  return SimpleNamespace(track_id=track_id, d_rel=d_rel, v_lead=v_lead)


def test_shed_keeps_moving_near_points_and_the_primary_lead():
  pts = (point(1, 20., 0.2), point(2, 30., 3.0), point(3, 60., 3.0), point(4, 10., -0.5), point(5, 70., 0.0))
  kept = shed_cutin_points(pts, {"radarTrackId": 5}, 30 / 3.6)
  assert [p.track_id for p in kept] == [2, 5]  # stationary 1/4 and far 3 out; lead 5 kept although far and still
  assert [p.track_id for p in shed_cutin_points(pts, None, 30 / 3.6)] == [2]


def test_far_points_stay_at_50_kph_and_above():
  # replay: the 45 m rule lost the only real cut-in in the logs (99 km/h, first seen at 42 m)
  pts = (point(1, 20., 0.2), point(3, 60., 25.0))
  assert [p.track_id for p in shed_cutin_points(pts, None, 50 / 3.6)] == [3]
  assert [p.track_id for p in shed_cutin_points(pts, None, 49 / 3.6)] == []
