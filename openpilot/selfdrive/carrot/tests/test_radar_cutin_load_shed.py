# RadarCutInLoadShed (radar setting, default off): radard sheds cut-in work only while its own rate is below 18 Hz.
# IONIQ 9 route 000004cd--da4e1b0f78 seg26-27 (rain, 8 km/h multi-lane jam): guardrail/rain reflections plus next-lane
# cars put ~27 points per frame through the cut-in detector, radard fell to 16-17 Hz and selfdrived disengaged.
from types import SimpleNamespace

from openpilot.selfdrive.carrot.radar_motion.controller import CutInLoadShed, shed_cutin_points
from openpilot.selfdrive.carrot.radar_motion.trajectory_cutin import MIN_MOVING_VLEAD_MPS


def feed(shed, hz, seconds, t0):
  t = t0
  for _ in range(int(hz * seconds)):
    t += 1.0 / hz
    shed.update(t)
  return t


def test_off_at_normal_rate_and_when_disabled():
  shed = CutInLoadShed(True)
  feed(shed, 20.0, 5, 0.)
  assert not shed.active  # healthy logs: 19.0 Hz at worst
  off = CutInLoadShed(False)
  feed(off, 10.0, 5, 0.)
  assert not off.active  # the setting is off: never


def test_on_below_18_hz_and_off_after_3_s_back_at_rate():
  shed = CutInLoadShed(True)
  t = feed(shed, 16.5, 2, 0.)  # the rainy jam ran a 16.6 Hz median
  assert shed.active
  t = feed(shed, 20.0, 2.5, t)
  assert shed.active  # 18.9 Hz or more for less than 3 s
  t = feed(shed, 18.5, 1, t)  # a slow stretch restarts the hold
  t = feed(shed, 20.0, 2.5, t)
  assert shed.active
  feed(shed, 20.0, 2, t)
  assert not shed.active


def test_one_late_frame_does_not_restart_the_hold():
  shed = CutInLoadShed(True)
  t = feed(shed, 16.5, 2, 0.)
  t = feed(shed, 20.0, 2.0, t)  # the window is clean after ~1.05 s: the 3 s hold starts there
  shed.update(t + 0.1)  # one 100 ms interval: 20 / 1.05 s = 19.05 Hz >= 18.9, the hold keeps running
  feed(shed, 20.0, 2.5, t + 0.1)  # ~3.5 s into the hold; restarting it at the late frame would still be on here
  assert not shed.active


def point(track_id, d_rel, v_lead, source="frontRadar"):
  return SimpleNamespace(track_id=track_id, d_rel=d_rel, v_lead=v_lead, source=source)


def test_shed_drops_only_points_the_detector_cannot_use_and_keeps_the_lead():
  slow = 30 / 3.6
  pts = (point(1, 20., 0.2), point(2, 30., 3.0), point(3, 60., 3.0), point(4, 10., -0.5), point(5, 70., 0.0),
         point(6, 15., 0.8))
  kept = shed_cutin_points(pts, {"radarTrackId": 5}, slow)
  # 1/4 at or below the detector's moving gate and 3 beyond 45 m are out; 0.8 m/s creeping car 6 stays; lead 5 stays
  assert MIN_MOVING_VLEAD_MPS == 0.5
  assert [p.track_id for p in kept] == [2, 5, 6]
  assert [p.track_id for p in shed_cutin_points(pts, None, slow)] == [2, 6]


def test_corner_track_standing_for_the_front_lead_is_kept():
  # corner mode: the matched front lead 7 is represented by corner track 1003 in motion_points
  pts = (point(1003, 8., 0.2, "corner235"), point(1012, 20., 3.0, "corner235"))
  matches = {("corner235", 1003): point(7, 8., 0.2)}
  kept = shed_cutin_points(pts, {"radarTrackId": 7}, 8 / 3.6, matches)
  assert [p.track_id for p in kept] == [1003, 1012]


def test_far_points_stay_at_50_kph_and_above():
  # replay: the 45 m rule lost the only real cut-in in the logs (99 km/h, first confirmed at 42 m)
  pts = (point(1, 20., 0.2), point(3, 60., 25.0))
  assert [p.track_id for p in shed_cutin_points(pts, None, 50 / 3.6)] == [3]
  assert [p.track_id for p in shed_cutin_points(pts, None, 49 / 3.6)] == []
