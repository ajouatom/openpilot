"""Radar pair evidence must be independent of uncertain vision and noisy corners."""
from dataclasses import replace

import pytest

from openpilot.selfdrive.carrot.radar_motion.controller import DPathRadarController
from openpilot.selfdrive.carrot.radar_motion.primary import VisionRadarMatcher, snapshot_radar_points, vision_lead_from_model
from openpilot.selfdrive.carrot.tests.test_radar_motion_predictor import Point, model_with_lead


@pytest.mark.parametrize("mode", (1, 2, 3))
@pytest.mark.parametrize("change", (
  "paired", "missing_corner", "corner_distance", "corner_lateral", "corner_speed",
  "tentative_front", "unmeasured_corner", "off_path", "vision_lateral", "brief_pair",
  "turn", "vision_too_far", "vision_too_fast",
))
def test_large_visual_range_and_speed_error_needs_agreeing_confirmed_radars(mode, change):
  controller = DPathRadarController(enable_radar_tracks=mode, prefer_corner_radar=True, cut_in_sensitivity=0)
  first_front_s = None
  for index in range(20):
    time_s = index * 0.05
    distance = 80.0 - 20.0 * time_s
    front = Point(40, distance, 0.0, v_rel=-20.0, trackState=2)
    corner = Point(1400, distance + 0.3, 0.2, v_rel=-20.0, source="corner235")
    model = model_with_lead(distance + 35.0, 0.0, 25.0)
    if change == "corner_distance":
      corner = replace(corner, d_rel=distance + 7.0)
    elif change == "corner_lateral":
      corner = replace(corner, y_rel=1.2)
    elif change == "corner_speed":
      corner = replace(corner, v_rel=-15.0)
    elif change == "tentative_front":
      front = replace(front, trackState=1)
    elif change == "unmeasured_corner":
      corner = replace(corner, measured=False)
    elif change == "off_path":
      front, corner = replace(front, y_rel=3.0), replace(corner, y_rel=3.2)
    elif change == "vision_lateral":
      model.leadsV3[0].y = (-1.5,)
    elif change == "vision_too_far":
      model = model_with_lead(distance + 65.0, 0.0, 25.0)
    elif change == "vision_too_fast":
      model = model_with_lead(distance + 35.0, 0.0, 31.0)
    points = [front]
    if change != "missing_corner" and not (change == "brief_pair" and index >= 2):
      points.append(corner)
    output = controller.update(time_s, 20.0, points, model, yaw_rate_rad_s=0.03 if change == "turn" else 0.0)
    if output.lead_one is not None and output.lead_one["radarTrackId"] == 40:
      first_front_s = time_s if first_front_s is None else first_front_s
      assert output.lead_one["dRel"] == pytest.approx(distance)
      assert output.lead_one["vLead"] == pytest.approx(0.0)
  if change == "paired":
    assert first_front_s == pytest.approx(0.25)
  else:
    assert first_front_s is None


@pytest.mark.parametrize("mode", (1, 2, 3))
@pytest.mark.parametrize("moving_corner_consistent", (False, True))
def test_corner_cannot_veto_slow_front_using_inconsistent_range_velocity(mode, moving_corner_consistent):
  controller = DPathRadarController(enable_radar_tracks=mode, prefer_corner_radar=True, cut_in_sensitivity=0)
  for index in range(30):
    time_s = index * 0.05
    front_distance = 110.0 - 20.0 * time_s
    corner_distance = 110.0 - (10.0 if moving_corner_consistent else 20.0) * time_s
    points = [
      Point(40, front_distance, 0.0, v_rel=-20.0, trackState=2),
      Point(1400, corner_distance, 0.1, v_rel=-10.0, source="corner235"),
    ]
    model = model_with_lead(corner_distance, 0.0, 10.0, probability=0.65)
    output = controller.update(time_s, 20.0, points, model)
    if index >= 16:
      if moving_corner_consistent:
        assert output.lead_one is None or output.lead_one["radarTrackId"] != 40
      else:
        assert output.lead_one["radarTrackId"] == 40


def corner_snapshot(distance, v_rel=-10.0):
  return snapshot_radar_points((Point(1400, distance, 0.0, v_rel=v_rel, source="corner235"),), 20.0, 0.0)[0]


@pytest.mark.parametrize("preferred", (None, ("frontRadar", 40), ("frontRadar", 41)))
@pytest.mark.parametrize("large_error", (False, True))
def test_broader_visual_support_does_not_steal_another_pending_or_held_front(preferred, large_error):
  front, corner = snapshot_radar_points((
    Point(40, 80.0, 0.0, v_rel=-20.0, trackState=2),
    Point(1400, 80.3, 0.2, v_rel=-20.0, source="corner235"),
  ), 20.0, 0.0)
  vision = vision_lead_from_model(model_with_lead(115.0 if large_error else 80.0, 0.0, 25.0 if large_error else 0.0))
  support = VisionRadarMatcher._stationary_vision_cross_source_front_support(
    vision, ((front, 0.0, corner, 0.2),), preferred_identity=preferred,
  )
  assert bool(support) == (not large_error or preferred in (None, ("frontRadar", 40)))


def test_corner_range_jump_does_not_rehabilitate_id_with_short_quiet_suffix():
  matcher = VisionRadarMatcher()
  for index in range(31):
    time_s = index * 0.05
    point = corner_snapshot(110.0 - 10.0 * time_s + (8.0 if index >= 12 else 0.0))
    matcher._update_corner_motion_history((point,), time_s)
    consistent = matcher._corner_motion_consistent(point, time_s)
    if 10 <= index < 12:
      assert consistent
    if 12 <= index <= 30:
      assert not consistent
  for index in range(31, 43):
    time_s = index * 0.05
    point = corner_snapshot(118.0 - 10.0 * time_s)
    matcher._update_corner_motion_history((point,), time_s)
  assert matcher._corner_motion_consistent(point, time_s)


@pytest.mark.parametrize("change", ("reset", "gap", "backwards", "duplicate", "invalid_time"))
def test_corner_consistency_needs_fresh_continuous_history(change):
  matcher = VisionRadarMatcher()
  for index in range(21):
    time_s = index * 0.05
    point = corner_snapshot(110.0 - 10.0 * time_s)
    matcher._update_corner_motion_history((point,), time_s)
  assert matcher._corner_motion_consistent(point, time_s)
  if change == "reset":
    matcher.reset()
  else:
    time_s = {"gap": 1.25, "backwards": 0.5, "duplicate": 1.0, "invalid_time": float("nan")}[change]
    point = corner_snapshot(110.0 - 10.0 * time_s if change != "invalid_time" else 100.0)
    matcher._update_corner_motion_history((point,), time_s)
  assert not matcher._corner_motion_consistent(point, time_s)
