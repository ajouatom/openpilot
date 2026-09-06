"""Stationary identity evidence and bounded visual-range dropout regression tests."""
from dataclasses import replace

import pytest

from openpilot.selfdrive.carrot.radar_motion.controller import DPathRadarController
from openpilot.selfdrive.carrot.tests.test_radar_motion_predictor import Point, model_with_lead


def approach_frame(time_s, *, distance_error=1.0, x_std=12.0):
  distance = 120.0 - 23.0 * time_s
  point = Point(53, distance, 0.2, v_rel=-23.0, trackState=2)
  model = model_with_lead(distance - distance_error, 0.0, 19.0, probability=0.65)
  model.leadsV3[0].xStd = (x_std,)
  model.leadsV3[0].vStd = (5.0,)
  return point, model


@pytest.mark.parametrize("mode", (2, 3))
@pytest.mark.parametrize("corner", ("absent", "paired", "one_frame_moving"))
def test_far_stopped_front_uses_repeated_position_evidence_on_gentle_turn(mode, corner):
  controller = DPathRadarController(enable_radar_tracks=mode, prefer_corner_radar=True, cut_in_sensitivity=0)
  for index in range(18):
    time_s = index * 0.05
    point, model = approach_frame(time_s)
    points = [point]
    if corner == "paired":
      points.append(replace(point, track_id=2004, source="corner235", d_rel=point.d_rel + 0.3))
    elif corner == "one_frame_moving":
      # A long-lived corner ID gives one contradictory sample near vision.
      # Its prior location and subsequent jump do not identify a moving car.
      points.append(Point(2004, point.d_rel, 0.0 if index == 13 else 5.0,
                          v_rel=-5.0, source="corner235"))
    output = controller.update(time_s, 24.0, points, model, yaw_rate_rad_s=0.04)
    if index < 10:
      assert controller.primary_matcher.stationary_identity is None
    if index >= 16:
      assert output.lead_one["radarTrackId"] == 53
      assert output.lead_one["radar"]
      assert output.lead_one["dRel"] > 100.0
      assert output.lead_one["vLead"] == pytest.approx(1.0)
  assert controller.primary_matcher.stationary_corner_supported == (corner == "paired")


@pytest.mark.parametrize("mode", (2, 3))
@pytest.mark.parametrize("change", (
  "no_vision", "one_anchor", "range_offset", "lateral_offset", "low_quality",
  "unmeasured", "new_ids", "position_jumps", "observation_gaps", "tight_turn",
))
def test_far_stopped_front_cannot_borrow_weak_or_discontinuous_position_evidence(mode, change):
  controller = DPathRadarController(enable_radar_tracks=mode, cut_in_sensitivity=0)
  for index in range(30):
    time_s = index * (0.2 if change == "observation_gaps" else 0.05)
    dx = 12.0 if change == "range_offset" or (change == "one_anchor" and index != 10) else 1.0
    point, model = approach_frame(time_s, distance_error=dx)
    if change == "no_vision":
      model.leadsV3[0].prob = 0.0
    elif change == "lateral_offset":
      model.leadsV3[0].y = (-2.0,)
    elif change == "low_quality":
      point = replace(point, trackState=1)
    elif change == "unmeasured":
      point = replace(point, measured=False)
    elif change == "new_ids":
      point = replace(point, track_id=53 + index // 3)
    elif change == "position_jumps":
      point = replace(point, d_rel=point.d_rel + (4.0 if index % 2 else -4.0))
      model.leadsV3[0].x = (point.d_rel + 1.52,)
    output = controller.update(time_s, 24.0, (point,), model,
                               yaw_rate_rad_s=0.11 if change == "tight_turn" else 0.04)
    assert controller.primary_matcher.stationary_identity is None
    assert output.lead_one is None or not output.lead_one["radar"]


@pytest.mark.parametrize("mode", (2, 3))
@pytest.mark.parametrize("source", ("frontRadar", "corner235"))
def test_real_moving_evidence_overrides_stationary_position_anchors(mode, source):
  controller = DPathRadarController(enable_radar_tracks=mode, prefer_corner_radar=True, cut_in_sensitivity=0)
  for index in range(32):
    time_s = index * 0.05
    point, model = approach_frame(time_s)
    # A physically consistent moving return becomes the visual target.
    moving_distance = 109.0 - 5.0 * time_s
    points = [point, Point(2004, moving_distance, 0.0, v_rel=-5.0, source=source, trackState=2)]
    if index < 13:
      points[1] = replace(points[1], y_rel=5.0)
    else:
      model.leadsV3[0].x = (moving_distance + 1.52,)
    output = controller.update(time_s, 24.0, points, model, yaw_rate_rad_s=0.04)
    if index >= (13 if source == "frontRadar" else 18):
      assert controller.primary_matcher.stationary_identity != ("frontRadar", 53)
      assert controller.primary_matcher._stationary_pending_identity != ("frontRadar", 53)
      assert output.lead_one is None or output.lead_one["radarTrackId"] != 53


@pytest.mark.parametrize("mode", (2, 3))
@pytest.mark.parametrize("v_std", (2.0, 0.0, -5.0, float("nan"), float("inf")))
def test_tight_position_crossing_cannot_override_precise_or_invalid_moving_speed(mode, v_std):
  controller = DPathRadarController(enable_radar_tracks=mode, cut_in_sensitivity=0)
  for index in range(30):
    point, model = approach_frame(index * 0.05)
    model.leadsV3[0].vStd = (v_std,)
    output = controller.update(index * 0.05, 24.0, (point,), model, yaw_rate_rad_s=0.04)
    assert controller.primary_matcher.stationary_identity is None
    assert output.lead_one is None or output.lead_one["radarTrackId"] != 53


def seed_stationary(controller):
  for index in range(20):
    time_s = index * 0.05
    point, model = approach_frame(time_s)
    model.leadsV3[0].v = (0.0,)
    output = controller.update(time_s, 24.0, (point,), model, yaw_rate_rad_s=0.04)
  assert output.lead_one["radarTrackId"] == 53


@pytest.mark.parametrize("mode", (2, 3))
def test_confirmed_stopped_front_bridges_bounded_uncertain_range_then_releases(mode):
  controller = DPathRadarController(enable_radar_tracks=mode, cut_in_sensitivity=0)
  seed_stationary(controller)
  for index in range(20, 33):
    time_s = index * 0.05
    point, model = approach_frame(time_s, distance_error=14.0)
    output = controller.update(time_s, 24.0, (point,), model, yaw_rate_rad_s=0.04)
    if index <= 28:
      assert output.lead_one["radarTrackId"] == 53
      assert output.lead_one["dRel"] == pytest.approx(point.d_rel)
      assert output.lead_one["vLead"] == pytest.approx(1.0)
    if index >= 30:
      assert output.lead_one["radarTrackId"] != 53


@pytest.mark.parametrize("mode", (2, 3))
@pytest.mark.parametrize("change", (
  "precise_range", "farther_object", "close_visual_car", "position_jump", "lateral_jump",
  "speed_jump", "unmeasured", "gap", "missed_match", "invalid_path", "legacy_mode",
))
def test_stationary_range_hold_cannot_cover_a_new_or_contradictory_object(mode, change):
  controller = DPathRadarController(enable_radar_tracks=mode, cut_in_sensitivity=0)
  seed_stationary(controller)
  time_s = 1.0
  point, model = approach_frame(time_s, distance_error=14.0, x_std=2.0 if change == "precise_range" else 20.0)
  if change == "farther_object":
    model.leadsV3[0].x = (point.d_rel - 30.0 + 1.52,)
  elif change == "close_visual_car":
    point = replace(point, d_rel=40.0)
    model.leadsV3[0].x = (30.0 + 1.52,)
  elif change == "position_jump":
    point = replace(point, d_rel=point.d_rel + 4.0)
  elif change == "lateral_jump":
    point = replace(point, y_rel=-1.4)
  elif change == "speed_jump":
    point = replace(point, v_rel=-20.9)
  elif change == "unmeasured":
    point = replace(point, measured=False)
  elif change == "gap":
    time_s = 1.2
  elif change in ("missed_match", "invalid_path", "legacy_mode"):
    empty = model_with_lead(90.0, 0.0, 19.0)
    if change == "invalid_path":
      empty.position.x = ()
      empty.position.y = ()
    if change == "legacy_mode":
      controller.enable_radar_tracks = -1
    controller.update(1.0, 24.0, (), empty)
    controller.enable_radar_tracks = mode
    time_s = 1.05
  output = controller.update(time_s, 24.0, (point,), model, yaw_rate_rad_s=0.04)
  assert output.lead_one is None or output.lead_one["radarTrackId"] != 53
