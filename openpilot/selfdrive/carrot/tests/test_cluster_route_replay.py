from dataclasses import replace
from pathlib import Path
from types import SimpleNamespace
import sys


CLUSTER_DIR = Path(__file__).resolve().parents[1] / "cluster"
sys.path.insert(0, str(CLUSTER_DIR))

from cluster_live import OpenpilotLiveSource
from cluster_models import ClusterAlert, ModelPathPoint
from cluster_route_replay import (
  RawCornerObject,
  RouteLogParser,
  StableCornerObjectTracker,
  adjacent_route_log_path,
  blend_frames,
  car_state_corner_detections,
  corner_track_label,
  frame_to_state,
  model_lead_detections_from_model_v2,
)


def corner_object(t, slot, object_id, age, x, y, vx, vy):
  return RawCornerObject(t, "180", 0x180, slot, 80, age, object_id, 3, 2.0, x, y, vx, vy, 0.0)


def live_track(track_id, d_rel, y_rel, v_rel):
  return SimpleNamespace(
    trackId=track_id,
    radarSource="corner180",
    dRel=d_rel,
    yRel=y_rel,
    vRel=v_rel,
    measured=True,
  )


def radar_lead(track_id, d_rel=25.0, y_rel=2.0):
  return SimpleNamespace(
    status=True,
    radar=True,
    radarTrackId=track_id,
    dRel=d_rel,
    yRel=y_rel,
    vRel=-1.0,
    vLead=15.0,
    vLat=0.0,
    aLeadK=0.0,
  )


def test_blindspot_flags_do_not_create_virtual_corner_vehicles():
  detections = car_state_corner_detections(SimpleNamespace(
    leftBlindspot=True,
    rightBlindspot=True,
  ))

  assert detections == ()


def test_blindspot_rear_distance_still_creates_measured_corner_vehicle():
  detections = car_state_corner_detections(SimpleNamespace(
    leftBlindspot=True,
    rightBlindspot=False,
    leftRearLongDist=3.2,
    leftRearLatDist=2.4,
  ))

  assert len(detections) == 1
  assert detections[0].label == "LR"
  assert detections[0].longitudinal_m == -3.2
  assert detections[0].lateral_m == -2.4
  assert detections[0].source == "carState"


def test_corner_430_track_labels_preserve_radar_group():
  assert corner_track_label(300, "corner430") == "CR430_000"
  assert corner_track_label(411, "corner430") == "CR430_111"
  assert corner_track_label(1204, "corner430") == "CR430_T1204"


def test_replay_uses_raw_object_identity_when_corner_slot_is_reused():
  parser = RouteLogParser()
  old_object = corner_object(13.969, 2, 108, 255, 3.6, -4.65, -4.10, -0.20)
  parser.raw_corner_objects[("180", 2)] = old_object
  old_track_id = next(iter(parser._cutin_points_by_stable_id(
    (live_track(242, 3.6, -4.65, -4.10),), 13.969
  )))

  new_object = corner_object(13.999, 1, 46, 2, 4.0, -4.30, -4.95, 0.55)
  parser.raw_corner_objects = {("180", 1): new_object}
  new_track_id = next(iter(parser._cutin_points_by_stable_id(
    (live_track(241, 4.0, -4.30, -4.95),), 13.999
  )))

  assert old_track_id != new_track_id

  moved_object = corner_object(14.028, 2, 46, 3, 3.9, -4.25, -4.95, 0.55)
  parser.raw_corner_objects = {("180", 2): moved_object}
  moved_track_id = next(iter(parser._cutin_points_by_stable_id(
    (live_track(242, 3.9, -4.25, -4.95),), 14.028
  )))

  assert moved_track_id == new_track_id


def test_reconstructed_corner_tracks_include_filtered_lead_dynamics():
  tracker = StableCornerObjectTracker()
  output = ()
  for index in range(14):
    t = index * 0.05
    vx = index * 0.08
    tracker.update(corner_object(t, 1, 46, index + 1, 12.0 + vx * t, 2.0, vx, 0.0))
    output = tracker.live_tracks_at(t, 15.0)

  assert len(output) == 1
  assert output[0].aLead > 0.05
  assert output[0].jLead != 0.0


def test_reconstructed_live_tracks_can_reject_stale_measurements():
  tracker = StableCornerObjectTracker()
  for index in range(4):
    tracker.update(corner_object(
      1.0 + index * 0.02,
      1,
      46,
      index + 1,
      12.0,
      2.0,
      1.0,
      -1.0,
    ))

  assert tracker.live_tracks_at(1.08, 15.0, max_measurement_age_s=0.10)
  assert not tracker.live_tracks_at(
    1.20, 15.0, max_measurement_age_s=0.10,
  )


def test_recorded_cutin_display_requires_current_leads_cutin_membership():
  parser = RouteLogParser(recompute_cutins=False)
  corner_lead = radar_lead(2540)

  parser._update_radar_state(SimpleNamespace(
    leadOne=radar_lead(62, d_rel=60.0, y_rel=0.0),
    leadTwo=corner_lead,
    leadsCutIn=[],
  ), 1.0)

  lead_two = next(d for d in parser.radar_detections if d.radar_track_id == 2540)
  assert lead_two.label == "L2"
  assert not lead_two.cut_in

  parser._update_radar_state(SimpleNamespace(
    leadOne=radar_lead(62, d_rel=60.0, y_rel=0.0),
    leadTwo=corner_lead,
    leadsCutIn=[corner_lead],
  ), 1.05)

  lead_two = next(d for d in parser.radar_detections if d.radar_track_id == 2540)
  assert lead_two.label == "L2 CUT-IN"
  assert lead_two.cut_in
  assert parser.recorded_cutin_ids == {2540}
  assert parser.cutin_debug_text.startswith("LOG CUT-IN: YES | id2540")


def test_route_playback_does_not_run_offline_cutin_recomputation():
  parser = RouteLogParser(recompute_cutins=False)

  def fail_if_called(*_args):
    raise AssertionError("offline cut-in recomputation must be disabled for route playback")

  parser._update_offline_cutin = fail_if_called
  parser._update_live_tracks(SimpleNamespace(points=()), 1.0)


def test_recorded_cutin_remains_visible_in_front_radar_only_mode():
  parser = RouteLogParser(recompute_cutins=False)
  parser.front_radar_only = True
  corner_lead = radar_lead(2540)

  parser._update_radar_state(SimpleNamespace(
    leadOne=radar_lead(62, d_rel=60.0, y_rel=0.0),
    leadTwo=corner_lead,
    leadsCutIn=[corner_lead],
  ), 1.0)

  recorded = next(d for d in parser.radar_detections if d.radar_track_id == 2540)
  assert recorded.label == "L2 CUT-IN"
  assert recorded.cut_in


def test_recorded_cutin_prompt_is_timestamped_for_replay_alert():
  parser = RouteLogParser(recompute_cutins=False)
  corner_lead = radar_lead(2540)
  parser._update_radar_state(SimpleNamespace(
    leadOne=radar_lead(62, d_rel=60.0, y_rel=0.0),
    leadTwo=corner_lead,
    leadsCutIn=[corner_lead],
  ), 1.0)

  parser._update_selfdrive_state(SimpleNamespace(
    alertSound="prompt",
    alertType="audioPrompt/warning",
  ), 1.01)

  assert parser.recorded_cutin_sound
  assert parser.recorded_cutin_sound_t == 1.01
  frame = parser._frame_from_car_state(SimpleNamespace(), 1.07)
  assert frame.recorded_cutin_active
  assert frame.recorded_cutin_sound
  assert any(detection.cut_in for detection in frame.detected_vehicles)


def test_dedicated_cutin_sound_is_timestamped_for_replay_alert():
  parser = RouteLogParser(recompute_cutins=False)
  corner_lead = radar_lead(2540)
  parser._update_radar_state(SimpleNamespace(
    leadOne=radar_lead(62, d_rel=60.0, y_rel=0.0),
    leadTwo=corner_lead,
    leadsCutIn=[corner_lead],
  ), 1.0)

  parser._update_selfdrive_state(SimpleNamespace(
    alertSound="radarCutin",
    alertType="radarCutin/warning",
  ), 1.01)

  assert parser.recorded_cutin_sound
  assert parser.recorded_cutin_sound_t == 1.01


def test_live_selfdrive_state_passes_event_timestamp():
  calls = []
  source = object.__new__(OpenpilotLiveSource)
  source.sm = {"selfdriveState": SimpleNamespace(enabled=True)}
  source.parser = SimpleNamespace(
    _update_selfdrive_state=lambda data, event_t: calls.append((data, event_t)),
  )

  source._apply_service_update("selfdriveState", 12.5)

  assert calls == [(source.sm["selfdriveState"], 12.5)]


def test_selfdrive_alert_reaches_cluster_state_and_clears() -> None:
  parser = RouteLogParser()
  parser._update_selfdrive_state(SimpleNamespace(
    enabled=True,
    alertText1="TAKE CONTROL IMMEDIATELY",
    alertText2="Camera Malfunction",
    alertSize=SimpleNamespace(raw=3),
    alertStatus=SimpleNamespace(raw=2),
    alertType="cameraMalfunction/immediateDisable",
    alertSound="warningImmediate",
  ), 1.0)

  frame = parser._frame_from_car_state(SimpleNamespace(), 1.01)
  assert frame.alert == ClusterAlert(
    text1="TAKE CONTROL IMMEDIATELY",
    text2="Camera Malfunction",
    size=3,
    status=2,
    alert_type="cameraMalfunction/immediateDisable",
  )
  assert frame_to_state(frame).alert == frame.alert

  parser._update_selfdrive_state(SimpleNamespace(alertSize="none"), 1.02)
  assert parser._frame_from_car_state(SimpleNamespace(), 1.03).alert is None


def test_selfdrive_alert_is_discrete_during_replay_interpolation() -> None:
  parser = RouteLogParser()
  base = parser._frame_from_car_state(SimpleNamespace(), 0.0)
  alert = ClusterAlert("Steering Unavailable", "Take Control", size=2, status=1)
  alerted = replace(base, t=0.0, alert=alert)
  cleared = replace(base, t=1.0, alert=None)

  assert blend_frames(alerted, cleared, 0.49).alert == alert
  assert blend_frames(alerted, cleared, 0.50).alert is None


def test_live_longitudinal_plan_passes_service_validity():
  calls = []
  source = object.__new__(OpenpilotLiveSource)
  source.sm = {"longitudinalPlan": SimpleNamespace(myDrivingMode=2)}
  source.parser = SimpleNamespace(
    _update_longitudinal_plan=lambda data, valid: calls.append((data, valid)),
  )
  source._service_valid = lambda service: service != "longitudinalPlan"

  source._apply_service_update("longitudinalPlan", 12.5)

  assert calls == [(source.sm["longitudinalPlan"], False)]


def test_controls_active_lane_line_reaches_cluster_state():
  parser = RouteLogParser()
  parser._update_controls_state(SimpleNamespace(activeLaneLine=True))

  frame = parser._frame_from_car_state(SimpleNamespace(), 1.0)
  state = frame_to_state(frame)

  assert frame.active_lane_line is True
  assert state.active_lane_line is True


def test_ev_mode_reaches_cluster_only_when_carstate_marks_it_valid():
  parser = RouteLogParser()

  active = parser._frame_from_car_state(SimpleNamespace(evModeValid=True, evModeActive=True), 1.0)
  engine = parser._frame_from_car_state(SimpleNamespace(evModeValid=True, evModeActive=False), 2.0)
  invalid = parser._frame_from_car_state(SimpleNamespace(evModeValid=False, evModeActive=True), 3.0)
  unsupported = parser._frame_from_car_state(SimpleNamespace(), 4.0)

  assert (active.ev_mode_valid, active.ev_mode_active) == (True, True)
  assert (engine.ev_mode_valid, engine.ev_mode_active) == (True, False)
  assert (invalid.ev_mode_valid, invalid.ev_mode_active) == (False, False)
  assert (unsupported.ev_mode_valid, unsupported.ev_mode_active) == (False, False)
  assert (frame_to_state(active).ev_mode_valid, frame_to_state(active).ev_mode_active) == (True, True)
  assert (frame_to_state(invalid).ev_mode_valid, frame_to_state(invalid).ev_mode_active) == (False, False)


def test_ev_mode_is_preserved_as_discrete_state_during_replay_interpolation():
  parser = RouteLogParser()
  base = parser._frame_from_car_state(SimpleNamespace(), 0.0)
  active = replace(base, t=0.0, ev_mode_valid=True, ev_mode_active=True)
  engine = replace(base, t=1.0, ev_mode_valid=True, ev_mode_active=False)

  assert blend_frames(active, engine, 0.49).ev_mode_active is True
  assert blend_frames(active, engine, 0.50).ev_mode_active is False


def test_driving_mode_reaches_cluster_only_for_known_values():
  parser = RouteLogParser()

  for mode in (1, 2, 3, 4):
    parser._update_longitudinal_plan(SimpleNamespace(myDrivingMode=mode))
    frame = parser._frame_from_car_state(SimpleNamespace(), float(mode))
    assert frame.driving_mode == mode
    assert frame_to_state(frame).driving_mode == mode

  for invalid_mode in (0, -1, 5, None):
    parser._update_longitudinal_plan(SimpleNamespace(myDrivingMode=invalid_mode))
    frame = parser._frame_from_car_state(SimpleNamespace(), 10.0)
    assert frame.driving_mode is None
    assert frame_to_state(frame).driving_mode is None

  parser._update_longitudinal_plan(SimpleNamespace())
  assert parser._frame_from_car_state(SimpleNamespace(), 11.0).driving_mode is None

  parser._update_longitudinal_plan(SimpleNamespace(myDrivingMode=2), valid=False)
  assert parser._frame_from_car_state(SimpleNamespace(), 12.0).driving_mode is None


def test_driving_mode_is_held_between_plan_updates_and_blended_discretely():
  parser = RouteLogParser()
  parser._update_longitudinal_plan(SimpleNamespace(myDrivingMode=1))
  eco = parser._frame_from_car_state(SimpleNamespace(), 0.0)
  held = parser._frame_from_car_state(SimpleNamespace(), 0.01)
  high = replace(eco, t=1.0, driving_mode=4)

  assert held.driving_mode == 1
  assert blend_frames(eco, high, 0.49).driving_mode == 1
  assert blend_frames(eco, high, 0.50).driving_mode == 4


def test_lane_change_animation_keeps_target_floor_and_lane_grid_without_blinker():
  parser = RouteLogParser()
  base = parser._frame_from_car_state(SimpleNamespace(), 1.0)
  lane_lines = tuple(
    (
      ModelPathPoint(0.0, lateral_m),
      ModelPathPoint(30.0, lateral_m),
    )
    for lateral_m in (-5.4, -1.8, 1.8, 5.4)
  )
  frame = replace(
    base,
    lane_change="left",
    lane_change_phase="changing",
    lane_change_progress=0.5,
    left_signal=False,
    right_signal=False,
    left_lane_visible=False,
    right_lane_visible=False,
    extra_left_lane_visible=False,
    extra_right_lane_visible=False,
    left_road_edge_offset=-2.0,
    right_road_edge_offset=2.0,
    model_lane_lines=lane_lines,
  )

  state = frame_to_state(frame)

  assert state.highlight_lane == "left"
  assert state.highlight_lane_offset == -1.0
  assert state.ego_lane_offset == -0.5
  assert [lane.offset for lane in state.lanes] == [-1.5, -0.5, 0.5, 1.5]
  assert [lane.visible for lane in state.lanes] == [True, True, True, False]


def model_lead(probability, distance=50.0):
  return SimpleNamespace(
    prob=probability,
    x=(distance,),
    y=(0.0,),
    v=(15.0,),
    a=(0.0,),
    xStd=(1.0,),
    yStd=(0.5,),
  )


def test_model_lead_display_hides_low_probability_candidates():
  model = SimpleNamespace(
    leadsV3=(model_lead(0.49), model_lead(0.50, 60.0)),
    velocity=SimpleNamespace(x=(20.0,)),
  )

  detections = model_lead_detections_from_model_v2(model)

  assert [d.label for d in detections] == ["M2"]
  assert detections[0].probability == 0.50


def test_live_calibration_height_is_not_overwritten_by_camera_odometry():
  parser = RouteLogParser()
  camera_odometry = SimpleNamespace(
    trans=(0.0, 0.0, 0.0),
    rot=(0.0, 0.0, 0.0),
    transStd=(0.0, 0.0, 0.0),
    rotStd=(0.0, 0.0, 0.0),
    wideFromDeviceEuler=(0.0, 0.02, 0.0),
    roadTransformTrans=(0.0, 0.0, 1.36),
    roadTransformTransStd=(0.0, 0.0, 0.02),
  )
  parser._update_camera_odometry(camera_odometry, True)
  assert parser.road_transform_trans == (0.0, 0.0, 1.36)

  parser._update_live_calibration(SimpleNamespace(
    rpyCalib=(0.0, 0.02, 0.0),
    height=(1.418,),
  ), True)
  parser._update_camera_odometry(SimpleNamespace(
    **{**camera_odometry.__dict__, "roadTransformTrans": (0.0, 0.0, 1.35)},
  ), True)

  assert parser.road_transform_trans == (0.0, 0.0, 1.418)


def test_adjacent_route_log_path_preserves_number_padding(tmp_path):
  previous_folder = tmp_path / "route--004"
  current_folder = tmp_path / "route--005"
  next_folder = tmp_path / "route--006"
  for folder in (previous_folder, current_folder, next_folder):
    folder.mkdir()
    (folder / "rlog.zst").write_bytes(b"")

  current_log = current_folder / "rlog.zst"
  assert adjacent_route_log_path(current_log, -1) == previous_folder / "rlog.zst"
  assert adjacent_route_log_path(current_log, 1) == next_folder / "rlog.zst"


def test_adjacent_route_log_path_returns_none_for_missing_number(tmp_path):
  folder = tmp_path / "route-without-segment"
  folder.mkdir()
  log_path = folder / "rlog.zst"
  log_path.write_bytes(b"")

  assert adjacent_route_log_path(log_path, 1) is None
