from pathlib import Path
from types import SimpleNamespace
import sys


CLUSTER_DIR = Path(__file__).resolve().parents[1] / "cluster"
sys.path.insert(0, str(CLUSTER_DIR))

from cluster_live import OpenpilotLiveSource
from cluster_route_replay import RawCornerObject, RouteLogParser, adjacent_route_log_path


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


def test_live_selfdrive_state_passes_event_timestamp():
  calls = []
  source = object.__new__(OpenpilotLiveSource)
  source.sm = {"selfdriveState": SimpleNamespace(enabled=True)}
  source.parser = SimpleNamespace(
    _update_selfdrive_state=lambda data, event_t: calls.append((data, event_t)),
  )

  source._apply_service_update("selfdriveState", 12.5)

  assert calls == [(source.sm["selfdriveState"], 12.5)]


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
