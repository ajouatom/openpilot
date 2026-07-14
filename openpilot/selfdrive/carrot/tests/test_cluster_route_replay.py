from pathlib import Path
from types import SimpleNamespace
import sys


CLUSTER_DIR = Path(__file__).resolve().parents[1] / "cluster"
sys.path.insert(0, str(CLUSTER_DIR))

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
