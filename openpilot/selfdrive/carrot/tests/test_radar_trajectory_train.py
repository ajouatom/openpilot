from types import SimpleNamespace

from openpilot.selfdrive.carrot.radar.tools.radar_lead_simulator import RadarPoint
from openpilot.selfdrive.carrot.radar.tools.radar_trajectory_train import (
  _future_targets,
  _load_log_rows,
  _save_log_rows,
  _track_observations,
)
from openpilot.selfdrive.carrot.radar.tools.radar_trajectory_compare import (
  ComparisonRow,
  _summary,
)


PATH = ((0.0, 0.0), (100.0, 0.0))
LANE_LINES = (
  ((0.0, 3.6), (100.0, 3.6)),
  ((0.0, 1.8), (100.0, 1.8)),
  ((0.0, -1.8), (100.0, -1.8)),
  ((0.0, -3.6), (100.0, -3.6)),
)
LANE_PROBS = (0.8, 0.95, 0.95, 0.8)


def point(
  d_rel: float,
  y_rel: float,
  *,
  measured: bool = True,
  v_rel: float = 0.0,
  yv_rel: float = 0.0,
) -> RadarPoint:
  return RadarPoint(
    track_id=10,
    d_rel=d_rel,
    y_rel=y_rel,
    v_rel=v_rel,
    a_rel=0.0,
    yv_rel=yv_rel,
    v_lead=20.0 + v_rel,
    measured=measured,
    source="frontRadar",
  )


def frame(time_s: float, points: tuple[RadarPoint, ...]) -> SimpleNamespace:
  return SimpleNamespace(
    mono_time_s=time_s,
    time_s=time_s,
    points=points,
    path=PATH,
    lane_lines=LANE_LINES,
    lane_probs=LANE_PROBS,
  )


def test_future_target_uses_measured_same_episode_position() -> None:
  frames = (
    frame(0.0, (point(25.0, 4.0),)),
    frame(0.25, (point(25.0, 3.2),)),
    frame(0.5, (point(25.0, 2.5),)),
    frame(0.75, (point(25.0, 1.8),)),
    frame(1.0, (point(25.0, 1.0),)),
  )
  by_frame, episodes = _track_observations(frames)
  current = by_frame[(0, "frontRadar", 10)]
  labels, valid = _future_targets(
    frames,
    current,
    episodes[("frontRadar", 10, current.episode_id)],
  )

  assert valid[:2] == [True, True]
  assert labels[:2] == [1.0, 1.0]
  assert valid[2:] == [False, False]


def test_unmeasured_future_slot_is_not_ground_truth() -> None:
  frames = (
    frame(0.0, (point(25.0, 4.0),)),
    frame(0.5, (point(0.0, 0.0, measured=False),)),
  )
  by_frame, episodes = _track_observations(frames)
  current = by_frame[(0, "frontRadar", 10)]
  labels, valid = _future_targets(
    frames,
    current,
    episodes[("frontRadar", 10, current.episode_id)],
  )

  assert not valid[0]
  assert labels[0] == 0.0


def test_currently_occupied_vehicle_keeps_unconditional_future_occupancy_target() -> None:
  frames = (
    frame(0.0, (point(25.0, 0.2),)),
    frame(0.25, (point(25.0, 0.3),)),
    frame(0.5, (point(25.0, 0.4),)),
  )
  by_frame, episodes = _track_observations(frames)
  current = by_frame[(0, "frontRadar", 10)]
  labels, valid = _future_targets(
    frames,
    current,
    episodes[("frontRadar", 10, current.episode_id)],
  )

  assert valid[0]
  assert labels[0] == 1.0


def test_future_path_exit_is_a_clear_occupancy_target() -> None:
  frames = (
    frame(0.0, (point(25.0, 0.2, yv_rel=5.0),)),
    frame(0.25, (point(25.0, 1.45, yv_rel=5.0),)),
    frame(0.5, (point(25.0, 2.8, yv_rel=5.0),)),
  )
  by_frame, episodes = _track_observations(frames)
  current = by_frame[(0, "frontRadar", 10)]
  labels, valid = _future_targets(
    frames,
    current,
    episodes[("frontRadar", 10, current.episode_id)],
  )

  assert valid[0]
  assert labels[0] == 0.0


def test_reused_id_cannot_supply_future_ground_truth() -> None:
  frames = (
    frame(0.0, (point(25.0, 4.0),)),
    frame(0.25, (point(25.0, 3.8),)),
    frame(0.5, (point(80.0, -5.0),)),
  )
  by_frame, episodes = _track_observations(frames)
  current = by_frame[(0, "frontRadar", 10)]
  reused = by_frame[(2, "frontRadar", 10)]
  _, valid = _future_targets(
    frames,
    current,
    episodes[("frontRadar", 10, current.episode_id)],
  )

  assert reused.episode_id != current.episode_id
  assert not valid[0]


def test_brief_physically_continuous_dropout_keeps_episode() -> None:
  frames = (
    frame(0.0, (point(25.0, 4.0, v_rel=-1.0),)),
    frame(0.1, ()),
    frame(0.2, (point(24.8, 3.9, v_rel=-1.0),)),
  )
  by_frame, _ = _track_observations(frames)

  assert by_frame[(0, "frontRadar", 10)].episode_id == by_frame[(2, "frontRadar", 10)].episode_id


def test_incremental_log_cache_round_trip_and_fingerprint_guard(tmp_path) -> None:
  cache_path = tmp_path / "log.npz"
  fingerprint = {"size": 123, "mtime_ns": 456}
  rows = {
    "front": [([1.0, 2.0], [1.0, 0.0, 0.0, 0.0], [True, True, False, False], False, "sample", "log")],
  }

  _save_log_rows(cache_path, "vehicle/log/rlog.zst", fingerprint, rows)

  assert _load_log_rows(cache_path, "vehicle/log/rlog.zst", fingerprint) == rows
  assert _load_log_rows(cache_path, "vehicle/log/rlog.zst", {"size": 124, "mtime_ns": 456}) is None


def test_carrot_wip_comparison_summary_is_source_separated() -> None:
  rows = (
    ComparisonRow("f-tp", "front", "detect", "detect", True, 0.9, 3),
    ComparisonRow("f-fp", "front", "clear", "detect", True, 0.8, 2),
    ComparisonRow("c-tn", "corner", "clear", "clear", False, 0.1, 4),
  )

  values = _summary(rows)

  assert values["front"]["tp"] == 1
  assert values["front"]["fp"] == 1
  assert values["front"]["precision"] == 0.5
  assert values["corner"]["tn"] == 1
  assert values["corner"]["fp"] == 0
