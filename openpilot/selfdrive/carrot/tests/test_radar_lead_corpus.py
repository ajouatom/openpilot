from pathlib import Path

from openpilot.selfdrive.carrot.radar.tools.radar_lead_corpus import (
  LogProfile,
  route_group,
  scenario_tags,
  select_profiles,
  split_profiles,
)


def profile(log: str, vehicle: str = "CAR device", **values: object) -> LogProfile:
  defaults: dict[str, object] = {
    "log": log,
    "vehicle": vehicle,
    "route_group": f"{vehicle}/{Path(log).parent.name.rsplit('--', 1)[0]}",
    "size_bytes": 1,
    "duration_s": 60.0,
    "car_state_frames": 600,
    "radar_state_frames": 1200,
    "live_tracks_frames": 1200,
    "model_frames": 1200,
    "camera_frames": 1200,
    "mean_speed_kph": 40.0,
    "p90_speed_kph": 70.0,
    "stopped_fraction": 0.1,
    "congestion_fraction": 0.2,
    "highway_fraction": 0.0,
    "curve_fraction": 0.0,
    "mean_exposure": 10.0,
    "p90_exposure": 20.0,
    "dark_fraction": 0.0,
    "mean_radar_points": 10.0,
    "front_points": 100,
    "corner_points": 0,
    "scc_points": 0,
    "raw_corner_messages": 0,
    "side_frames": 0,
    "side_motion_frames": 0,
    "stationary_center_frames": 0,
    "stationary_side_frames": 0,
    "stationary_selected_frames": 0,
    "recorded_lead_one_frames": 1000,
    "recorded_lead_two_frames": 0,
    "recorded_cutin_frames": 0,
    "model_lead_frames": 1000,
    "has_qcamera": True,
  }
  defaults.update(values)
  return LogProfile(**defaults)


def test_route_group_keeps_adjacent_segments_together() -> None:
  first = Path("W:/routes/CAR device/route--hash--1/rlog.zst")
  second = Path("W:/routes/CAR device/route--hash--2/rlog.zst")
  assert route_group(first) == route_group(second) == "CAR device/route--hash"


def test_scenario_tags_include_stationary_positives_and_hard_negatives() -> None:
  item = profile(
    "W:/routes/CAR device/route--1/rlog.zst",
    stationary_selected_frames=8,
    stationary_center_frames=100,
    stationary_side_frames=120,
    p90_exposure=90.0,
  )
  tags = scenario_tags(item)
  assert "stationary-selected" in tags
  assert "stationary-center" in tags
  assert "stationary-side" in tags
  assert "night" in tags


def test_gold_logs_are_excluded_from_selection() -> None:
  gold = profile("W:/routes/CAR device/gold--1/rlog.zst")
  regular = profile("W:/routes/CAR device/train--1/rlog.zst")
  selected, held_out = select_profiles(
    [gold, regular], 2, 10, {"car device/gold--1/rlog.zst"}, 42,
  )
  assert selected == [regular]
  assert held_out == [gold]


def test_route_groups_never_cross_dataset_splits() -> None:
  profiles = [
    profile(f"W:/routes/CAR device/route{i // 2}--{i % 2}/rlog.zst", route_group=f"CAR device/route{i // 2}")
    for i in range(20)
  ]
  splits = split_profiles(profiles, 0.2, 0.2, 42)
  owners: dict[str, str] = {}
  for split, items in splits.items():
    for item in items:
      assert item.route_group not in owners or owners[item.route_group] == split
      owners[item.route_group] = split
