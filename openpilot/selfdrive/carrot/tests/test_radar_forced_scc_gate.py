"""EnableRadarTracks 3: the unconditional SCC fallback refuses a body that front/corner radar put beside the path.

Geometry follows field logs: 2026-09-24 adjacent-lane queue (front y -2.4, corner y -2.9),
2026-10-06 car left behind by a lane change, 2026-09-18 stopped truck in the lane being left,
2026-09-18 in-lane car whose close-range radar lateral read 1.3-2.0 m.
"""
from dataclasses import dataclass
from types import SimpleNamespace

import pytest

from openpilot.selfdrive.carrot.radar_motion.controller import (
  FORCED_SCC_REJECT_HOLD_S,
  DPathRadarController,
)


@dataclass(frozen=True)
class Point:
  track_id: int
  d_rel: float
  y_rel: float
  v_rel: float = 0.0
  a_rel: float = 0.0
  measured: bool = True
  source: str = "frontRadar"
  v_lead: float | None = None
  a_lead: float | None = None
  yv_rel: float = 0.0
  j_lead: float = 0.0
  trackState: int = 0


def model(lead_d: float, lead_y: float, lead_v: float, probability: float,
          lane_change_state: str = "off") -> SimpleNamespace:
  return SimpleNamespace(
    position=SimpleNamespace(x=(0.0, 100.0), y=(0.0, 0.0)),
    leadsV3=(SimpleNamespace(
      prob=probability, x=(lead_d + 1.52,), y=(-lead_y,), v=(lead_v,),
      xStd=(2.0,), yStd=(0.6,), vStd=(1.5,),
    ),),
    meta=SimpleNamespace(laneChangeState=lane_change_state),
  )


def body(d_rel: float, front_y: float | None, corner_y: float | None, v_rel: float) -> tuple[Point, ...]:
  points = [Point(0, d_rel, 0.0, v_rel=v_rel, source="scc")]
  if front_y is not None:
    points.append(Point(38, d_rel - 0.3, front_y, v_rel=v_rel))
  if corner_y is not None:
    points.append(Point(6762, d_rel + 0.1, corner_y, v_rel=v_rel, source="corner235"))
  return tuple(points)


def is_forced_scc(lead: dict | None) -> bool:
  return lead is not None and lead["radar"] and lead["radarTrackId"] == 0


def test_beside_path_queue_is_not_adopted() -> None:
  output = DPathRadarController(enable_radar_tracks=3).update(
    time_s=1.0, v_ego=8.6, radar_points=body(16.5, -2.4, -2.9, v_rel=-1.3),
    model=model(60.0, 0.1, 9.0, probability=0.15),
  )
  assert not is_forced_scc(output.lead_one)


def test_car_left_behind_by_lane_change_is_not_adopted() -> None:
  # Moving car in the old lane: the slow-object lane change exception does not apply.
  output = DPathRadarController(enable_radar_tracks=3).update(
    time_s=1.0, v_ego=14.6, radar_points=body(12.0, 3.4, 3.4, v_rel=-3.0),
    model=model(50.0, -7.0, 14.0, probability=0.1, lane_change_state="laneChangeStarting"),
  )
  assert not is_forced_scc(output.lead_one)


def test_stopped_car_in_lane_being_left_is_kept() -> None:
  output = DPathRadarController(enable_radar_tracks=3).update(
    time_s=1.0, v_ego=16.4, radar_points=body(31.0, -2.5, -3.0, v_rel=-16.8),
    model=model(60.0, 0.2, 16.0, probability=0.7, lane_change_state="laneChangeStarting"),
  )
  assert is_forced_scc(output.lead_one)


def test_close_in_lane_car_with_wide_radar_lateral_is_kept() -> None:
  output = DPathRadarController(enable_radar_tracks=3).update(
    time_s=1.0, v_ego=10.5, radar_points=body(7.5, 1.6, 1.9, v_rel=-2.0),
    model=model(24.0, 0.5, 9.0, probability=0.98),
  )
  assert is_forced_scc(output.lead_one)


def test_vision_at_same_distance_keeps_scc() -> None:
  output = DPathRadarController(enable_radar_tracks=3).update(
    time_s=1.0, v_ego=12.0, radar_points=body(20.0, -2.6, -2.9, v_rel=-2.0),
    model=model(20.0, 0.0, 10.0, probability=0.9),
  )
  assert output.lead_one is not None and output.lead_one["radar"]


def test_scc_without_physical_match_is_still_adopted() -> None:
  output = DPathRadarController(enable_radar_tracks=3).update(
    time_s=1.0, v_ego=10.0, radar_points=body(30.0, None, None, v_rel=-2.0),
    model=model(60.0, 0.0, 9.0, probability=0.1),
  )
  assert is_forced_scc(output.lead_one)


def test_refusal_holds_through_support_dropout_then_expires() -> None:
  controller = DPathRadarController(enable_radar_tracks=3)
  far_vision = model(60.0, 0.1, 9.0, probability=0.15)
  first = controller.update(time_s=1.0, v_ego=8.6, radar_points=body(16.5, -2.4, -2.9, v_rel=-1.3), model=far_vision)
  assert not is_forced_scc(first.lead_one)
  dropout = controller.update(time_s=1.5, v_ego=8.6, radar_points=body(16.0, None, None, v_rel=-1.3), model=far_vision)
  assert not is_forced_scc(dropout.lead_one)
  expired = controller.update(time_s=1.5 + FORCED_SCC_REJECT_HOLD_S + 0.2, v_ego=8.6,
                              radar_points=body(15.0, None, None, v_rel=-1.3), model=far_vision)
  assert is_forced_scc(expired.lead_one)


@pytest.mark.parametrize("mode", (1, 2))
def test_other_modes_never_take_the_unconditional_path(mode: int) -> None:
  controller = DPathRadarController(enable_radar_tracks=mode)
  controller.update(time_s=1.0, v_ego=8.6, radar_points=body(16.5, -2.4, -2.9, v_rel=-1.3),
                    model=model(60.0, 0.1, 9.0, probability=0.15))
  assert controller.forced_scc_rejected_s is None
