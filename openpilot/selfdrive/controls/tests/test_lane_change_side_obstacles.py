from types import SimpleNamespace

import pytest

from openpilot.common.realtime import DT_MDL
from openpilot.selfdrive.controls.lib.desire_lib.side_state import SideState


def lead(d_rel: float, v_rel: float, v_ego: float, d_path: float = 2.0, track_id: int = 1):
  return SimpleNamespace(
    status=True,
    dRel=d_rel,
    vRel=v_rel,
    vLead=v_ego + v_rel,
    dPath=d_path,
    radarTrackId=track_id,
  )


def settle_clear(side: SideState, frames: int = 10):
  for _ in range(frames):
    side.update_obstacles(25.0, SimpleNamespace(status=False), False, False)


class TestLaneChangeSideObstacles:
  def test_blocks_closing_side_lead_by_ttc(self):
    side = SideState("left")
    settle_clear(side)

    side.update_obstacles(25.0, lead(20.0, -10.0, 25.0), False, False)

    assert side.side_object_detected

  def test_allows_side_lead_pulling_away_with_gap(self):
    side = SideState("left")
    settle_clear(side)

    side.update_obstacles(25.0, lead(20.0, 5.0, 25.0), False, False)

    assert not side.side_object_detected

  def test_blocks_close_side_lead_even_when_pulling_away(self):
    side = SideState("left")
    settle_clear(side)

    side.update_obstacles(25.0, lead(5.0, 5.0, 25.0), False, False)

    assert side.side_object_detected

  def test_receding_close_track_releases_bsd_hold_early(self):
    side = SideState("left")
    no_lead = SimpleNamespace(status=False)
    side.update_obstacles(25.0, no_lead, True, False)

    for frame in range(6):
      receding = lead(4.0 + frame * 0.45, 9.0, 25.0, track_id=1661)
      side.update_obstacles(25.0, no_lead, False, False, radar_objects=(receding,))
      if frame < 5:
        assert side.bsd_hold_counter > 0

    assert side.bsd_hold_counter == 0

  def test_far_receding_track_does_not_release_bsd_hold(self):
    side = SideState("left")
    no_lead = SimpleNamespace(status=False)
    side.update_obstacles(25.0, no_lead, True, False)

    for frame in range(10):
      far_track = lead(40.0 + frame * 0.5, 5.0, 25.0, track_id=1662)
      side.update_obstacles(25.0, no_lead, False, False, radar_objects=(far_track,))

    assert side.bsd_hold_counter > 0

  def test_changing_track_ids_do_not_release_bsd_hold(self):
    side = SideState("left")
    no_lead = SimpleNamespace(status=False)
    side.update_obstacles(25.0, no_lead, True, False)

    for frame in range(10):
      transient = lead(4.0 + frame * 0.1, 9.0, 25.0, track_id=1700 + frame)
      side.update_obstacles(25.0, no_lead, False, False, radar_objects=(transient,))

    assert side.bsd_hold_counter > 0

  def test_other_unsafe_track_prevents_early_release(self):
    side = SideState("left")
    no_lead = SimpleNamespace(status=False)
    side.update_obstacles(25.0, no_lead, True, False)

    for frame in range(10):
      receding = lead(4.0 + frame * 0.45, 9.0, 25.0, track_id=1661)
      closing = lead(8.0, -5.0, 25.0, track_id=1777)
      side.update_obstacles(25.0, no_lead, False, False, radar_objects=(receding, closing))

    assert side.bsd_hold_counter > 0

  def test_front_track_does_not_release_bsd_hold_early(self):
    side = SideState("left")
    no_lead = SimpleNamespace(status=False)
    side.update_obstacles(25.0, no_lead, True, False)

    for frame in range(10):
      front_track = lead(4.0 + frame * 0.45, 9.0, 25.0, track_id=46)
      side.update_obstacles(25.0, no_lead, False, False, radar_objects=(front_track,))

    assert side.bsd_hold_counter > 0

  def test_unsafe_corner_track_blocks_after_bsd_hold_expires(self):
    side = SideState("left")
    no_lead = SimpleNamespace(status=False)
    side.update_obstacles(25.0, no_lead, True, False)

    corner_track = lead(4.0, 0.0, 25.0, track_id=1661)
    for _ in range(45):
      side.update_obstacles(25.0, no_lead, False, False, radar_objects=(corner_track,))

    assert side.bsd_hold_counter == 0
    assert side.side_object_detected


def rear_step(side: SideState, rear_dist: float, blindspot: bool = True):
  side.update_obstacles(25.0, SimpleNamespace(status=False), blindspot, False, rear_dist=rear_dist)
  side.compute_lane_change_available(True, False)
  return side.bsd_hold_counter


def rear_side():
  side = SideState("left")
  side.lane_available = True
  return side


class TestBsdRearDistanceRelease:
  # Model rate 20 Hz: 0.08 m/frame = 1.6 m/s.
  def test_receding_passed_car_releases_bsd(self):
    side = rear_side()
    holds = [rear_step(side, 8.0 + 0.08 * i) for i in range(50)]

    assert all(h > 0 for h in holds[:10])  # 0.5 s of history first
    assert holds[10] == 0
    assert side.lane_change_available

  def test_approaching_car_keeps_blocking(self):
    side = rear_side()
    for i in range(50):
      assert rear_step(side, 12.0 - 0.08 * i) > 0
    assert not side.lane_change_available

  @pytest.mark.parametrize("rear_dist", [25.0, 0.0])
  def test_saturated_or_missing_distance_keeps_blocking(self, rear_dist):
    side = rear_side()
    for _ in range(50):
      assert rear_step(side, rear_dist) > 0
    assert not side.lane_change_available

  def test_jump_up_from_a_near_car_waits_two_seconds(self):
    side = rear_side()
    for _ in range(20):
      rear_step(side, 2.0)
    holds = [rear_step(side, 16.0 + 0.08 * i) for i in range(50)]

    # the near car may now be hidden behind the farther one on the single-object display (1109 seg100, 11-12)
    assert all(h > 0 for h in holds[:40])
    assert holds[40] == 0

  def test_jump_through_a_missing_frame_still_waits_two_seconds(self):
    side = rear_side()
    for _ in range(20):
      rear_step(side, 2.0)
    rear_step(side, 0.0)  # e.g. masked for one frame by the front-corner guard
    holds = [rear_step(side, 16.0 + 0.08 * i) for i in range(50)]
    assert all(h > 0 for h in holds[:39])

  def test_jump_between_far_objects_waits_for_a_continuous_window(self):
    side = rear_side()
    for _ in range(20):
      rear_step(side, 9.0)
    holds = [rear_step(side, 16.0 + 0.08 * i) for i in range(30)]

    assert all(h > 0 for h in holds[:10])
    assert holds[10] == 0

  def test_hold_rearms_when_the_car_closes_again(self):
    side = rear_side()
    d = 8.0
    for _ in range(20):
      d += 0.08
      rear_step(side, d)
    assert side.bsd_hold_counter == 0

    released = []
    for _ in range(10):
      d -= 0.08
      released.append(rear_step(side, d) == 0)

    assert released.index(False) == 1  # window rate drops under 1 m/s on the 2nd closing frame
    assert side.bsd_hold_counter == int(2.0 / DT_MDL)
    assert not side.lane_change_available

  def test_receding_under_min_distance_keeps_hold(self):
    side = rear_side()
    holds = [rear_step(side, 5.0 + 0.08 * i) for i in range(45)]

    assert holds[37] > 0   # 7.96 m
    assert holds[38] == 0  # 8.04 m

  def test_default_rear_distance_keeps_old_behaviour(self):
    side = SideState("left")
    side.update_obstacles(25.0, SimpleNamespace(status=False), True, False)
    for _ in range(10):
      side.update_obstacles(25.0, SimpleNamespace(status=False), False, False)
    assert side.bsd_hold_counter == int(2.0 / DT_MDL) - 10


class TestLaneChangeAvailabilityRelease:
  def test_legal_lane_color_releases_available_side(self):
    side = SideState("left")
    side.lane_available = True
    side.compute_lane_change_available(False, False)
    side.commit_last()

    side.compute_lane_change_available(True, False)

    assert side.lane_change_available
    assert side.lane_change_available_released

  def test_appearing_side_lane_releases_availability(self):
    side = SideState("left")
    side.compute_lane_change_available(True, False)
    side.commit_last()

    side.lane_available = True
    side.compute_lane_change_available(True, False)

    assert side.lane_change_available
    assert side.lane_change_available_released


@pytest.mark.parametrize("front, vision, rear, expected", [
  (0.0, False, 12.0, 12.0),   # only a car behind: the release may use it
  (5.0, False, 12.0, 0.0),    # a car beside (front corner 0..7 m) also sets the blindspot: no release
  (9.0, False, 12.0, 12.0),   # front object beyond 7 m does not set the blindspot
  (0.0, True, 12.0, 0.0),     # vision BSD also feeds the blindspot: no release
])
def test_rear_release_distance_ignores_cars_beside(front, vision, rear, expected):
  assert SideState.rear_release_distance(front, rear, vision) == expected


def test_desire_helper_passes_each_side_its_own_rear_distance():
  from openpilot.selfdrive.controls.lib.desire_helper import DesireHelper
  helper = DesireHelper()
  seen = {}
  for name in ("left", "right"):
    side = getattr(helper, name)
    side.update_lane_geometry = lambda *a, **k: None
    side.update_lane_line_info = lambda *a, **k: None
    side.update_obstacles = (lambda n: lambda *a, **k: seen.__setitem__(n, (a[2], k["rear_dist"])))(name)
  car = SimpleNamespace(vEgo=20.0, leftLaneLine=0, rightLaneLine=0, leftBlindspot=False, rightBlindspot=True,
                        leftLongDist=0.0, rightLongDist=0.0, leftRearLongDist=3.0, rightRearLongDist=12.0,
                        leftBlindspotOnnx=False, rightBlindspotOnnx=False)
  model = SimpleNamespace(laneLines=[None] * 4, laneLineProbs=[0.0] * 4, roadEdges=[None] * 2)
  radar = SimpleNamespace(leadLeft=None, leadRight=None, leadsLeft=[], leadsRight=[])
  try:
    helper._process_sides(car, model, radar)
  except (AttributeError, TypeError):
    pass  # only the obstacle wiring is under test; later steps need a full model
  assert seen == {"left": (False, 3.0), "right": (True, 12.0)}

  car.rightLongDist = 5.0  # a car beside on the right: only the right side loses the rear release
  seen.clear()
  try:
    helper._process_sides(car, model, radar)
  except (AttributeError, TypeError):
    pass
  assert seen == {"left": (False, 3.0), "right": (True, 0.0)}
