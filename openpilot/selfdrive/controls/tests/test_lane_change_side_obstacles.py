from types import SimpleNamespace

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
