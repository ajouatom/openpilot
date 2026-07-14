from openpilot.selfdrive.controls.lib.cutin_helpers import (
  associate_cutin_tracks,
  combine_cutin_future_projection,
  is_cutin_track_discontinuous,
  is_fast_cutin_entry,
  is_front_radar_cutin_candidate,
  is_front_radar_cutin_enabled,
)


class TestFrontRadarCutin:
  def test_corner_track_lateral_jump_resets_cutin_history(self):
    assert is_cutin_track_discontinuous(
      True,
      prev_d_rel=18.75,
      prev_y_rel=10.70,
      prev_v_lead=5.0,
      d_rel=17.80,
      y_rel=9.80,
      v_lead=5.0,
    )
    assert not is_cutin_track_discontinuous(
      True,
      prev_d_rel=18.75,
      prev_y_rel=10.70,
      prev_v_lead=5.0,
      d_rel=18.50,
      y_rel=10.50,
      v_lead=5.0,
    )

  def test_enable_condition(self):
    assert is_front_radar_cutin_enabled(3, 0, "hyundai")
    assert is_front_radar_cutin_enabled(3, 1, "hyundai")
    assert not is_front_radar_cutin_enabled(2, 0, "hyundai")
    assert not is_front_radar_cutin_enabled(3, 2, "hyundai")
    assert not is_front_radar_cutin_enabled(3, 0, "volkswagen")

  def test_candidate_limits(self):
    assert is_front_radar_cutin_candidate(32, "frontRadar", 8.0, 3.0, False)
    assert not is_front_radar_cutin_candidate(0, "scc", 8.0, 0.0, False)
    assert not is_front_radar_cutin_candidate(200, "corner235", 8.0, 3.0, True)
    assert not is_front_radar_cutin_candidate(32, "frontRadar", 4.9, 3.0, False)
    assert is_front_radar_cutin_candidate(32, "frontRadar", 40.0, 3.0, False)
    assert not is_front_radar_cutin_candidate(32, "frontRadar", 50.1, 3.0, False)
    assert not is_front_radar_cutin_candidate(32, "frontRadar", 8.0, 7.1, False)

  def test_fast_entry_rejects_rapidly_pulling_away_track(self):
    args = (6.2, 20.0, 2.84, 1.59, 0.72, 0.45)
    assert is_fast_cutin_entry(*args, v_rel=3.0)
    assert not is_fast_cutin_entry(*args, v_rel=3.01)

  def test_fast_entry_rejects_lane_motion_not_supported_by_radar(self):
    assert not is_fast_cutin_entry(
      d_rel=2.8,
      v_ego=11.54,
      d_path=-2.88,
      lane_half_width=1.42,
      inward_speed=0.89,
      radar_inward_speed=0.20,
      v_rel=-4.75,
    )

  def test_future_projection_limits_lane_motion_to_radar_evidence(self):
    d_path_future, in_lane_prob_future = combine_cutin_future_projection(
      d_path=-5.41,
      d_path_rate=3.50,
      horizon_s=1.5,
      lane_half_width=1.41,
      projected_d_path=-3.54,
      projected_in_lane_prob=0.0,
      radar_inward_speed=1.15,
    )

    assert d_path_future == -3.54
    assert in_lane_prob_future == 0.0

  def test_stable_corner_ids_do_not_inherit_another_objects_history(self):
    old_object = {1000: (3.6, -4.65, -4.05)}
    new_object = {1001: (4.0, -4.30, -4.95)}

    assert associate_cutin_tracks(old_object, new_object) == {}
    assert associate_cutin_tracks(old_object, {1000: (3.5, -4.60, -4.10)}) == {1000: 1000}
