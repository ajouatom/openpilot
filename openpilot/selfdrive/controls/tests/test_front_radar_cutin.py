from openpilot.selfdrive.controls.lib.cutin_helpers import (
  is_fast_cutin_entry,
  is_front_radar_cutin_candidate,
  is_front_radar_cutin_enabled,
)


class TestFrontRadarCutin:
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
