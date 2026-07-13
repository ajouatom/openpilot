from openpilot.selfdrive.controls.lib.cutin_helpers import (
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
    assert not is_front_radar_cutin_candidate(32, "frontRadar", 12.1, 3.0, False)
    assert not is_front_radar_cutin_candidate(32, "frontRadar", 8.0, 7.1, False)
