import pytest

from openpilot.selfdrive.selfdrived.camera_config import get_camera_packets


@pytest.mark.parametrize('wide,expected', [
  (True, ['roadCameraState', 'wideRoadCameraState']),
  (False, ['roadCameraState']),
])
def test_road_camera_health_is_independent_of_driver_camera_fallback(wide, expected):
  assert get_camera_packets(wide) == expected
