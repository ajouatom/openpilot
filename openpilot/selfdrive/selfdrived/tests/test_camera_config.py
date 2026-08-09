import pytest

from openpilot.selfdrive.selfdrived.camera_config import get_camera_packets


@pytest.mark.parametrize(
  ("use_wide_camera", "disable_dm", "simulation", "expected"),
  [
    (True, 0, False, ["roadCameraState", "driverCameraState", "wideRoadCameraState"]),
    (False, 0, False, ["roadCameraState", "driverCameraState"]),
    (True, 1, False, ["roadCameraState", "wideRoadCameraState"]),
    (False, 1, False, ["roadCameraState"]),
    (False, 1, True, ["roadCameraState", "driverCameraState"]),
  ],
)
def test_get_camera_packets(use_wide_camera, disable_dm, simulation, expected):
  assert get_camera_packets(use_wide_camera, disable_dm, simulation) == expected
