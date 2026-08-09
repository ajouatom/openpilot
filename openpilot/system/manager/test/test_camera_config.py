from openpilot.system.manager.camera_config import configure_wide_camera


class FakeParams:
  def __init__(self, enabled):
    self.enabled = enabled

  def get(self, key, return_default=False):
    assert (key, return_default) == ("UseWideCamera", True)
    return self.enabled


def test_enabled_wide_camera_preserves_environment():
  environ = {}
  assert configure_wide_camera(FakeParams(True), environ)
  assert "DISABLE_WIDE_ROAD" not in environ


def test_disabled_wide_camera_configures_camerad_environment():
  environ = {}
  assert not configure_wide_camera(FakeParams(False), environ)
  assert environ["DISABLE_WIDE_ROAD"] == "1"
