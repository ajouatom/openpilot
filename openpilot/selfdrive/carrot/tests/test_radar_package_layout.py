import importlib.util


def test_model_radard_compatibility_entrypoint_is_installed() -> None:
  assert importlib.util.find_spec("openpilot.selfdrive.carrot.radard_model") is not None
  assert importlib.util.find_spec("openpilot.selfdrive.carrot.radar.radard_model") is not None
