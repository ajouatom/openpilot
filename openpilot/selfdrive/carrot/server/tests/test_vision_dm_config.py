from openpilot.selfdrive.carrot.server.services import vision_test
from openpilot.selfdrive.monitoring.config import configure_monitoring
from openpilot.selfdrive.monitoring.test_disable_dm_config import FakeParams


def test_vision_test_status_keeps_pending_mode_separate(monkeypatch):
  params = FakeParams({"DisableDM": 2, "DriverMonitoringMode": 0, "CarrotVisionEnabled": 0})
  configure_monitoring(params, {})
  monkeypatch.setattr(vision_test, "_params", lambda: params)
  monkeypatch.setattr(vision_test, "_read_state", dict)
  monkeypatch.setattr(vision_test, "_pid_alive", lambda *_args: False)
  monkeypatch.setattr(vision_test, "_children_status", lambda _state: {})
  monkeypatch.setattr(vision_test, "_vipc_streams", list)
  monkeypatch.setattr(vision_test, "_port_open", lambda _port: False)
  params.put("DisableDM", 0)
  device = vision_test.get_status()["device"]
  assert device["disable_dm_active"] == 2
  assert device["carrot_vision_enabled"] == 0
  configure_monitoring(params, {})
  assert vision_test.get_status()["device"]["disable_dm_active"] == 0
