from openpilot.selfdrive.carrot.server.services import web_capabilities, web_settings
from openpilot.selfdrive.carrot.server.terminal_commands.custom_commands import web_lab


def test_web_lab_is_declarative_and_off_by_default():
  settings = web_settings.sanitize_web_settings({})
  assert settings["web_lab_enabled"] is False
  assert web_capabilities.resolve_web_capabilities(settings) == {"web_lab": False}
  assert web_capabilities.web_capability_client_spec() == [{
    "id": "web_lab",
    "settingKey": "web_lab_enabled",
    "labelKey": "web_lab",
    "lockedLabelKey": "web_lab_locked",
  }]
  assert web_settings.web_setting_defaults_for_capability("web_lab") == {
    "vision_ar_enabled": False,
    "vision_ar_debug": False,
  }


def test_web_lab_unlock_never_enables_gated_features(tmp_path, monkeypatch):
  settings_path = tmp_path / "web_settings.json"
  monkeypatch.setattr(web_settings, "CARROT_WEB_SETTINGS_PATH", str(settings_path))

  web_settings.write_web_settings({
    "web_lab_enabled": False,
    "vision_ar_enabled": True,
    "vision_ar_debug": True,
  })
  enabled = web_capabilities.set_web_capability_enabled("web_lab", True)
  assert enabled["enabled"] is True
  assert enabled["settings"]["vision_ar_enabled"] is False
  assert enabled["settings"]["vision_ar_debug"] is False

  web_settings.update_web_settings({
    "vision_ar_enabled": True,
    "vision_ar_debug": True,
  })
  disabled = web_capabilities.set_web_capability_enabled("web_lab", False)
  assert disabled["enabled"] is False
  assert disabled["settings"]["vision_ar_enabled"] is False
  assert disabled["settings"]["vision_ar_debug"] is False


def test_carrot_web_lab_command_controls_the_shared_capability(tmp_path, monkeypatch, capsys):
  settings_path = tmp_path / "web_settings.json"
  monkeypatch.setattr(web_settings, "CARROT_WEB_SETTINGS_PATH", str(settings_path))

  assert web_lab.run(["status"]) == 0
  assert "[web-lab] off" in capsys.readouterr().out

  assert web_lab.run(["on"]) == 0
  assert web_capabilities.resolve_web_capabilities(web_settings.read_web_settings())["web_lab"] is True

  assert web_lab.run(["off"]) == 0
  assert web_capabilities.resolve_web_capabilities(web_settings.read_web_settings())["web_lab"] is False

  assert web_lab.run(["unknown"]) == 2
