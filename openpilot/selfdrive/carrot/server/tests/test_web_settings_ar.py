from openpilot.selfdrive.carrot.server.services import web_settings


def test_vision_ar_is_explicit_opt_in():
  defaults = web_settings.sanitize_web_settings({})
  assert defaults["vision_ar_enabled"] is False

  enabled = web_settings.sanitize_web_settings({"vision_ar_enabled": "true"})
  assert enabled["vision_ar_enabled"] is True

  spec = {field["key"]: field for field in web_settings.web_settings_client_spec()}
  assert spec["vision_ar_enabled"] == {
    "key": "vision_ar_enabled",
    "type": "bool",
    "default": False,
    "requiresCapability": "web_lab",
  }
