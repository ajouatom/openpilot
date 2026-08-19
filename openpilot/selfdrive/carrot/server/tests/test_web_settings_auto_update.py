from openpilot.selfdrive.carrot.server.services import web_settings


def test_auto_update_reboot_defaults_off_and_accepts_supported_modes():
  defaults = web_settings.sanitize_web_settings({})
  assert defaults["auto_update_reboot"] == "off"

  assert web_settings.sanitize_web_settings({"auto_update_reboot": "park"})["auto_update_reboot"] == "park"
  assert web_settings.sanitize_web_settings({"auto_update_reboot": "disengaged"})["auto_update_reboot"] == "disengaged"
  assert web_settings.sanitize_web_settings({"auto_update_reboot": "unsafe"})["auto_update_reboot"] == "off"

  spec = {field["key"]: field for field in web_settings.web_settings_client_spec()}
  assert spec["auto_update_reboot"] == {
    "key": "auto_update_reboot",
    "type": "enum",
    "default": "off",
    "choices": ["disengaged", "off", "park"],
  }
