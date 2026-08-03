from openpilot.selfdrive.carrot.server.services import web_settings


def test_carrot_navi_fullscreen_on_tap_is_explicit_opt_in():
  defaults = web_settings.sanitize_web_settings({})
  assert defaults["carrot_navi_fullscreen_on_tap"] is False

  enabled = web_settings.sanitize_web_settings({"carrot_navi_fullscreen_on_tap": "true"})
  assert enabled["carrot_navi_fullscreen_on_tap"] is True

  spec = {field["key"]: field for field in web_settings.web_settings_client_spec()}
  assert spec["carrot_navi_fullscreen_on_tap"] == {
    "key": "carrot_navi_fullscreen_on_tap",
    "type": "bool",
    "default": False,
  }
