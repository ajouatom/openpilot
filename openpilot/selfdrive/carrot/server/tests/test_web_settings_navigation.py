import json

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


def test_fresh_drive_layout_defaults_to_full_area_one_navigation():
  defaults = web_settings.sanitize_web_settings({})

  for orientation in ("horizontal", "vertical"):
    assert defaults[f"carrot_navi_{orientation}_mode"] == "area_1"
    assert defaults[f"carrot_navi_{orientation}_area_1"] == "navigation"
    assert defaults[f"carrot_navi_{orientation}_area_2"] == "vision"


def test_existing_settings_without_layout_keys_keep_legacy_layout(tmp_path, monkeypatch):
  settings_path = tmp_path / "web_settings.json"
  settings_path.write_text(json.dumps({"start_page": "carrot"}), encoding="utf-8")
  monkeypatch.setattr(web_settings, "CARROT_WEB_SETTINGS_PATH", str(settings_path))

  settings = web_settings.read_web_settings()

  for orientation in ("horizontal", "vertical"):
    assert settings[f"carrot_navi_{orientation}_mode"] == "split"
    assert settings[f"carrot_navi_{orientation}_area_1"] == "vision"
    assert settings[f"carrot_navi_{orientation}_area_2"] == "navigation"
