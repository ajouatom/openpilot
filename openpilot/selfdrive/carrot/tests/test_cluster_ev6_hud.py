"""P/R-only integration: gear transitions, dormant driving UI and GPU ownership."""
from dataclasses import replace
from pathlib import Path
import sys
from types import SimpleNamespace

import pytest

CLUSTER = Path(__file__).resolve().parents[1] / "cluster"
sys.path.insert(0, str(CLUSTER))

import cluster_renderer as renderer_module
import cluster_scene
from cluster_live import standby_state
from cluster_renderer import ClusterUiRenderer
from cluster_route_replay import RouteLogParser, frame_to_state
from cluster_vehicle_models import EGO_MODEL_FINGERPRINTS, ego_model_name


@pytest.fixture(autouse=True)
def device_flags(monkeypatch, tmp_path):
  monkeypatch.setattr(renderer_module, "EV6_PARK_SCREEN_OFF_FLAG", str(tmp_path / "park-off"))
  monkeypatch.setattr(renderer_module, "EV6_REVERSE_SCREEN_OFF_FLAG", str(tmp_path / "reverse-off"))


@pytest.mark.parametrize("override", [None, "0", "1"])
def test_imported_driving_layout_cannot_be_enabled_by_environment(monkeypatch, override):
  if override is None:
    monkeypatch.delenv("EV6_HUD_CLEAN", raising=False)
  else:
    monkeypatch.setenv("EV6_HUD_CLEAN", override)
  assert not cluster_scene.clean_ui_enabled()
  renderer = ClusterUiRenderer()
  state = replace(standby_state(), onroad=True, gear_text="P")
  assert renderer._ev6_park_screen_active(state)
  assert renderer._ev6_reverse_screen_active(replace(state, gear_text="R"))


@pytest.mark.parametrize("fingerprint,name", list(EGO_MODEL_FINGERPRINTS.items()) + [
  ("KIA_EV6", "ev6"), ("KIA_EV6_PE", "ev6"), ("HYUNDAI_IONIQ_6", "ev6"), ("", "ev6"), ("unknown", "ev6"),
])
def test_park_identity_does_not_replace_the_driving_ego(fingerprint, name):
  parser = RouteLogParser(recompute_cutins=False)
  parser._update_car_params(SimpleNamespace(brand="hyundai", carFingerprint=fingerprint))
  frame = parser._frame_from_car_state(SimpleNamespace(), 1.0)
  state = frame_to_state(frame)
  assert state.car_fingerprint == fingerprint
  assert ego_model_name(state.car_fingerprint) == name
  ego = next(v for v in cluster_scene.build_cluster_scene(state).vehicles if not v.source)
  assert ego.model_name == "cybertruck"
  assert ego.length_m != cluster_scene.EV6_EGO_LENGTH_M


def test_identity_changes_invalidate_scene_cache():
  first = replace(standby_state(), car_fingerprint="HYUNDAI_IONIQ_5")
  second = replace(first, car_fingerprint="unknown")
  assert cluster_scene.cluster_scene_state_key(first) != cluster_scene.cluster_scene_state_key(second)


@pytest.mark.parametrize("screen_mode", range(-1, 8))
def test_gear_transitions_restore_selected_mode_and_keep_alerts(monkeypatch, screen_mode):
  from openpilot.common import indicator_display

  renderer = ClusterUiRenderer()
  renderer.screen_mode = screen_mode
  calls = []
  for method, label in (("_ev6_draw_park_screen", "P"), ("_ev6_draw_reverse_screen", "R"),
                        ("_render_world", "world"), ("_clear_world", "clear"), ("_draw_hud", "hud")):
    monkeypatch.setattr(renderer, method, lambda *_, label=label: calls.append(label))
  monkeypatch.setattr(indicator_display, "draw", lambda *_: calls.append("indicator"))
  alert = object()
  monkeypatch.setattr(renderer, "_draw_alert_overlay", lambda value: calls.append(value))
  world = "clear" if screen_mode in (renderer_module.CLUSTER_SCREEN_MODE_DEBUG_GRAPH,
                                    renderer_module.CLUSTER_SCREEN_MODE_NAVI) else "world"
  for gear, onroad in (("D", True), ("P", True), ("R", True), ("D", True), ("N", True),
                       ("L", True), ("B", True), ("?", True), ("P", False), ("R", False)):
    calls.clear()
    state = replace(standby_state(), onroad=onroad, gear_text=gear, alert=alert)
    renderer.render(state, signal_lights=(False, False))
    expected = [gear] if onroad and gear in ("P", "R") else [world, "hud"]
    assert calls == [*expected, "indicator", alert]
    assert renderer.screen_mode == screen_mode


@pytest.mark.parametrize("gear,onroad", [("D", True), ("N", True), ("L", True), ("", True), ("P", False), ("R", False)])
def test_driving_does_not_load_or_prepare_park_resources(monkeypatch, gear, onroad):
  renderer = ClusterUiRenderer()
  def unexpected(*_):
    pytest.fail("P/R resources must not be prepared by the driving path")
  for method in ("_load_park_vehicle_models", "_prepare_park_model_texture", "_ev6_prepare_reverse_sprite"):
    monkeypatch.setattr(renderer, method, unexpected)
  monkeypatch.setattr(renderer, "_effective_screen_mode", lambda _: -1)
  renderer._prepare_trip_report_cache(replace(standby_state(), gear_text=gear, onroad=onroad))


def test_loaded_park_models_never_replace_driving_models(monkeypatch):
  renderer = ClusterUiRenderer()
  renderer._vehicle_model = object()
  renderer._park_vehicle_model = object()
  renderer._other_vehicle_models = [object()]
  draws = []
  monkeypatch.setattr(renderer_module.rl, "draw_model_ex", lambda model, *_: draws.append(model))
  monkeypatch.setattr(renderer_module.rl, "rl_disable_backface_culling", lambda: None)
  monkeypatch.setattr(renderer_module.rl, "rl_enable_backface_culling", lambda: None)
  for source, primary, cut_in in (("", False, False), ("radarState", True, False),
                                 ("radarState", True, True), ("cornerRadar", False, False)):
    vehicle = cluster_scene.vehicle_box(0.0, 20.0, 0.0, 3.6, (255, 255, 255), False,
                                       source=source, primary=primary, cut_in=cut_in)
    assert not renderer._uses_other_model(vehicle)
    renderer._draw_vehicle_model(vehicle)
  assert draws == [renderer._vehicle_model] * 4


def test_missing_model_falls_back_and_shared_model_paint_is_reset(monkeypatch):
  renderer = ClusterUiRenderer()
  renderer._park_vehicle_model = object()
  renderer._other_vehicle_models = [object()]
  renderer._other_vehicle_names = ["ioniq5"]
  paints = []
  monkeypatch.setattr(renderer, "_set_other_paint", lambda *args: paints.append(args))
  assert renderer._ego_model("ioniq5") == (renderer._other_vehicle_models[0], "ioniq5")
  assert paints == [(0, None)]
  assert renderer._ego_model("ev9") == (renderer._park_vehicle_model, "ev6")


@pytest.mark.parametrize("onroad,gear,expected", [(True, "P", True), (True, "D", False),
                                                 (True, "R", False), (False, "P", False)])
def test_park_and_reverse_screen_selection(monkeypatch, tmp_path, onroad, gear, expected):
  monkeypatch.setattr(renderer_module, "EV6_PARK_SCREEN_OFF_FLAG", str(tmp_path / "off"))
  monkeypatch.setattr(renderer_module, "EV6_REVERSE_SCREEN_OFF_FLAG", str(tmp_path / "reverse-off"))
  renderer = ClusterUiRenderer()
  assert renderer._ev6_reverse_screen_active(replace(standby_state(), onroad=onroad, gear_text=gear)) is (onroad and gear == "R")
  assert renderer._ev6_park_screen_active(replace(standby_state(), onroad=onroad, gear_text=gear)) is expected


def test_individual_screen_disable_files_preserve_the_other_gear(monkeypatch):
  renderer = ClusterUiRenderer()
  state = replace(standby_state(), onroad=True)
  monkeypatch.setattr(renderer_module.time, "monotonic", lambda: 100.0)
  Path(renderer_module.EV6_PARK_SCREEN_OFF_FLAG).touch()
  assert not renderer._ev6_park_screen_active(replace(state, gear_text="P"))
  assert renderer._ev6_reverse_screen_active(replace(state, gear_text="R"))
  Path(renderer_module.EV6_PARK_SCREEN_OFF_FLAG).unlink()
  Path(renderer_module.EV6_REVERSE_SCREEN_OFF_FLAG).touch()
  monkeypatch.setattr(renderer_module.time, "monotonic", lambda: 102.0)
  assert renderer._ev6_park_screen_active(replace(state, gear_text="P"))
  assert not renderer._ev6_reverse_screen_active(replace(state, gear_text="R"))


def test_reverse_reference_bends_with_steering():
  straight = ClusterUiRenderer.ev6_reverse_path(0.0)
  left = ClusterUiRenderer.ev6_reverse_path(160.0)
  right = ClusterUiRenderer.ev6_reverse_path(-160.0)
  assert all(point[0] == 0.0 for point in straight)
  assert left[-1][0] < 0.0 < right[-1][0]
  assert left[-1][0] == pytest.approx(-right[-1][0])
  assert left[-1][1] == pytest.approx(right[-1][1])


@pytest.mark.parametrize("fraction,expected", [(0.0, "0%"), (0.1, "10%"), (0.765, "76%"), (1.0, "100%")])
def test_park_battery_uses_the_existing_normalized_vehicle_value(monkeypatch, fraction, expected):
  renderer = ClusterUiRenderer()
  labels = []
  monkeypatch.setattr(renderer, "_rounded_rect", lambda *_: None)
  monkeypatch.setattr(renderer_module.rl, "draw_rectangle_rounded", lambda *_: None)
  renderer._ev6_draw_battery(fraction, 0.0, 0.0, renderer_module.EV6_PARK_COLORS_DARK,
                             lambda text, *_: labels.append(text))
  assert labels == [expected]


@pytest.mark.parametrize("interrupted", [False, True])
def test_park_texture_cleanup_releases_all_handles_and_can_repeat(monkeypatch, interrupted):
  renderer = ClusterUiRenderer()
  renderer._ev6_park_textures = ["light", "dark"]
  renderer._park_model_texture = "model"
  renderer._park_model_key = "ioniq5"
  renderer._ev6_reverse_sprite = ("reverse", (0, 0, 1, 1))
  renderer._ev6_reverse_sprite_key = ("ioniq5", 1920, 462)
  renderer._park_model_retry_t = 1000.0
  unloaded = []

  def unload(texture):
    unloaded.append(texture)
    if interrupted and texture == "light":
      raise KeyboardInterrupt()

  monkeypatch.setattr(renderer_module.rl, "unload_texture", unload)
  if interrupted:
    with pytest.raises(KeyboardInterrupt):
      renderer._close_park_textures()
  else:
    renderer._close_park_textures()
  assert unloaded == ["light", "dark", "model", "reverse"]
  assert renderer._ev6_park_textures is None
  assert renderer._park_model_texture is None and renderer._park_model_key is None
  assert renderer._ev6_reverse_sprite is None and renderer._ev6_reverse_sprite_key is None
  assert renderer._park_model_retry_t == renderer._ev6_park_retry_t == 0.0
  renderer._close_park_textures()
  assert unloaded == ["light", "dark", "model", "reverse"]


def test_normal_renderer_close_clears_park_handles_before_window(monkeypatch):
  renderer = ClusterUiRenderer()
  renderer._window_open = True
  renderer._ev6_park_textures = ["photo"]
  renderer._park_model_texture = "model"
  calls = []
  monkeypatch.setattr(renderer_module.rl, "unload_texture", lambda t: calls.append(t))
  monkeypatch.setattr(renderer_module.rl, "close_window", lambda: calls.append("window"))
  renderer.close()
  assert calls == ["photo", "model", "window"]
  renderer.close()
  assert calls == ["photo", "model", "window"]


def test_partial_photo_upload_releases_images_and_uploaded_texture(monkeypatch, tmp_path):
  renderer = ClusterUiRenderer()
  photo = tmp_path / "photo.png"
  photo.touch()
  monkeypatch.setenv("EV6_PARK_CAR_IMAGE", str(photo))
  ffi = renderer_module.rl.ffi
  image = SimpleNamespace(width=1, height=1, data=ffi.new("unsigned char[]", [1, 2, 3, 255]))
  dark = SimpleNamespace(width=1, height=1, data=ffi.new("unsigned char[]", [1, 2, 3, 255]))
  freed = []
  rl = renderer_module.rl
  monkeypatch.setattr(rl, "load_image", lambda _: image)
  monkeypatch.setattr(rl, "image_format", lambda *_: None)
  monkeypatch.setattr(rl, "image_copy", lambda _: dark)
  monkeypatch.setattr(rl, "load_texture_from_image", lambda _: "partial")
  monkeypatch.setattr(rl, "gen_texture_mipmaps", lambda _: (_ for _ in ()).throw(RuntimeError("upload failed")))
  monkeypatch.setattr(rl, "unload_image", lambda i: freed.append(i))
  monkeypatch.setattr(rl, "unload_texture", lambda t: freed.append(t))
  assert renderer._ev6_park_car_texture(False) is None
  assert freed == [image, dark, "partial"]
  assert renderer._ev6_park_textures is None


def test_park_preview_failure_unwinds_render_target_and_image(monkeypatch, tmp_path):
  renderer = ClusterUiRenderer()
  renderer._park_vehicle_model = object()
  monkeypatch.setattr(renderer_module, "EV6_PARK_SCREEN_OFF_FLAG", str(tmp_path / "off"))
  rl = renderer_module.rl
  calls = []
  target = SimpleNamespace(texture="target")
  image = object()
  monkeypatch.setattr(renderer, "_crop_car_image", lambda _: True)
  monkeypatch.setattr(rl, "load_render_texture", lambda *_: target)
  monkeypatch.setattr(rl, "begin_texture_mode", lambda *_: calls.append("begin_target"))
  monkeypatch.setattr(rl, "end_texture_mode", lambda: calls.append("end_target"))
  monkeypatch.setattr(rl, "begin_mode_3d", lambda *_: calls.append("begin_3d"))
  monkeypatch.setattr(rl, "end_mode_3d", lambda: calls.append("end_3d"))
  for name in ("clear_background", "rl_disable_backface_culling", "rl_enable_backface_culling", "draw_model_ex"):
    monkeypatch.setattr(rl, name, lambda *_: None)
  monkeypatch.setattr(rl, "load_image_from_texture", lambda _: image)
  monkeypatch.setattr(rl, "image_flip_vertical", lambda _: None)
  monkeypatch.setattr(rl, "load_texture_from_image", lambda _: (_ for _ in ()).throw(RuntimeError("upload failed")))
  monkeypatch.setattr(rl, "unload_image", lambda _: calls.append("free_image"))
  monkeypatch.setattr(rl, "unload_render_texture", lambda _: calls.append("free_target"))
  renderer._prepare_park_model_texture(replace(standby_state(), onroad=True, gear_text="P"))
  assert calls == ["begin_target", "begin_3d", "end_3d", "end_target", "free_image", "free_target"]
  assert renderer._park_model_texture is None and renderer._park_model_key is None


def test_park_preview_switches_to_fallback_without_a_retry_delay(monkeypatch, tmp_path):
  renderer = ClusterUiRenderer()
  renderer._park_vehicle_model = object()
  renderer._other_vehicle_models = [object()]
  renderer._other_vehicle_names = ["ioniq5"]
  rl = renderer_module.rl
  target = SimpleNamespace(texture="target")
  freed = []
  uploads = iter(["ioniq5-texture", "ev6-texture"])
  monkeypatch.setattr(renderer_module, "EV6_PARK_SCREEN_OFF_FLAG", str(tmp_path / "off"))
  monkeypatch.setattr(renderer_module.time, "monotonic", lambda: 100.0)
  monkeypatch.setattr(renderer, "_crop_car_image", lambda _: True)
  monkeypatch.setattr(rl, "load_render_texture", lambda *_: target)
  monkeypatch.setattr(rl, "load_image_from_texture", lambda _: object())
  monkeypatch.setattr(rl, "load_texture_from_image", lambda _: next(uploads))
  monkeypatch.setattr(rl, "unload_texture", lambda t: freed.append(t))
  for name in ("begin_texture_mode", "end_texture_mode", "begin_mode_3d", "end_mode_3d", "clear_background",
               "rl_disable_backface_culling", "rl_enable_backface_culling", "draw_model_ex", "image_flip_vertical",
               "gen_texture_mipmaps", "set_texture_filter", "unload_image", "unload_render_texture"):
    monkeypatch.setattr(rl, name, lambda *_: None)
  state = replace(standby_state(), onroad=True, gear_text="P", car_fingerprint="HYUNDAI_IONIQ_5")
  renderer._prepare_park_model_texture(state)
  renderer._prepare_park_model_texture(replace(state, car_fingerprint="unknown"))
  assert freed == ["ioniq5-texture"]
  assert renderer._park_model_texture == "ev6-texture"
  assert renderer._park_model_key == "ev6"
