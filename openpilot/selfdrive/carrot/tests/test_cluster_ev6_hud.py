"""Selected EV6 HUD integration: real defaults, vehicle identity and GPU ownership."""
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
def clean_hud(monkeypatch):
  monkeypatch.setenv("EV6_HUD_CLEAN", "1")
  monkeypatch.setattr(cluster_scene, "_clean_ui_cache", [-10.0, True])


def test_production_hud_default_is_also_exercised_under_pytest(monkeypatch, tmp_path):
  monkeypatch.delenv("EV6_HUD_CLEAN")
  monkeypatch.setattr(cluster_scene, "CLEAN_UI_CLASSIC_FLAG", str(tmp_path / "classic"))
  assert "pytest" in sys.modules
  assert cluster_scene.clean_ui_enabled()
  (tmp_path / "classic").touch()
  cluster_scene._clean_ui_cache[0] = -10.0
  assert not cluster_scene.clean_ui_enabled()


@pytest.mark.parametrize("fingerprint,name", list(EGO_MODEL_FINGERPRINTS.items()) + [
  ("KIA_EV6", "ev6"), ("KIA_EV6_PE", "ev6"), ("HYUNDAI_IONIQ_6", "ev6"), ("", "ev6"), ("unknown", "ev6"),
])
def test_detected_identity_reaches_the_rendered_ego(fingerprint, name):
  parser = RouteLogParser(recompute_cutins=False)
  parser._update_car_params(SimpleNamespace(brand="hyundai", carFingerprint=fingerprint))
  frame = parser._frame_from_car_state(SimpleNamespace(), 1.0)
  state = frame_to_state(frame)
  assert state.car_fingerprint == fingerprint
  assert ego_model_name(state.car_fingerprint) == name
  ego = next(v for v in cluster_scene.build_cluster_scene(state).vehicles if not v.source)
  assert ego.model_name == name


def test_identity_changes_invalidate_scene_cache():
  first = replace(standby_state(), car_fingerprint="HYUNDAI_IONIQ_5")
  second = replace(first, car_fingerprint="unknown")
  assert cluster_scene.cluster_scene_state_key(first) != cluster_scene.cluster_scene_state_key(second)


def test_clean_vehicle_labels_keep_lead_and_cutin_with_closing_warning(monkeypatch):
  renderer = ClusterUiRenderer()
  labels = []
  monkeypatch.setattr(renderer, "_rounded_rect", lambda *_: None)
  monkeypatch.setattr(renderer, "_measure_text", lambda text, size, *_: (len(text) * size / 2, size))
  monkeypatch.setattr(renderer, "_draw_world_label_text", lambda text, x, y, size, color, **_: labels.append((text, color)))
  monkeypatch.setattr(renderer_module, "world_to_screen_label_anchor",
                      lambda point, *_: SimpleNamespace(x=100.0 + point.y * 6, y=220.0))
  def vehicle(label, distance, **kwargs):
    return cluster_scene.vehicle_box(0.0, distance, 0.0, 3.6, (255, 255, 255), False,
                                     label=label, source="radarState", longitudinal_m=distance,
                                     absolute_speed_kph=50, **kwargs)
  lead = vehicle("L1", 20, primary=True, relative_speed_mps=-5.0, ttc_s=2.0)
  cutin = vehicle("CUT-IN", 70, cut_in=True)
  side = vehicle("RF", 120)
  renderer._draw_vehicle_badges((lead, cutin, side), object())
  assert len(labels) == 2
  assert any("20 m" in text and color == renderer_module.RED for text, color in labels)
  assert any("70 m" in text for text, _ in labels)
  assert all("120 m" not in text for text, _ in labels)


def test_missing_model_falls_back_and_shared_model_paint_is_reset(monkeypatch):
  renderer = ClusterUiRenderer()
  renderer._vehicle_model = object()
  renderer._other_vehicle_models = [object()]
  renderer._other_vehicle_names = ["ioniq5"]
  paints = []
  monkeypatch.setattr(renderer, "_set_other_paint", lambda *args: paints.append(args))
  assert renderer._ego_model("ioniq5") == (renderer._other_vehicle_models[0], "ioniq5")
  assert paints == [(0, None)]
  assert renderer._ego_model("ev9") == (renderer._vehicle_model, "ev6")


@pytest.mark.parametrize("onroad,gear,expected", [(True, "P", True), (True, "D", False),
                                                 (True, "R", False), (False, "P", False)])
def test_park_and_reverse_screen_selection(monkeypatch, tmp_path, onroad, gear, expected):
  monkeypatch.setattr(renderer_module, "EV6_PARK_SCREEN_OFF_FLAG", str(tmp_path / "off"))
  monkeypatch.setattr(renderer_module, "EV6_REVERSE_SCREEN_OFF_FLAG", str(tmp_path / "reverse-off"))
  renderer = ClusterUiRenderer()
  assert renderer._ev6_reverse_screen_active(replace(standby_state(), onroad=onroad, gear_text=gear)) is (onroad and gear == "R")
  assert renderer._ev6_park_screen_active(replace(standby_state(), onroad=onroad, gear_text=gear)) is expected


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
  renderer._vehicle_model = object()
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
  renderer._vehicle_model = object()
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
