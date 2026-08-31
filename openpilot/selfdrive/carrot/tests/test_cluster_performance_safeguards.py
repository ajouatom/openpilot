import ctypes
from collections import OrderedDict, deque
from dataclasses import replace
import importlib
import os
from pathlib import Path
import queue
import sys
import types

import pytest


CLUSTER_DIR = Path(__file__).resolve().parents[1] / "cluster"
CLUSTER_MAIN_PATH = CLUSTER_DIR / "main.py"
LOGGERD_SCONSCRIPT = Path(__file__).resolve().parents[3] / "system" / "loggerd" / "SConscript"
sys.path.insert(0, str(CLUSTER_DIR))

from cluster_git_status import GitBranchStatusProvider
import cluster_gles_readback
import cluster_h264_decoder
from cluster_h264_decoder import TiciH264DecodedBuffer
from cluster_h264_pipeline import (
  H264PipelineInitializationError,
  H264UsbPipeline,
  NATIVE_ACCESS_UNIT_QUEUE_MAX,
)
import cluster_renderer
from cluster_live import standby_state
from cluster_renderer import ClusterUiRenderer
from cluster_usb_display import product_id_for_hud_mode


def test_loggerd_builds_cluster_h264_bridges_on_tici():
  source = LOGGERD_SCONSCRIPT.read_text(encoding="utf-8")

  assert 'if arch in ("aarch64", "larch64"):' in source
  assert "'cluster_h264_encoder_bridge'" in source
  assert "'cluster_h264_decoder_bridge'" in source


def test_managed_h264_orientation_change_requests_restart():
  source = CLUSTER_MAIN_PATH.read_text(encoding="utf-8")

  assert "if h264_pipeline is not None and hud_mode_watch is not None:" in source
  assert "exiting for H264 restart" in source


def test_tici_decoded_buffer_releases_fd_and_capture_lease_once():
  read_fd, write_fd = os.pipe()
  owner = types.SimpleNamespace(generation=7, releases=[])
  owner.release = owner.releases.append
  owner.disable = lambda _reason: None
  buffer = TiciH264DecodedBuffer(owner, 3, read_fd, 960, 540, 1024, 557056, 42)

  buffer.release()
  buffer.release()

  assert owner.releases == [3]
  with pytest.raises(OSError):
    os.fstat(read_fd)
  os.close(write_fd)


def test_hardware_h264_decoder_factory_does_not_require_tici_marker(monkeypatch, tmp_path):
  library_path = tmp_path / "libcluster_h264_decoder_bridge.so"
  library_path.touch()
  calls = []
  original_is_file = Path.is_file

  def reject_tici_marker_check(path):
    assert str(path) != "/TICI"
    return original_is_file(path)

  monkeypatch.setenv("CLUSTER_HARDWARE_H264_DECODE", "1")
  monkeypatch.setenv("CLUSTER_H264_DECODER_LIBRARY", str(library_path))
  monkeypatch.setenv("CLUSTER_H264_DECODER_DEVICE", "/dev/test-vidc")
  monkeypatch.setattr(Path, "is_file", reject_tici_marker_check)
  monkeypatch.setattr(
    cluster_h264_decoder,
    "TiciH264Decoder",
    lambda width, height, fps, **kwargs: calls.append((width, height, fps, kwargs)) or "decoder",
  )

  decoder = cluster_h264_decoder.create_tici_h264_decoder(960, 540, 30)

  assert decoder == "decoder"
  assert calls == [(960, 540, 30, {
    "library_path": library_path,
    "device_path": "/dev/test-vidc",
    "timeout_ms": 250,
    "debug": False,
  })]


def _import_cluster_autorun(monkeypatch):
  params_module = types.ModuleType("openpilot.common.params")
  params_module.Params = object
  hardware_module = types.ModuleType("openpilot.system.hardware")
  hardware_module.TICI = False
  monkeypatch.setitem(sys.modules, "openpilot.common.params", params_module)
  monkeypatch.setitem(sys.modules, "openpilot.system.hardware", hardware_module)

  module_name = "openpilot.selfdrive.carrot.cluster_autorun"
  sys.modules.pop(module_name, None)
  cluster_autorun = importlib.import_module(module_name)
  cluster_autorun._ensure_cluster_paths()
  return cluster_autorun


def test_cluster_autorun_output_gate_preserves_debug_override(monkeypatch):
  cluster_autorun = _import_cluster_autorun(monkeypatch)

  class FakeParams:
    def __init__(self, onroad, debug):
      self.onroad = onroad
      self.debug = debug

    def get_bool(self, name):
      assert name == cluster_autorun.IS_ONROAD_PARAM
      return self.onroad

    def get_int(self, name):
      assert name == cluster_autorun.HUD_DEBUG_PARAM
      return self.debug

  assert not cluster_autorun._hud_output_allowed(FakeParams(False, 0))
  assert cluster_autorun._hud_output_allowed(FakeParams(False, 1))
  assert cluster_autorun._hud_output_allowed(FakeParams(True, 0))
  assert cluster_autorun.HUD_CHECK_INTERVAL_S == 0.1


def test_cluster_autorun_restarts_without_delay_after_orientation_change(monkeypatch):
  cluster_autorun = _import_cluster_autorun(monkeypatch)
  usb_display_module = importlib.import_module("cluster_usb_display")
  values = {
    cluster_autorun.HUD_PARAM: 1,
    cluster_autorun.HUD_DEBUG_PARAM: 1,
    cluster_autorun.HUD_ENCODER_PARAM: cluster_autorun.ENCODER_HARDWARE,
    cluster_autorun.HUD_LIVE_FPS_PARAM: 1,
    cluster_autorun.HUD_ORIENTATION_PARAM: 0,
    cluster_autorun.HUD_CORE_MODE_PARAM: cluster_autorun.CORE_MODE_DEDICATED,
    cluster_autorun.HUD_PRIORITY_PARAM: 10,
  }
  runs = []

  class FakeParams:
    def get_int(self, name):
      return values[name]

    def put_bool_nonblocking(self, _name, _value):
      return None

  def run_cluster_once(*_args, **_kwargs):
    runs.append(values[cluster_autorun.HUD_ORIENTATION_PARAM])
    if len(runs) == 1:
      values[cluster_autorun.HUD_ORIENTATION_PARAM] = 2
    else:
      values[cluster_autorun.HUD_PARAM] = 0

  monkeypatch.setattr(cluster_autorun, "Params", FakeParams)
  monkeypatch.setattr(cluster_autorun, "_configure_autorun_locale", lambda: None)
  monkeypatch.setattr(cluster_autorun, "_configure_autorun_affinity", lambda: None)
  monkeypatch.setattr(cluster_autorun, "_apply_realtime_setting_env", lambda *_args: None)
  monkeypatch.setattr(cluster_autorun, "_hud_output_allowed", lambda _params: True)
  monkeypatch.setattr(cluster_autorun, "_wait_for_usbgpu_startup", lambda _params: None)
  monkeypatch.setattr(cluster_autorun, "_run_cluster_once", run_cluster_once)
  monkeypatch.setattr(
    usb_display_module,
    "find_supported_usb_product",
    lambda expected_product_id: expected_product_id,
  )
  monkeypatch.setattr(
    cluster_autorun.time,
    "sleep",
    lambda _seconds: pytest.fail("orientation restart must not wait for the retry delay"),
  )

  try:
    cluster_autorun.main()
    assert runs == [0, 2]
  finally:
    sys.modules.pop("openpilot.selfdrive.carrot.cluster_autorun", None)


def test_cluster_autorun_waits_for_first_egpu_output_before_usb_display(monkeypatch):
  cluster_autorun = _import_cluster_autorun(monkeypatch)
  clock = [0.0]
  states = [
    {
      cluster_autorun.USBGPU_LOADING_PARAM: True,
      cluster_autorun.USBGPU_ACTIVE_PARAM: True,
      cluster_autorun.USBGPU_STARTUP_FAILED_PARAM: False,
    },
    {
      cluster_autorun.USBGPU_LOADING_PARAM: False,
      cluster_autorun.USBGPU_ACTIVE_PARAM: True,
      cluster_autorun.USBGPU_STARTUP_FAILED_PARAM: False,
    },
  ]
  state_index = [0]

  class FakeParams:
    def get_bool(self, name):
      if name == cluster_autorun.USBGPU_HARDWARE_SEEN_PARAM:
        return True
      return states[state_index[0]][name]

  monkeypatch.setattr(cluster_autorun, "_usbgpu_startup_expected", lambda: True)
  monkeypatch.setattr(cluster_autorun.time, "monotonic", lambda: clock[0])

  def advance(seconds):
    clock[0] += seconds
    state_index[0] = min(state_index[0] + 1, len(states) - 1)

  monkeypatch.setattr(cluster_autorun.time, "sleep", advance)

  cluster_autorun._wait_for_usbgpu_startup(FakeParams())

  assert state_index[0] == 1
  assert clock[0] == pytest.approx(
    cluster_autorun.USBGPU_STARTUP_POLL_S + cluster_autorun.USBGPU_DISPLAY_STABILIZE_S,
  )


def test_cluster_autorun_skips_egpu_gate_for_unrelated_usb_display(monkeypatch):
  cluster_autorun = _import_cluster_autorun(monkeypatch)

  class FakeParams:
    def get_bool(self, name):
      assert name == cluster_autorun.USBGPU_HARDWARE_SEEN_PARAM
      return False

  monkeypatch.setattr(cluster_autorun, "_usbgpu_startup_expected", lambda: False)
  monkeypatch.setattr(
    cluster_autorun.time,
    "sleep",
    lambda _seconds: pytest.fail("non-eGPU systems must not wait for the startup gate"),
  )

  cluster_autorun._wait_for_usbgpu_startup(FakeParams())


def test_cluster_autorun_releases_usb_display_after_egpu_fallback(monkeypatch):
  cluster_autorun = _import_cluster_autorun(monkeypatch)

  class FakeParams:
    def get_bool(self, name):
      return {
        cluster_autorun.USBGPU_HARDWARE_SEEN_PARAM: True,
        cluster_autorun.USBGPU_LOADING_PARAM: False,
        cluster_autorun.USBGPU_ACTIVE_PARAM: False,
        cluster_autorun.USBGPU_STARTUP_FAILED_PARAM: True,
      }[name]

  monkeypatch.setattr(cluster_autorun, "_usbgpu_startup_expected", lambda: True)
  monkeypatch.setattr(
    cluster_autorun.time,
    "sleep",
    lambda _seconds: pytest.fail("failed eGPU startup must release the USB display gate"),
  )

  cluster_autorun._wait_for_usbgpu_startup(FakeParams())


def _new_h264_pipeline() -> H264UsbPipeline:
  return H264UsbPipeline(
    usb_display=types.SimpleNamespace(),
    width=400,
    height=400,
    encoder_align=1,
    fps=10,
    bitrate="1m",
    gop=10,
    backend="native",
    library_path="",
    ffmpeg_path="ffmpeg",
    ffmpeg_encoder="libx264",
    device_path="",
    input_format="nv12",
    slice_max_bytes=4096,
    rate_control="vbr-cfr",
    realtime_priority=False,
    requested_chunk_size=0,
    wait_for_ack=False,
    soft_ack=False,
    dump_path="",
    debug=False,
  )


def test_cluster_autorun_uses_selected_affinity(monkeypatch):
  module_name = "openpilot.selfdrive.carrot.cluster_autorun"
  cluster_autorun = _import_cluster_autorun(monkeypatch)
  affinity_calls = []
  monkeypatch.setattr(cluster_autorun, "_cluster_realtime_cores", lambda: [1, 2, 3, 4])
  monkeypatch.setattr(
    cluster_autorun,
    "_set_current_process_affinity",
    lambda cores: affinity_calls.append(cores) or cores,
  )

  try:
    cluster_autorun._configure_autorun_affinity()
    assert affinity_calls == [[1, 2, 3, 4]]
  finally:
    sys.modules.pop(module_name, None)


def test_cluster_hud_mode_two_has_no_usb_product_mapping():
  assert product_id_for_hud_mode(1) == 0x0092
  assert product_id_for_hud_mode(2) is None


@pytest.mark.parametrize("legacy_realtime_env", ("0", "1"))
def test_cluster_run_always_applies_core_mode_and_priority(monkeypatch, legacy_realtime_env):
  cluster_run = importlib.import_module("openpilot.selfdrive.carrot.cluster_run")
  realtime_module = types.ModuleType("openpilot.common.realtime")
  calls = []
  realtime_module.config_realtime_process = lambda cores, priority: calls.append((cores, priority))
  monkeypatch.setitem(sys.modules, "openpilot.common.realtime", realtime_module)
  monkeypatch.setenv("CLUSTER_REALTIME", legacy_realtime_env)
  monkeypatch.setattr(cluster_run, "_resolved_realtime_cores", lambda: [1, 2, 3, 4])
  monkeypatch.setattr(cluster_run, "_resolved_realtime_priority", lambda: 37)

  cluster_run.configure_cluster_scheduling()

  assert calls == [([1, 2, 3, 4], 37)]


def test_git_status_remote_disabled_never_starts_git_worker(tmp_path, monkeypatch):
  git_dir = tmp_path / ".git"
  git_dir.mkdir()
  (git_dir / "HEAD").write_text("ref: refs/heads/performance\n", encoding="utf-8")
  provider = GitBranchStatusProvider(tmp_path, remote_enabled=False)

  def fail_git(*_args, **_kwargs):
    raise AssertionError("remote-disabled provider must not run git")

  monkeypatch.setattr(provider, "_git", fail_git)

  assert provider.status().branch == "performance"
  assert provider.status().detail == ""
  assert provider._worker is None


def test_git_status_remote_enabled_starts_background_refresh(tmp_path, monkeypatch):
  git_dir = tmp_path / ".git"
  git_dir.mkdir()
  (git_dir / "HEAD").write_text("ref: refs/heads/navigation\n", encoding="utf-8")
  provider = GitBranchStatusProvider(tmp_path)
  refreshed = []
  monkeypatch.setattr(provider, "_refresh", lambda: refreshed.append(True))

  assert provider.status().branch == "navigation"
  assert provider._worker is not None
  provider._worker.join(timeout=1.0)
  assert refreshed == [True]


def test_lfa_icon_uses_active_color_rotation_and_c4_lane_overlay(monkeypatch):
  renderer = ClusterUiRenderer()
  inactive_texture = object()
  active_texture = object()
  lane_texture = object()
  renderer._lfa_texture = inactive_texture
  renderer._lfa_active_texture = active_texture
  renderer._lfa_lane_texture = lane_texture
  draws = []

  def record_draw(texture, center_x, bottom_y, width, height, tint, alpha=None, rotation_deg=0.0):
    draws.append((texture, center_x, bottom_y, width, height, tint, alpha, rotation_deg))
    return texture is not None

  monkeypatch.setattr(renderer, "_draw_bottom_aligned_texture_icon", record_draw)
  state = replace(
    standby_state(),
    lfa_active=True,
    steering_angle_deg=23.5,
    active_lane_line=True,
  )

  renderer._draw_lfa_status_icon(state, 100.0)

  assert draws[0][0] is active_texture
  assert draws[0][7] == -23.5
  assert draws[1][0] is lane_texture
  assert draws[1][3] == cluster_renderer.LFA_STATUS_ICON_SIZE * 2.0
  assert draws[1][7] == 0.0


def test_lane_floor_keeps_blinking_after_physical_signal_ends(monkeypatch):
  renderer = ClusterUiRenderer()
  times = iter((100.0, 100.75, 101.0))
  monkeypatch.setattr(cluster_renderer.time, "perf_counter", lambda: next(times))
  state = replace(
    standby_state(),
    lane_change="left",
    lane_change_phase="changing",
    highlight_lane="left",
    highlight_lane_offset=-1.0,
    left_signal=False,
  )

  assert renderer._highlight_lane_lit(state, (False, False))
  assert not renderer._highlight_lane_lit(state, (False, False))
  assert renderer._highlight_lane_lit(state, (False, False))

  idle = replace(state, lane_change=None, lane_change_phase="idle", highlight_lane=None, highlight_lane_offset=None)
  renderer._highlight_lane_lit(idle, (False, False))
  assert renderer._lane_highlight_side is None
  assert renderer._lane_highlight_started_at is None


def test_navi_screen_keeps_bottom_status_footer(monkeypatch):
  renderer = ClusterUiRenderer(screen_mode=cluster_renderer.CLUSTER_SCREEN_MODE_NAVI)
  calls = []
  monkeypatch.setattr(cluster_renderer.rl, "rl_push_matrix", lambda: None)
  monkeypatch.setattr(cluster_renderer.rl, "rl_scalef", lambda *_args: None)
  monkeypatch.setattr(cluster_renderer.rl, "rl_pop_matrix", lambda: None)
  monkeypatch.setattr(renderer, "_draw_navi_dashboard", lambda _state: calls.append("navi"))
  monkeypatch.setattr(renderer, "_draw_status_footer", lambda _state: calls.append("footer"))

  renderer._draw_hud(object(), (False, False))

  assert calls == ["navi", "footer"]


def test_renderer_fonts_use_bilinear_filter_without_mipmaps(tmp_path, monkeypatch):
  font_path = tmp_path / "font.ttf"
  font_path.touch()
  renderer = ClusterUiRenderer()
  loaded_fonts = [
    types.SimpleNamespace(texture=types.SimpleNamespace(id=1)),
    types.SimpleNamespace(texture=types.SimpleNamespace(id=2)),
  ]
  load_args = []
  filters = []

  renderer.screen_mode = cluster_renderer.CLUSTER_SCREEN_MODE_NAVI
  monkeypatch.setattr(renderer, "_font_candidates", lambda: [font_path])
  monkeypatch.setattr(cluster_renderer.rl, "set_trace_log_level", lambda _level: None)
  monkeypatch.setattr(
    cluster_renderer.rl,
    "load_font_ex",
    lambda *args: load_args.append(args) or loaded_fonts.pop(0),
  )
  monkeypatch.setattr(
    cluster_renderer.rl,
    "gen_texture_mipmaps",
    lambda _texture: (_ for _ in ()).throw(AssertionError("HUD fonts must not generate mipmaps")),
  )
  monkeypatch.setattr(
    cluster_renderer.rl,
    "set_texture_filter",
    lambda texture, filter_mode: filters.append((texture.id, filter_mode)),
  )

  renderer._load_font()
  renderer._load_korean_font()

  assert filters == [
    (1, cluster_renderer.rl.TextureFilter.TEXTURE_FILTER_BILINEAR),
    (2, cluster_renderer.rl.TextureFilter.TEXTURE_FILTER_BILINEAR),
  ]
  assert load_args[0][1:] == (160, None, 0)
  assert load_args[1][1] == cluster_renderer.KOREAN_FONT_BASE_SIZE
  assert load_args[1][3] == len(renderer._navi_font_codepoints())


def test_stroked_text_texture_rasterizes_once_and_reuses_cached_draw(monkeypatch):
  renderer = object.__new__(ClusterUiRenderer)
  renderer._font = object()
  renderer._korean_font = None
  renderer._stroked_text_texture_cache = OrderedDict()
  renderer._pending_stroked_text_textures = OrderedDict()
  renderer._stroked_text_texture_cache_enabled = True
  renderer._text_measure_cache = {}
  renderer.profile_enabled = False
  renderer._profile_samples = []
  image = object()
  texture = types.SimpleNamespace(id=7)
  image_draws = []
  texture_loads = []
  texture_draws = []

  monkeypatch.setattr(cluster_renderer, "rl_color", lambda color: color)
  monkeypatch.setattr(cluster_renderer.rl, "measure_text_ex", lambda *_args: types.SimpleNamespace(x=50, y=20))
  monkeypatch.setattr(cluster_renderer.rl, "gen_image_color", lambda *_args: image)
  monkeypatch.setattr(
    cluster_renderer.rl,
    "image_draw_text_ex",
    lambda *args: image_draws.append(args),
  )
  monkeypatch.setattr(
    cluster_renderer.rl,
    "load_texture_from_image",
    lambda source: texture_loads.append(source) or texture,
  )
  monkeypatch.setattr(cluster_renderer.rl, "is_texture_valid", lambda _texture: True)
  monkeypatch.setattr(cluster_renderer.rl, "set_texture_filter", lambda *_args: None)
  monkeypatch.setattr(cluster_renderer.rl, "unload_image", lambda *_args: None)
  monkeypatch.setattr(
    cluster_renderer.rl,
    "draw_texture_pro",
    lambda *args: texture_draws.append(args),
  )

  direct_draws = []
  monkeypatch.setattr(renderer, "_draw_text", lambda *args: direct_draws.append(args))

  renderer._draw_text_with_stroke("NAVI", 100, 50, 27, (0, 255, 0), (0, 0, 0), 2, cache=True)

  assert len(direct_draws) == 9
  assert image_draws == []
  assert texture_loads == []
  assert len(renderer._pending_stroked_text_textures) == 1

  renderer._flush_pending_stroked_text_textures()
  renderer._draw_text_with_stroke("NAVI", 100, 50, 27, (0, 255, 0), (0, 0, 0), 2, cache=True)

  assert len(image_draws) == 9
  assert texture_loads == [image]
  assert len(texture_draws) == 1
  assert len(renderer._stroked_text_texture_cache) == 1


def test_fast_changing_stroked_text_can_bypass_texture_cache(monkeypatch):
  renderer = object.__new__(ClusterUiRenderer)
  renderer._stroked_text_texture_cache_enabled = True
  direct_draws = []
  monkeypatch.setattr(
    renderer,
    "_draw_text",
    lambda *args: direct_draws.append(args),
  )
  monkeypatch.setattr(
    renderer,
    "_stroked_text_texture",
    lambda *_args: pytest.fail("dynamic text must not create a cached texture"),
  )

  renderer._draw_text_with_stroke(
    "+0.17",
    100,
    50,
    17,
    (255, 255, 255),
    (0, 0, 0),
    2,
    cache=False,
  )

  assert len(direct_draws) == 9


def test_raw_stroked_text_preserves_draw_order_positions_and_colors(monkeypatch):
  renderer = object.__new__(ClusterUiRenderer)
  renderer._raw_stroked_text_enabled = True
  renderer._font = object()
  renderer._korean_font = None
  renderer._text_measure_cache = {}
  calls = []

  class Position:
    def __init__(self, x, y):
      self.x = x
      self.y = y

  def draw_text_ex(font, text, position, size, spacing, color):
    calls.append((font, text, position.x, position.y, size, spacing, color))

  renderer._raw_draw_text_ex = draw_text_ex
  monkeypatch.setattr(cluster_renderer.rl, "Vector2", Position)
  monkeypatch.setattr(cluster_renderer, "rl_color", lambda color: color)
  monkeypatch.setattr(renderer, "_measure_text", lambda *_args: (40.0, 20.0))
  monkeypatch.setattr(
    renderer,
    "_draw_text",
    lambda *_args: pytest.fail("raw stroked-text path must bypass the pyray wrapper"),
  )

  renderer._draw_text_with_stroke(
    "NAVI",
    100.0,
    50.0,
    25.0,
    (0, 255, 0),
    (0, 0, 0),
    2,
    anchor="center",
  )

  assert [(call[2], call[3]) for call in calls] == [
    (78.0, 40.0),
    (82.0, 40.0),
    (80.0, 38.0),
    (80.0, 42.0),
    (78.0, 38.0),
    (82.0, 38.0),
    (78.0, 42.0),
    (82.0, 42.0),
    (80.0, 40.0),
  ]
  assert [call[6] for call in calls] == [(0, 0, 0)] * 8 + [(0, 255, 0)]
  assert all(call[1] == b"NAVI" for call in calls)
  assert all(call[4:6] == (25.0, 1.0) for call in calls)


def test_cluster_autorun_falls_back_only_for_h264_initialization(monkeypatch):
  cluster_autorun = _import_cluster_autorun(monkeypatch)
  carrot_package = importlib.import_module("selfdrive.carrot")
  cluster_run = types.ModuleType("selfdrive.carrot.cluster_run")
  calls = []

  def run_cluster(*, exit_on_error):
    assert exit_on_error is False
    calls.append(sys.argv[:])
    if len(calls) == 1:
      raise H264PipelineInitializationError("native initialization failed")

  cluster_run.main = run_cluster
  monkeypatch.setattr(carrot_package, "cluster_run", cluster_run, raising=False)
  monkeypatch.setitem(sys.modules, "selfdrive.carrot.cluster_run", cluster_run)
  monkeypatch.setattr(
    cluster_autorun,
    "_encoder_sequence",
    lambda _mode: [cluster_autorun.ENCODER_HARDWARE, cluster_autorun.ENCODER_SOFTWARE],
  )

  cluster_autorun._run_cluster_once(
    hud_mode=0,
    encoder_mode=cluster_autorun.ENCODER_AUTO,
    core_mode=0,
    priority=10,
  )

  assert calls[0][calls[0].index("--usb-h264-backend") + 1] == "native"
  assert calls[1][calls[1].index("--usb-h264-backend") + 1] == "ffmpeg"


def test_h264_start_classifies_only_initialization_errors(monkeypatch):
  pipeline = _new_h264_pipeline()

  def fail_start():
    raise RuntimeError("device open failed")

  monkeypatch.setattr(pipeline, "_start_requested_backend", fail_start)

  with pytest.raises(H264PipelineInitializationError, match="native pipeline initialization failed"):
    pipeline.start()


def test_cluster_autorun_leaves_navi_server_owned_by_standalone_process(monkeypatch):
  cluster_autorun = _import_cluster_autorun(monkeypatch)

  args = cluster_autorun._cluster_args(
    hud_mode=0,
    configured_encoder_mode=cluster_autorun.ENCODER_AUTO,
    active_encoder_mode=cluster_autorun.ENCODER_HARDWARE,
    core_mode=0,
    priority=10,
  )

  assert "--navi-overlay" not in args
  assert "--navi-publish-cereal" not in args
  assert args[args.index("--output") + 1] == "usb"
  assert args[args.index("--usb-h264-backend") + 1] == "native"


def test_cluster_autorun_caps_h264_upload_rate_while_egpu_is_active(monkeypatch):
  cluster_autorun = _import_cluster_autorun(monkeypatch)

  args = cluster_autorun._cluster_args(
    hud_mode=0,
    configured_encoder_mode=cluster_autorun.ENCODER_AUTO,
    active_encoder_mode=cluster_autorun.ENCODER_HARDWARE,
    core_mode=0,
    priority=10,
    usbgpu_active=True,
  )

  expected = str(cluster_autorun.USBGPU_DISPLAY_FPS)
  assert args[args.index("--fps") + 1] == expected
  assert args[args.index("--usb-display-fps") + 1] == expected


def test_cluster_autorun_does_not_fallback_after_runtime_failure(monkeypatch):
  cluster_autorun = _import_cluster_autorun(monkeypatch)
  carrot_package = importlib.import_module("selfdrive.carrot")
  cluster_run = types.ModuleType("selfdrive.carrot.cluster_run")
  calls = []

  def run_cluster(*, exit_on_error):
    assert exit_on_error is False
    calls.append(sys.argv[:])
    raise RuntimeError("render loop failed")

  cluster_run.main = run_cluster
  monkeypatch.setattr(carrot_package, "cluster_run", cluster_run, raising=False)
  monkeypatch.setitem(sys.modules, "selfdrive.carrot.cluster_run", cluster_run)
  monkeypatch.setattr(
    cluster_autorun,
    "_encoder_sequence",
    lambda _mode: [cluster_autorun.ENCODER_HARDWARE, cluster_autorun.ENCODER_SOFTWARE],
  )

  with pytest.raises(RuntimeError, match="render loop failed"):
    cluster_autorun._run_cluster_once(
      hud_mode=0,
      encoder_mode=cluster_autorun.ENCODER_AUTO,
      core_mode=0,
      priority=10,
    )

  assert len(calls) == 1


def test_native_h264_queue_keeps_config_keyframe_and_latest_access_unit():
  pipeline = _new_h264_pipeline()
  pipeline._packet_queue = queue.Queue(maxsize=NATIVE_ACCESS_UNIT_QUEUE_MAX)

  pipeline._enqueue_native_access_unit([b"config"], codec_config=True, keyframe=False)
  pipeline._enqueue_native_access_unit([b"key-1a", b"key-1b"], codec_config=False, keyframe=True)
  pipeline._enqueue_native_access_unit([b"old"], codec_config=False, keyframe=False)
  pipeline._enqueue_native_access_unit([b"latest"], codec_config=False, keyframe=False)

  pending = []
  while not pipeline._packet_queue.empty():
    pending.append(pipeline._packet_queue.get_nowait())

  assert [unit.chunks for unit in pending] == [
    (b"config",),
    (b"key-1a", b"key-1b"),
    (b"latest",),
  ]
  assert pipeline._debug_dropped_access_units == 1
  assert pipeline._debug_dropped_chunks == 1


def test_renderer_rgba_upload_reuses_frame_buffer(monkeypatch):
  renderer = object.__new__(ClusterUiRenderer)
  renderer._navi_media_textures = {}
  image = types.SimpleNamespace()
  texture = types.SimpleNamespace(id=1, width=1, height=1)
  buffer_calls = []
  cast_calls = []
  upload_calls = []
  buffer_pointer = object()
  void_pointer = object()

  monkeypatch.setattr(cluster_renderer, "rl_color", lambda _color: object())
  monkeypatch.setattr(cluster_renderer.rl, "gen_image_color", lambda *_args: image)
  monkeypatch.setattr(cluster_renderer.rl, "load_texture_from_image", lambda _image: texture)
  monkeypatch.setattr(cluster_renderer.rl, "unload_image", lambda _image: None)
  monkeypatch.setattr(cluster_renderer.rl, "is_texture_valid", lambda _texture: True)
  monkeypatch.setattr(cluster_renderer.rl, "set_texture_filter", lambda *_args: None)
  monkeypatch.setattr(
    cluster_renderer.rl,
    "ffi",
    types.SimpleNamespace(
      from_buffer=lambda ctype, data: buffer_calls.append((ctype, data)) or buffer_pointer,
      cast=lambda ctype, pointer: cast_calls.append((ctype, pointer)) or void_pointer,
    ),
  )
  monkeypatch.setattr(
    cluster_renderer.rl,
    "update_texture",
    lambda target, pixels: upload_calls.append((target, pixels)),
  )
  frame_data = b"rgba"

  result = renderer._navi_media_texture_for(
    cluster_renderer.NaviMediaFrame("render:map_main", 1, True, "image/rgba", 1, 1, frame_data)
  )

  assert result is texture
  assert buffer_calls == [("const unsigned char[]", frame_data)]
  assert cast_calls == [("void *", buffer_pointer)]
  assert upload_calls == [(texture, void_pointer)]


def test_renderer_yuv420_upload_reuses_plane_textures(monkeypatch):
  renderer = object.__new__(ClusterUiRenderer)
  renderer._navi_media_textures = {}
  renderer.profile_enabled = False
  renderer._profile_samples = []
  created = []
  filters = []
  uploads = []
  ffi_arrays = []

  def load_texture(image):
    texture = types.SimpleNamespace(id=len(created) + 1, width=image.width, height=image.height)
    created.append(texture)
    return texture

  monkeypatch.setattr(
    cluster_renderer.rl,
    "Image",
    lambda _data, width, height, _mips, _format: types.SimpleNamespace(width=width, height=height),
  )
  monkeypatch.setattr(cluster_renderer.rl, "load_texture_from_image", load_texture)
  monkeypatch.setattr(cluster_renderer.rl, "is_texture_valid", lambda _texture: True)
  monkeypatch.setattr(
    cluster_renderer.rl,
    "set_texture_filter",
    lambda texture, texture_filter: filters.append((texture.id, texture_filter)),
  )
  monkeypatch.setattr(cluster_renderer.rl, "unload_texture", lambda *_args: None)
  monkeypatch.setattr(
    cluster_renderer.rl,
    "ffi",
    types.SimpleNamespace(
      new=lambda _ctype, values: ffi_arrays.append(tuple(values)) or tuple(values),
      from_buffer=lambda _ctype, data: data,
      cast=lambda _ctype, pointer: pointer,
    ),
  )
  monkeypatch.setattr(
    cluster_renderer.rl,
    "update_texture",
    lambda texture, pixels: uploads.append((texture.id, pixels)),
  )

  first = cluster_renderer.NaviMediaFrame(
    "render:map_main",
    1,
    True,
    "image/yuv420p",
    4,
    2,
    plane_data=(b"y" * 16, b"u" * 4, b"v" * 4),
    plane_strides=(8, 4, 4),
  )
  second = replace(first, sequence=2, plane_data=(b"Y" * 16, b"U" * 4, b"V" * 4))

  assert renderer._navi_media_texture_for(first) is created[0]
  assert renderer._navi_media_texture_for(second) is created[0]
  assert [(texture.width, texture.height) for texture in created] == [(8, 2), (4, 1), (4, 1)]
  assert filters == [
    (1, cluster_renderer.rl.TextureFilter.TEXTURE_FILTER_BILINEAR),
    (2, cluster_renderer.rl.TextureFilter.TEXTURE_FILTER_POINT),
    (3, cluster_renderer.rl.TextureFilter.TEXTURE_FILTER_POINT),
  ]
  assert [texture_id for texture_id, _pixels in uploads] == [1, 2, 3, 1, 2, 3]
  assert ffi_arrays == [(1.0, 1.0), (1.0, 1.0)]
  cached = renderer._navi_media_textures["render:map_main"]
  assert cached.sequence == 2
  assert cached.size == (4, 2)


def test_renderer_binds_hardware_nv12_dmabuf_without_pixel_upload(monkeypatch):
  renderer = object.__new__(ClusterUiRenderer)
  renderer._navi_media_textures = {}
  renderer._navi_egl_images = OrderedDict()
  renderer._navi_external_shader = object()
  renderer.profile_enabled = False
  renderer._profile_samples = []
  texture = types.SimpleNamespace(id=7, width=1, height=1)
  image = object()
  created_images = []
  bindings = []
  destroyed_images = []
  failed = []

  egl_module = types.ModuleType("openpilot.system.ui.lib.egl")
  egl_module.init_egl = lambda: True
  egl_module.create_egl_image = (
    lambda width, height, stride, fd, uv_offset:
      created_images.append((width, height, stride, fd, uv_offset)) or object()
  )
  egl_module.bind_egl_image_to_texture = lambda texture_id, egl_image: bindings.append((texture_id, egl_image))
  egl_module.destroy_egl_image = destroyed_images.append
  monkeypatch.setitem(sys.modules, "openpilot.system.ui.lib.egl", egl_module)
  monkeypatch.setattr(cluster_renderer.rl, "gen_image_color", lambda *_args: image)
  monkeypatch.setattr(cluster_renderer.rl, "load_texture_from_image", lambda _image: texture)
  monkeypatch.setattr(cluster_renderer.rl, "unload_image", lambda _image: None)
  monkeypatch.setattr(cluster_renderer.rl, "is_texture_valid", lambda _texture: True)
  monkeypatch.setattr(cluster_renderer.rl, "set_texture_filter", lambda *_args: None)
  monkeypatch.setattr(cluster_renderer.rl, "unload_texture", lambda *_args: None)

  first_buffer = types.SimpleNamespace(
    fd=31,
    stride=1024,
    uv_offset=557056,
    token=(3, 0),
    mark_egl_import_failed=lambda reason: failed.append(reason),
  )
  second_buffer = types.SimpleNamespace(
    fd=32,
    stride=1024,
    uv_offset=557056,
    token=(3, 1),
    mark_egl_import_failed=lambda reason: failed.append(reason),
  )
  third_buffer = types.SimpleNamespace(
    fd=33,
    stride=1024,
    uv_offset=557056,
    token=(4, 0),
    mark_egl_import_failed=lambda reason: failed.append(reason),
  )
  first = cluster_renderer.NaviMediaFrame(
    "render:map_main", 1, True, "video/nv12-dmabuf", 960, 540, hardware_buffer=first_buffer
  )
  second = replace(first, sequence=2, hardware_buffer=second_buffer)
  third = replace(first, sequence=3, hardware_buffer=third_buffer)

  assert renderer._navi_media_texture_for(first) is texture
  assert renderer._navi_media_texture_for(second) is texture
  previous_generation_images = tuple(renderer._navi_egl_images.values())
  assert renderer._navi_media_texture_for(third) is texture
  assert created_images == [
    (960, 540, 1024, 31, 557056),
    (960, 540, 1024, 32, 557056),
    (960, 540, 1024, 33, 557056),
  ]
  assert len(bindings) == 3
  assert destroyed_images == list(previous_generation_images)
  assert failed == []
  assert renderer._navi_media_textures["render:map_main"].hardware_token == (4, 0)
  assert (texture.width, texture.height) == (960, 540)

  monkeypatch.setattr(cluster_renderer.rl, "begin_shader_mode", lambda _shader: None)
  monkeypatch.setattr(cluster_renderer.rl, "draw_texture_pro", lambda *_args: None)
  monkeypatch.setattr(cluster_renderer.rl, "end_shader_mode", lambda: None)
  cached = renderer._navi_media_textures["render:map_main"]
  rect = cluster_renderer.rl.Rectangle(0.0, 0.0, 960.0, 540.0)
  renderer._draw_cached_navi_media(cached, rect, rect)
  renderer._draw_cached_navi_media(cached, rect, rect)
  assert bindings[-2:] == [(7, renderer._navi_egl_images[(4, 0)])] * 2


def test_native_h264_direct_input_lease_submits_or_cancels():
  pipeline = _new_h264_pipeline()
  calls = []

  def acquire(_handle, address_out, size_out, index_out, _callback, _opaque):
    ctypes.cast(address_out, ctypes.POINTER(ctypes.c_void_p))[0] = ctypes.c_void_p(0x12340000)
    ctypes.cast(size_out, ctypes.POINTER(ctypes.c_size_t))[0] = 8192
    ctypes.cast(index_out, ctypes.POINTER(ctypes.c_uint32))[0] = 3
    calls.append("acquire")
    return 0

  def submit(_handle, index, timestamp_us, _callback, _opaque):
    calls.append(("submit", index, timestamp_us))
    return 0

  def cancel(_handle, index):
    calls.append(("cancel", index))
    return 0

  pipeline._native_lib = types.SimpleNamespace(
    cluster_h264_encoder_bridge_acquire_nv12_input=acquire,
    cluster_h264_encoder_bridge_submit_nv12_input=submit,
    cluster_h264_encoder_bridge_cancel_nv12_input=cancel,
  )
  pipeline._native_handle = 1
  pipeline._native_callback = object()
  pipeline._native_has_direct_input = True
  pipeline._native_input_bytesused = 8192

  with pipeline.native_nv12_input_buffer() as input_buffer:
    assert (input_buffer.address, input_buffer.size, input_buffer.index) == (0x12340000, 8192, 3)
    pipeline.submit_native_nv12_input(input_buffer)

  assert calls == ["acquire", ("submit", 3, 0)]
  assert pipeline._native_frame_index == 1

  with pipeline.native_nv12_input_buffer():
    pass

  assert calls[-2:] == ["acquire", ("cancel", 3)]
  assert pipeline._native_frame_index == 1


def test_native_h264_direct_input_lease_exposes_dmabuf_fd():
  pipeline = _new_h264_pipeline()
  calls = []

  def acquire_dmabuf(_handle, address_out, size_out, index_out, fd_out, _callback, _opaque):
    ctypes.cast(address_out, ctypes.POINTER(ctypes.c_void_p))[0] = ctypes.c_void_p(0x56780000)
    ctypes.cast(size_out, ctypes.POINTER(ctypes.c_size_t))[0] = 16384
    ctypes.cast(index_out, ctypes.POINTER(ctypes.c_uint32))[0] = 5
    ctypes.cast(fd_out, ctypes.POINTER(ctypes.c_int))[0] = 42
    calls.append("acquire_dmabuf")
    return 0

  def cancel(_handle, index):
    calls.append(("cancel", index))
    return 0

  def submit_dmabuf(_handle, index, timestamp_us, _callback, _opaque):
    calls.append(("submit_dmabuf", index, timestamp_us))
    return 0

  pipeline._native_lib = types.SimpleNamespace(
    cluster_h264_encoder_bridge_acquire_nv12_input_dmabuf=acquire_dmabuf,
    cluster_h264_encoder_bridge_submit_nv12_input_dmabuf=submit_dmabuf,
    cluster_h264_encoder_bridge_cancel_nv12_input=cancel,
  )
  pipeline._native_handle = 1
  pipeline._native_callback = object()
  pipeline._native_has_direct_input = True
  pipeline._native_has_dmabuf_input = True
  pipeline._native_input_bytesused = 16384

  with pipeline.native_nv12_input_buffer() as input_buffer:
    assert (
      input_buffer.address,
      input_buffer.size,
      input_buffer.index,
      input_buffer.dmabuf_fd,
    ) == (0x56780000, 16384, 5, 42)
    pipeline.submit_native_nv12_dmabuf_input(input_buffer)

  assert calls == ["acquire_dmabuf", ("submit_dmabuf", 5, 0)]


def test_gles_direct_readback_restores_gl_state():
  calls = []
  errors = iter((0, 0))

  class FakeGles:
    def glGetIntegerv(self, name, value_out):
      if name == cluster_gles_readback.GL_FRAMEBUFFER_BINDING:
        value = 17
      elif name == cluster_gles_readback.GL_PIXEL_PACK_BUFFER_BINDING:
        value = 19
      else:
        value = 8
      ctypes.cast(value_out, ctypes.POINTER(ctypes.c_int))[0] = value

    def glBindFramebuffer(self, target, framebuffer):
      calls.append(("framebuffer", target, framebuffer))

    def glPixelStorei(self, name, value):
      calls.append(("pack", name, value))

    def glBindBuffer(self, target, buffer):
      calls.append(("buffer", target, buffer))

    def glReadPixels(self, x, y, width, height, pixel_format, pixel_type, destination):
      calls.append(("read", x, y, width, height, pixel_format, pixel_type, destination.value))

    def glGetError(self):
      return next(errors)

  readback = object.__new__(cluster_gles_readback.GlesDirectReadback)
  readback._gles = FakeGles()
  readback.read_rgba(23, 16, 8, 0x12340000, 16 * 8 * 4)

  assert calls == [
    ("framebuffer", cluster_gles_readback.GL_FRAMEBUFFER, 23),
    ("buffer", cluster_gles_readback.GL_PIXEL_PACK_BUFFER, 0),
    ("pack", cluster_gles_readback.GL_PACK_ALIGNMENT, 4),
    (
      "read",
      0,
      0,
      16,
      8,
      cluster_gles_readback.GL_RGBA,
      cluster_gles_readback.GL_UNSIGNED_BYTE,
      0x12340000,
    ),
    ("pack", cluster_gles_readback.GL_PACK_ALIGNMENT, 8),
    ("buffer", cluster_gles_readback.GL_PIXEL_PACK_BUFFER, 19),
    ("framebuffer", cluster_gles_readback.GL_FRAMEBUFFER, 17),
  ]


def test_gles_async_readback_uses_nonblocking_fence_and_copies_ready_pbo():
  calls = []
  errors = iter((0, 0, 0, 0, 0, 0))
  waits = iter((cluster_gles_readback.GL_TIMEOUT_EXPIRED, cluster_gles_readback.GL_ALREADY_SIGNALED))
  source = (ctypes.c_ubyte * 16)(*range(16))
  destination = (ctypes.c_ubyte * 16)()

  class FakeGles:
    def glGetIntegerv(self, name, value_out):
      if name == cluster_gles_readback.GL_FRAMEBUFFER_BINDING:
        value = 17
      elif name == cluster_gles_readback.GL_PIXEL_PACK_BUFFER_BINDING:
        value = 19
      else:
        value = 8
      ctypes.cast(value_out, ctypes.POINTER(ctypes.c_int))[0] = value

    def glGetError(self):
      return next(errors)

    def glGenBuffers(self, count, buffers):
      for index in range(count):
        buffers[index] = 31 + index

    def glDeleteBuffers(self, count, buffers):
      calls.append(("delete_buffers", tuple(buffers[index] for index in range(count))))

    def glBindBuffer(self, target, buffer):
      calls.append(("buffer", target, buffer))

    def glBufferData(self, target, size, _data, usage):
      calls.append(("allocate", target, size, usage))

    def glBindFramebuffer(self, target, framebuffer):
      calls.append(("framebuffer", target, framebuffer))

    def glPixelStorei(self, name, value):
      calls.append(("pack", name, value))

    def glReadPixels(self, _x, _y, width, height, _format, _type, destination_pointer):
      calls.append(("read_pbo", width, height, destination_pointer.value))

    def glFenceSync(self, condition, flags):
      calls.append(("fence", condition, flags))
      return 0x1234

    def glFlush(self):
      calls.append(("flush",))

    def glClientWaitSync(self, fence, flags, timeout):
      calls.append(("wait", fence.value, flags, timeout))
      return next(waits)

    def glMapBufferRange(self, target, offset, length, access):
      calls.append(("map", target, offset, length, access))
      return ctypes.addressof(source)

    def glUnmapBuffer(self, target):
      calls.append(("unmap", target))
      return True

    def glDeleteSync(self, fence):
      calls.append(("delete_fence", fence.value))

  readback = object.__new__(cluster_gles_readback.GlesDirectReadback)
  readback._gles = FakeGles()
  readback._async_supported = True
  readback._pbo_slots = []
  readback._pbo_free = deque()
  readback._pbo_pending = deque()
  readback._pbo_byte_count = 0

  assert readback.enqueue_rgba(23, 2, 2)
  assert not readback.async_ready()
  assert readback.copy_ready(ctypes.addressof(destination), ctypes.sizeof(destination))
  assert bytes(destination) == bytes(source)
  assert not readback.async_ready()
  assert len(readback._pbo_free) == cluster_gles_readback.ASYNC_READBACK_RING_SIZE

  readback.close()
  assert ("read_pbo", 2, 2, None) in calls
  assert ("map", cluster_gles_readback.GL_PIXEL_PACK_BUFFER, 0, 16, cluster_gles_readback.GL_MAP_READ_BIT) in calls
  assert ("delete_fence", 0x1234) in calls
  assert ("delete_buffers", (31, 32, 33)) in calls
