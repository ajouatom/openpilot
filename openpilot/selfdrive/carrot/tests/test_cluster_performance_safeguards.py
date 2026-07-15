import ctypes
import importlib
from pathlib import Path
import queue
import sys
import types

import pytest


CLUSTER_DIR = Path(__file__).resolve().parents[1] / "cluster"
sys.path.insert(0, str(CLUSTER_DIR))

from cluster_git_status import GitBranchStatusProvider
import cluster_gles_readback
from cluster_h264_pipeline import (
  H264PipelineInitializationError,
  H264UsbPipeline,
  NATIVE_ACCESS_UNIT_QUEUE_MAX,
)
import cluster_renderer
from cluster_renderer import ClusterUiRenderer


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


def test_cluster_autorun_defaults_to_normal_scheduler_and_keeps_affinity(monkeypatch):
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
    assert cluster_autorun.AUTORUN_DEFAULT_ENV["CLUSTER_REALTIME"] == "0"
    cluster_autorun._configure_autorun_affinity()
    assert affinity_calls == [[1, 2, 3, 4]]
  finally:
    sys.modules.pop(module_name, None)


def test_cluster_run_explicitly_drops_realtime_and_keeps_affinity(monkeypatch):
  cluster_run = importlib.import_module("openpilot.selfdrive.carrot.cluster_run")
  realtime_module = types.ModuleType("openpilot.common.realtime")
  calls = []
  realtime_module.config_realtime_process = lambda *_args: calls.append("realtime")
  realtime_module.drop_realtime = lambda: calls.append("drop")
  realtime_module.set_core_affinity = lambda cores: calls.append(("affinity", cores))
  monkeypatch.setitem(sys.modules, "openpilot.common.realtime", realtime_module)
  monkeypatch.setenv("CLUSTER_REALTIME", "0")
  monkeypatch.setattr(cluster_run, "_resolved_realtime_cores", lambda: [1, 2, 3, 4])
  monkeypatch.setattr(cluster_run.gc, "enable", lambda: calls.append("gc_enable"))

  cluster_run.configure_cluster_realtime()

  assert calls == ["gc_enable", "drop", ("affinity", [1, 2, 3, 4])]


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
