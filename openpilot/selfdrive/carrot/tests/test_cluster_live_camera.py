import sys
from pathlib import Path
from types import ModuleType, SimpleNamespace


if sys.platform == "win32":
  swaglog = ModuleType("openpilot.common.swaglog")
  swaglog.cloudlog = SimpleNamespace(
    error=lambda *args, **kwargs: None,
    warning=lambda *args, **kwargs: None,
    info=lambda *args, **kwargs: None,
    exception=lambda *args, **kwargs: None,
  )
  sys.modules["openpilot.common.swaglog"] = swaglog


CLUSTER_DIR = Path(__file__).resolve().parents[1] / "cluster"
sys.path.insert(0, str(CLUSTER_DIR))

import cluster_live_camera
from cluster_live_camera import LiveRoadCamera
from openpilot.system.ui.lib import egl
from openpilot.system.ui.lib.egl import egl_error_text


def test_egl_error_text_names_not_initialized():
  assert egl_error_text(12289) == "0x3001 EGL_NOT_INITIALIZED"


def test_egl_image_retries_after_display_reinitialization(monkeypatch):
  egl_image = object()
  fake_ffi = SimpleNamespace(NULL=None, new=lambda *args: object())
  fake_egl = SimpleNamespace(
    initialized=True,
    ffi=fake_ffi,
    display=object(),
    NO_CONTEXT=None,
    NO_IMAGE_KHR=None,
    create_image_khr=lambda *args: next(images),
    get_error=lambda: egl.EGL_NOT_INITIALIZED,
  )
  images = iter((None, egl_image))
  reinitializations = []

  monkeypatch.setattr(egl, "_egl", fake_egl)
  monkeypatch.setattr(egl.os, "dup", lambda fd: 101)
  monkeypatch.setattr(egl, "_initialize_current_display", lambda force=False: reinitializations.append(force) or True)

  result = egl.create_egl_image(1928, 1208, 2048, 7, 2473984)

  assert result is not None
  assert result.egl_image is egl_image
  assert result.fd == 101
  assert reinitializations == [True]


def test_zero_copy_rebinds_current_camera_image_before_every_draw(monkeypatch):
  camera = object.__new__(LiveRoadCamera)
  camera._frame = SimpleNamespace(idx=3, width=1928, height=1208)
  camera._texture = SimpleNamespace(id=7, width=1, height=1)
  camera._egl_images = {3: object()}
  camera._egl_import_failures = 0
  camera._texture_needs_update = True
  camera._shader = object()
  binds = []

  monkeypatch.setattr(
    "openpilot.system.ui.lib.egl.bind_egl_image_to_texture",
    lambda texture_id, egl_image: binds.append((texture_id, egl_image)),
  )
  monkeypatch.setattr(cluster_live_camera.rl, "begin_shader_mode", lambda shader: None)
  monkeypatch.setattr(cluster_live_camera.rl, "draw_texture_pro", lambda *args: None)
  monkeypatch.setattr(cluster_live_camera.rl, "end_shader_mode", lambda: None)

  rect = cluster_live_camera.rl.Rectangle(0.0, 0.0, 100.0, 100.0)
  assert camera._draw_zero_copy(rect, rect)
  assert camera._draw_zero_copy(rect, rect)
  assert binds == [(7, camera._egl_images[3]), (7, camera._egl_images[3])]

  camera._texture_needs_update = True
  assert camera._draw_zero_copy(rect, rect)
  assert binds == [(7, camera._egl_images[3])] * 3


def test_camera_switch_waits_for_wide_frame_before_changing_projection(monkeypatch):
  road_stream = 0
  wide_stream = 2

  class FakeClient:
    streams = {road_stream, wide_stream}
    target_frame = None

    def __init__(self, _name, stream, conflate=True):
      self.stream = stream
      self.num_buffers = 4
      self.width = 1928
      self.height = 1208
      self.connected = stream == road_stream

    @classmethod
    def available_streams(cls, _name, block=False):
      return cls.streams

    def is_connected(self):
      return self.connected

    def connect(self, _blocking):
      self.connected = True
      return True

    def recv(self, timeout_ms=0):
      return self.target_frame if self.stream == wide_stream else None

  camera = object.__new__(LiveRoadCamera)
  camera._client_cls = FakeClient
  camera._road_stream_type = road_stream
  camera._wide_stream_type = wide_stream
  camera._stream_type = road_stream
  camera._client = FakeClient("camerad", road_stream)
  camera._target_client = None
  camera._target_stream_type = None
  camera._available_streams = set()
  camera._last_stream_discovery = 0.0
  camera._frame = object()
  camera._connected_at = 1.0
  camera._last_frame_at = 1.0
  camera._texture_needs_update = False
  camera._zero_copy = False
  monkeypatch.setattr(camera, "_destroy_egl_images", lambda: None)
  monkeypatch.setattr(camera, "_clear_copy_textures", lambda: None)

  assert not camera.select_stream(True)
  assert camera._target_stream_type == wide_stream

  FakeClient.target_frame = SimpleNamespace(idx=1)
  camera._last_stream_discovery = 0.0
  assert camera.select_stream(True)
  assert camera._stream_type == wide_stream
  assert camera._frame is FakeClient.target_frame


def test_camera_wide_request_falls_back_when_stream_is_unavailable():
  road_stream = 0

  class FakeClient:
    @classmethod
    def available_streams(cls, _name, block=False):
      return {road_stream}

  camera = object.__new__(LiveRoadCamera)
  camera._client_cls = FakeClient
  camera._road_stream_type = road_stream
  camera._wide_stream_type = 2
  camera._stream_type = road_stream
  camera._target_client = None
  camera._target_stream_type = None
  camera._available_streams = set()
  camera._last_stream_discovery = 0.0

  assert not camera.select_stream(True)
  assert camera._target_client is None
