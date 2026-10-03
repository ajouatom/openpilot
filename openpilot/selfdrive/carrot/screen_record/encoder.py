"""ctypes wrapper around the V4L2/ION hardware H.264 encoder bridge.

The bridge is the compiled carrot helper also used by the cluster feature
(system/loggerd/libcluster_h264_encoder_bridge.so). Its C API is handle based,
so screen recording creates its own encoder instance and never shares state
with the cluster feature (which additionally runs in its own process).
"""
from __future__ import annotations

import ctypes
import threading
from dataclasses import dataclass
from pathlib import Path

from openpilot.common.basedir import BASEDIR

DEFAULT_DEVICE_PATH = "/dev/v4l/by-path/platform-aa00000.qcom_vidc-video-index1"
INPUT_FORMAT_NV12 = 1
RATE_CONTROL_VBR_CFR = 2

# <package>/system/loggerd/libcluster_h264_encoder_bridge.so (encoder.py lives
# at <package>/selfdrive/carrot/screen_record/encoder.py).
_PACKAGE_ROOT = Path(__file__).resolve().parents[3]

_PACKET_CALLBACK = ctypes.CFUNCTYPE(
  None,
  ctypes.c_void_p,
  ctypes.c_size_t,
  ctypes.c_uint32,
  ctypes.c_uint64,
  ctypes.c_int,
  ctypes.c_int,
  ctypes.c_void_p,
)


class HwEncoderUnavailable(RuntimeError):
  pass


@dataclass
class HwNv12Input:
  address: int
  size: int
  index: int
  dmabuf_fd: int = -1
  active: bool = True


@dataclass
class HwNv12Layout:
  """Venus NV12 layout reported by the encoder, with packed render geometry."""

  stride: int
  y_scanlines: int
  uv_scanlines: int
  uv_offset: int
  bytesused: int
  active_bytes: int
  render_bytes: int
  pack_width: int
  pack_height: int
  y_pack_y: int
  uv_pack_y: int


class HwNv12Encoder:
  def __init__(
    self,
    width: int,
    height: int,
    fps: int,
    bitrate: int,
    gop: int = 30,
    device_path: str = DEFAULT_DEVICE_PATH,
    slice_max_bytes: int = 4096,
    rate_control: int = RATE_CONTROL_VBR_CFR,
    debug: bool = False,
  ):
    self._width = int(width)
    self._height = int(height)
    self._fps = int(fps)
    self._packets: list[bytes] = []
    self._packets_lock = threading.Lock()
    self._callback = _PACKET_CALLBACK(self._on_packet)
    self._handle = None

    self._lib = self._load_library()
    self._configure_library()
    handle = self._lib.cluster_h264_encoder_bridge_create(
      self._width,
      self._height,
      self._fps,
      int(bitrate),
      int(gop),
      str(device_path).encode("utf-8"),
      INPUT_FORMAT_NV12,
      0,
      1 if debug else 0,
    )
    if not handle:
      raise HwEncoderUnavailable("encoder bridge allocation failed")
    self._handle = handle

    self._best_effort_setup(slice_max_bytes, rate_control)
    if self._lib.cluster_h264_encoder_bridge_open(self._handle) != 0:
      error = self.last_error()
      self._destroy()
      raise HwEncoderUnavailable(f"encoder open failed: {error}")

    self._layout = self._read_layout()

  # -- public API ---------------------------------------------------------

  @property
  def layout(self) -> HwNv12Layout:
    return self._layout

  @property
  def has_dmabuf_input(self) -> bool:
    return self._acquire_dmabuf is not None and self._submit_dmabuf is not None

  @property
  def fps(self) -> int:
    return self._fps

  def acquire(self) -> HwNv12Input:
    address = ctypes.c_void_p()
    size = ctypes.c_size_t()
    index = ctypes.c_uint32()
    dmabuf_fd = ctypes.c_int(-1)
    if self.has_dmabuf_input:
      result = self._acquire_dmabuf(
        self._handle, ctypes.byref(address), ctypes.byref(size), ctypes.byref(index),
        ctypes.byref(dmabuf_fd), self._callback, None,
      )
    else:
      result = self._acquire(
        self._handle, ctypes.byref(address), ctypes.byref(size), ctypes.byref(index),
        self._callback, None,
      )
    if result != 0:
      raise HwEncoderUnavailable(self._error_text("input acquire failed"))
    buffer = HwNv12Input(
      address=int(address.value or 0),
      size=int(size.value),
      index=int(index.value),
      dmabuf_fd=int(dmabuf_fd.value),
    )
    if buffer.size < self._layout.bytesused or (buffer.address == 0 and buffer.dmabuf_fd < 0):
      self.cancel(buffer.index)
      raise HwEncoderUnavailable("input acquire returned an invalid buffer")
    return buffer

  def submit(self, buffer: HwNv12Input, timestamp_us: int) -> None:
    if not buffer.active:
      raise HwEncoderUnavailable("input buffer lease is not active")
    if buffer.dmabuf_fd >= 0 and self._submit_dmabuf is not None:
      result = self._submit_dmabuf(self._handle, buffer.index, int(timestamp_us), self._callback, None)
    else:
      result = self._submit(self._handle, buffer.index, int(timestamp_us), self._callback, None)
    if result != 0:
      raise HwEncoderUnavailable(self._error_text("input submit failed"))
    buffer.active = False

  def cancel(self, index: int) -> None:
    if self._cancel is not None:
      self._cancel(self._handle, int(index))

  def drain(self, timeout_ms: int = 0) -> None:
    if self._drain is not None:
      self._drain(self._handle, int(timeout_ms), self._callback, None)

  def take_packets(self) -> list[bytes]:
    with self._packets_lock:
      packets = self._packets
      self._packets = []
    return packets

  def last_error(self) -> str:
    if self._handle is None or self._last_error is None:
      return ""
    value = self._last_error(self._handle)
    return value.decode("utf-8", errors="ignore") if value else ""

  def close(self) -> None:
    if self._handle is None:
      return
    try:
      self.drain(200)
    except Exception:
      pass
    try:
      self._lib.cluster_h264_encoder_bridge_close(self._handle)
    except Exception:
      pass
    self._destroy()

  # -- internals ----------------------------------------------------------

  def _destroy(self) -> None:
    if self._handle is None:
      return
    try:
      self._lib.cluster_h264_encoder_bridge_destroy(self._handle)
    finally:
      self._handle = None

  def _on_packet(self, data, size, _flags, _timestamp_us, _codec_config, _keyframe, _opaque) -> None:
    if not data or size <= 0:
      return
    packet = ctypes.string_at(int(data), int(size))
    with self._packets_lock:
      self._packets.append(packet)

  def _error_text(self, message: str) -> str:
    error = self.last_error()
    return f"{message}: {error}" if error else message

  def _best_effort_setup(self, slice_max_bytes: int, rate_control: int) -> None:
    for name, value in (
      ("cluster_h264_encoder_bridge_set_slice_max_bytes", slice_max_bytes),
      ("cluster_h264_encoder_bridge_set_rate_control", rate_control),
    ):
      fn = getattr(self._lib, name, None)
      if fn is None:
        continue
      try:
        fn(self._handle, int(value))
      except Exception:
        continue

  def _read_layout(self) -> HwNv12Layout:
    lib = self._lib
    handle = self._handle
    stride = int(lib.cluster_h264_encoder_bridge_input_stride(handle))
    y_scanlines = int(lib.cluster_h264_encoder_bridge_input_y_scanlines(handle))
    uv_scanlines = int(lib.cluster_h264_encoder_bridge_input_uv_scanlines(handle))
    uv_offset = int(lib.cluster_h264_encoder_bridge_input_uv_offset(handle))
    bytesused = int(lib.cluster_h264_encoder_bridge_input_bytesused(handle))
    active_fn = getattr(lib, "cluster_h264_encoder_bridge_input_active_bytes", None)
    active_bytes = int(active_fn(handle)) if active_fn is not None else 0
    if active_bytes <= 0:
      active_bytes = uv_offset + stride * uv_scanlines
    if stride <= 0 or stride % 4 != 0 or y_scanlines <= 0 or uv_scanlines <= 0 or bytesused <= 0:
      raise HwEncoderUnavailable(
        f"encoder reported an unusable NV12 layout (stride={stride} y={y_scanlines} uv={uv_scanlines} bytes={bytesused})"
      )
    use_active = 0 < active_bytes < bytesused
    render_bytes = active_bytes if use_active else bytesused
    if render_bytes % stride != 0:
      raise HwEncoderUnavailable("encoder NV12 render layout is not four-byte packed")
    pack_width = stride // 4
    pack_height = render_bytes // stride
    tail_rows = max(0, pack_height - y_scanlines - uv_scanlines)
    return HwNv12Layout(
      stride=stride,
      y_scanlines=y_scanlines,
      uv_scanlines=uv_scanlines,
      uv_offset=uv_offset,
      bytesused=bytesused,
      active_bytes=active_bytes,
      render_bytes=render_bytes,
      pack_width=pack_width,
      pack_height=pack_height,
      y_pack_y=tail_rows + uv_scanlines,
      uv_pack_y=tail_rows,
    )

  @staticmethod
  def _load_library() -> ctypes.CDLL:
    candidates = (
      str(_PACKAGE_ROOT / "system" / "loggerd" / "libcluster_h264_encoder_bridge.so"),
      str(Path(BASEDIR) / "openpilot" / "system" / "loggerd" / "libcluster_h264_encoder_bridge.so"),
      "libcluster_h264_encoder_bridge.so",
    )
    errors: list[str] = []
    for candidate in candidates:
      try:
        return ctypes.CDLL(candidate)
      except OSError as exc:
        errors.append(f"{candidate}: {exc}")
    raise HwEncoderUnavailable("encoder bridge library unavailable; " + "; ".join(errors[:2]))

  def _configure_library(self) -> None:
    lib = self._lib
    lib.cluster_h264_encoder_bridge_create.argtypes = [
      ctypes.c_int, ctypes.c_int, ctypes.c_int, ctypes.c_int, ctypes.c_int,
      ctypes.c_char_p, ctypes.c_int, ctypes.c_int, ctypes.c_int,
    ]
    lib.cluster_h264_encoder_bridge_create.restype = ctypes.c_void_p
    lib.cluster_h264_encoder_bridge_open.argtypes = [ctypes.c_void_p]
    lib.cluster_h264_encoder_bridge_open.restype = ctypes.c_int
    lib.cluster_h264_encoder_bridge_close.argtypes = [ctypes.c_void_p]
    lib.cluster_h264_encoder_bridge_close.restype = None
    lib.cluster_h264_encoder_bridge_destroy.argtypes = [ctypes.c_void_p]
    lib.cluster_h264_encoder_bridge_destroy.restype = None
    lib.cluster_h264_encoder_bridge_last_error.argtypes = [ctypes.c_void_p]
    lib.cluster_h264_encoder_bridge_last_error.restype = ctypes.c_char_p
    for name in (
      "cluster_h264_encoder_bridge_input_stride",
      "cluster_h264_encoder_bridge_input_y_scanlines",
      "cluster_h264_encoder_bridge_input_uv_scanlines",
      "cluster_h264_encoder_bridge_input_uv_offset",
      "cluster_h264_encoder_bridge_input_sizeimage",
      "cluster_h264_encoder_bridge_input_bytesused",
      "cluster_h264_encoder_bridge_input_active_bytes",
      "cluster_h264_encoder_bridge_capture_sizeimage",
    ):
      fn = getattr(lib, name, None)
      if fn is None:
        continue
      fn.argtypes = [ctypes.c_void_p]
      fn.restype = ctypes.c_size_t

    self._acquire = self._configure_optional(lib, "cluster_h264_encoder_bridge_acquire_nv12_input", [
      ctypes.c_void_p, ctypes.POINTER(ctypes.c_void_p), ctypes.POINTER(ctypes.c_size_t),
      ctypes.POINTER(ctypes.c_uint32), _PACKET_CALLBACK, ctypes.c_void_p,
    ])
    self._submit = self._configure_optional(lib, "cluster_h264_encoder_bridge_submit_nv12_input", [
      ctypes.c_void_p, ctypes.c_uint32, ctypes.c_uint64, _PACKET_CALLBACK, ctypes.c_void_p,
    ])
    self._acquire_dmabuf = self._configure_optional(lib, "cluster_h264_encoder_bridge_acquire_nv12_input_dmabuf", [
      ctypes.c_void_p, ctypes.POINTER(ctypes.c_void_p), ctypes.POINTER(ctypes.c_size_t),
      ctypes.POINTER(ctypes.c_uint32), ctypes.POINTER(ctypes.c_int), _PACKET_CALLBACK, ctypes.c_void_p,
    ])
    self._submit_dmabuf = self._configure_optional(lib, "cluster_h264_encoder_bridge_submit_nv12_input_dmabuf", [
      ctypes.c_void_p, ctypes.c_uint32, ctypes.c_uint64, _PACKET_CALLBACK, ctypes.c_void_p,
    ])
    self._cancel = self._configure_optional(lib, "cluster_h264_encoder_bridge_cancel_nv12_input", [
      ctypes.c_void_p, ctypes.c_uint32,
    ])
    self._drain = self._configure_optional(lib, "cluster_h264_encoder_bridge_drain", [
      ctypes.c_void_p, ctypes.c_int, _PACKET_CALLBACK, ctypes.c_void_p,
    ])
    self._last_error = getattr(lib, "cluster_h264_encoder_bridge_last_error", None)
    if self._acquire is None or self._submit is None:
      raise HwEncoderUnavailable("encoder bridge does not expose the NV12 input API")

  @staticmethod
  def _configure_optional(lib, name: str, argtypes: list):
    fn = getattr(lib, name, None)
    if fn is None:
      return None
    fn.argtypes = argtypes
    fn.restype = ctypes.c_int
    return fn
