from __future__ import annotations

import ctypes
import itertools
import os
from pathlib import Path
import threading


OPENPILOT_ROOT = Path(__file__).resolve().parents[3]
DEFAULT_DECODER_LIBRARY = OPENPILOT_ROOT / "system" / "loggerd" / "libcluster_h264_decoder_bridge.so"
DEFAULT_DECODER_DEVICE = "/dev/v4l/by-path/platform-aa00000.qcom_vidc-video-index0"
DEFAULT_DECODE_TIMEOUT_MS = 250


class HardwareH264DecoderError(RuntimeError):
  pass


class TiciH264DecodedBuffer:
  def __init__(
    self,
    owner: TiciH264Decoder,
    index: int,
    fd: int,
    width: int,
    height: int,
    stride: int,
    uv_offset: int,
    sequence: int,
  ) -> None:
    self._owner = owner
    self.index = int(index)
    self.fd = int(fd)
    self.width = int(width)
    self.height = int(height)
    self.stride = int(stride)
    self.uv_offset = int(uv_offset)
    self.sequence = int(sequence)
    self.token = (owner.generation, self.index)
    self._released = False

  def mark_egl_import_failed(self, reason: str) -> None:
    self._owner.disable(reason)

  def release(self) -> None:
    if self._released:
      return
    self._released = True
    try:
      os.close(self.fd)
    except OSError:
      pass
    self._owner.release(self.index)

  def __del__(self) -> None:
    self.release()


class TiciH264Decoder:
  _generation_counter = itertools.count(1)

  def __init__(
    self,
    width: int,
    height: int,
    fps: int = 30,
    *,
    library_path: str | os.PathLike[str] = DEFAULT_DECODER_LIBRARY,
    device_path: str = DEFAULT_DECODER_DEVICE,
    timeout_ms: int = DEFAULT_DECODE_TIMEOUT_MS,
    debug: bool = False,
  ) -> None:
    self.width = int(width)
    self.height = int(height)
    self.fps = max(1, int(fps))
    self.timeout_ms = max(1, int(timeout_ms))
    self.generation = next(self._generation_counter)
    self._lock = threading.Lock()
    self._disabled_reason = ""
    self._handle: int | None = None
    self._lib = ctypes.CDLL(str(library_path))
    self._configure_library(self._lib)
    self._handle = self._lib.cluster_h264_decoder_bridge_create(
      self.width,
      self.height,
      self.fps,
      os.fsencode(device_path),
      int(debug),
    )
    if not self._handle:
      raise HardwareH264DecoderError("failed to allocate the TICI H264 decoder bridge")
    if self._lib.cluster_h264_decoder_bridge_open(self._handle) != 0:
      error = self._last_error()
      self.close()
      raise HardwareH264DecoderError(error or "failed to open the TICI H264 decoder")

  @staticmethod
  def _configure_library(lib: ctypes.CDLL) -> None:
    lib.cluster_h264_decoder_bridge_create.argtypes = [
      ctypes.c_int,
      ctypes.c_int,
      ctypes.c_int,
      ctypes.c_char_p,
      ctypes.c_int,
    ]
    lib.cluster_h264_decoder_bridge_create.restype = ctypes.c_void_p
    lib.cluster_h264_decoder_bridge_open.argtypes = [ctypes.c_void_p]
    lib.cluster_h264_decoder_bridge_open.restype = ctypes.c_int
    lib.cluster_h264_decoder_bridge_decode.argtypes = [
      ctypes.c_void_p,
      ctypes.c_void_p,
      ctypes.c_size_t,
      ctypes.c_uint64,
      ctypes.c_int,
      ctypes.POINTER(ctypes.c_uint),
      ctypes.POINTER(ctypes.c_int),
      ctypes.POINTER(ctypes.c_size_t),
      ctypes.POINTER(ctypes.c_size_t),
      ctypes.POINTER(ctypes.c_size_t),
      ctypes.POINTER(ctypes.c_size_t),
      ctypes.POINTER(ctypes.c_uint64),
    ]
    lib.cluster_h264_decoder_bridge_decode.restype = ctypes.c_int
    lib.cluster_h264_decoder_bridge_release.argtypes = [ctypes.c_void_p, ctypes.c_uint]
    lib.cluster_h264_decoder_bridge_release.restype = ctypes.c_int
    lib.cluster_h264_decoder_bridge_close.argtypes = [ctypes.c_void_p]
    lib.cluster_h264_decoder_bridge_close.restype = None
    lib.cluster_h264_decoder_bridge_destroy.argtypes = [ctypes.c_void_p]
    lib.cluster_h264_decoder_bridge_destroy.restype = None
    lib.cluster_h264_decoder_bridge_last_error.argtypes = [ctypes.c_void_p]
    lib.cluster_h264_decoder_bridge_last_error.restype = ctypes.c_char_p

  def decode(self, access_unit: bytes, sequence: int) -> TiciH264DecodedBuffer | None:
    if not access_unit:
      return None
    if self._disabled_reason:
      raise HardwareH264DecoderError(self._disabled_reason)
    handle = self._handle
    if handle is None:
      raise HardwareH264DecoderError("TICI H264 decoder is closed")

    index = ctypes.c_uint()
    fd = ctypes.c_int(-1)
    width = ctypes.c_size_t()
    height = ctypes.c_size_t()
    stride = ctypes.c_size_t()
    uv_offset = ctypes.c_size_t()
    decoded_sequence = ctypes.c_uint64()
    payload = ctypes.c_char_p(access_unit)
    result = self._lib.cluster_h264_decoder_bridge_decode(
      handle,
      ctypes.cast(payload, ctypes.c_void_p),
      len(access_unit),
      max(0, int(sequence)),
      self.timeout_ms,
      ctypes.byref(index),
      ctypes.byref(fd),
      ctypes.byref(width),
      ctypes.byref(height),
      ctypes.byref(stride),
      ctypes.byref(uv_offset),
      ctypes.byref(decoded_sequence),
    )
    if result < 0:
      raise HardwareH264DecoderError(self._last_error() or "TICI H264 decode failed")
    if result == 0:
      return None
    try:
      lease_fd = os.dup(fd.value)
    except OSError as exc:
      self.release(index.value)
      raise HardwareH264DecoderError(f"failed to duplicate decoded DMA-BUF: {exc}") from exc
    return TiciH264DecodedBuffer(
      self,
      index.value,
      lease_fd,
      width.value,
      height.value,
      stride.value,
      uv_offset.value,
      decoded_sequence.value or sequence,
    )

  def disable(self, reason: str) -> None:
    with self._lock:
      if not self._disabled_reason:
        self._disabled_reason = reason or "TICI H264 EGL import failed"

  def release(self, index: int) -> None:
    with self._lock:
      handle = self._handle
      if handle is None:
        return
      if self._lib.cluster_h264_decoder_bridge_release(handle, int(index)) != 0 and not self._disabled_reason:
        self._disabled_reason = self._last_error() or "failed to return a decoded VIDC buffer"

  def close(self) -> None:
    with self._lock:
      handle = self._handle
      self._handle = None
      if handle is not None:
        self._lib.cluster_h264_decoder_bridge_destroy(handle)

  def _last_error(self) -> str:
    handle = self._handle
    if handle is None:
      return ""
    error = self._lib.cluster_h264_decoder_bridge_last_error(handle)
    return error.decode("utf-8", errors="replace") if error else ""

  def __del__(self) -> None:
    self.close()


def create_tici_h264_decoder(width: int, height: int, fps: int = 30) -> TiciH264Decoder | None:
  if os.environ.get("CLUSTER_HARDWARE_H264_DECODE", "1") == "0":
    return None
  library_path = Path(os.environ.get("CLUSTER_H264_DECODER_LIBRARY", str(DEFAULT_DECODER_LIBRARY)))
  if not library_path.is_file():
    raise HardwareH264DecoderError(f"TICI H264 decoder bridge is missing: {library_path}")
  return TiciH264Decoder(
    width,
    height,
    fps,
    library_path=library_path,
    device_path=os.environ.get("CLUSTER_H264_DECODER_DEVICE", DEFAULT_DECODER_DEVICE),
    timeout_ms=int(os.environ.get("CLUSTER_H264_DECODE_TIMEOUT_MS", str(DEFAULT_DECODE_TIMEOUT_MS))),
    debug=os.environ.get("CLUSTER_H264_DECODER_DEBUG") == "1",
  )
