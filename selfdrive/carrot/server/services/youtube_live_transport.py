from __future__ import annotations

import ctypes
import ctypes.util
import io
import threading
from typing import Any


REQUIRED_RTMP_SYMBOLS = (
  "RTMP_Alloc",
  "RTMP_Init",
  "RTMP_SetupURL",
  "RTMP_EnableWrite",
  "RTMP_Connect",
  "RTMP_ConnectStream",
  "RTMP_Write",
  "RTMP_IsConnected",
  "RTMP_Close",
  "RTMP_Free",
)


class LibrtmpError(RuntimeError):
  pass


def librtmp_capabilities() -> dict[str, Any]:
  path = ctypes.util.find_library("rtmp") or ""
  if not path:
    return {"available": False, "path": "", "missing_symbols": [], "error": "librtmp not found"}
  try:
    library = ctypes.CDLL(path)
    missing = [name for name in REQUIRED_RTMP_SYMBOLS if not hasattr(library, name)]
    return {
      "available": not missing,
      "path": path,
      "missing_symbols": missing,
      "error": f"missing librtmp symbols: {', '.join(missing)}" if missing else "",
    }
  except Exception as exc:
    return {"available": False, "path": path, "missing_symbols": [], "error": str(exc)}


class LibrtmpClient:
  def __init__(self, url: str) -> None:
    path = ctypes.util.find_library("rtmp")
    if not path:
      raise LibrtmpError("librtmp not found")
    self._library = ctypes.CDLL(path)
    self._configure_library()
    self._url = url
    self._url_buffer: ctypes.Array[ctypes.c_char] | None = None
    self._handle: int | None = None
    self._lock = threading.RLock()
    self._bytes_written = 0

  @property
  def bytes_written(self) -> int:
    with self._lock:
      return self._bytes_written

  def connect(self) -> None:
    with self._lock:
      if self._handle is not None:
        return
      handle = self._library.RTMP_Alloc()
      if not handle:
        raise LibrtmpError("librtmp allocation failed")
      self._handle = handle
      try:
        self._library.RTMP_Init(handle)
        self._url_buffer = ctypes.create_string_buffer(self._url.encode("utf-8"))
        if not self._library.RTMP_SetupURL(handle, self._url_buffer):
          raise LibrtmpError("librtmp URL setup failed")
        self._library.RTMP_EnableWrite(handle)
        if not self._library.RTMP_Connect(handle, None):
          raise LibrtmpError("YouTube RTMPS connection failed")
        if not self._library.RTMP_ConnectStream(handle, 0):
          raise LibrtmpError("YouTube rejected the publish connection")
      except Exception:
        self._close_locked()
        raise

  def write(self, data: bytes | bytearray | memoryview) -> int:
    payload = bytes(data)
    if not payload:
      return 0
    with self._lock:
      handle = self._handle
      if handle is None or not self._library.RTMP_IsConnected(handle):
        raise LibrtmpError("YouTube RTMPS connection is closed")
      offset = 0
      while offset < len(payload):
        buffer = ctypes.create_string_buffer(payload[offset:])
        written = int(self._library.RTMP_Write(handle, buffer, len(payload) - offset))
        if written <= 0:
          raise LibrtmpError("YouTube RTMPS write failed")
        offset += written
      self._bytes_written += offset
      return offset

  def is_connected(self) -> bool:
    with self._lock:
      return bool(self._handle and self._library.RTMP_IsConnected(self._handle))

  def close(self) -> None:
    with self._lock:
      self._close_locked()

  def _close_locked(self) -> None:
    handle = self._handle
    self._handle = None
    if handle is None:
      self._url_buffer = None
      return
    try:
      self._library.RTMP_Close(handle)
    finally:
      self._library.RTMP_Free(handle)
      self._url_buffer = None

  def _configure_library(self) -> None:
    missing = [name for name in REQUIRED_RTMP_SYMBOLS if not hasattr(self._library, name)]
    if missing:
      raise LibrtmpError(f"missing librtmp symbols: {', '.join(missing)}")

    self._library.RTMP_Alloc.restype = ctypes.c_void_p
    self._library.RTMP_Init.argtypes = [ctypes.c_void_p]
    self._library.RTMP_SetupURL.argtypes = [ctypes.c_void_p, ctypes.c_char_p]
    self._library.RTMP_SetupURL.restype = ctypes.c_int
    self._library.RTMP_EnableWrite.argtypes = [ctypes.c_void_p]
    self._library.RTMP_Connect.argtypes = [ctypes.c_void_p, ctypes.c_void_p]
    self._library.RTMP_Connect.restype = ctypes.c_int
    self._library.RTMP_ConnectStream.argtypes = [ctypes.c_void_p, ctypes.c_int]
    self._library.RTMP_ConnectStream.restype = ctypes.c_int
    self._library.RTMP_Write.argtypes = [ctypes.c_void_p, ctypes.c_void_p, ctypes.c_int]
    self._library.RTMP_Write.restype = ctypes.c_int
    self._library.RTMP_IsConnected.argtypes = [ctypes.c_void_p]
    self._library.RTMP_IsConnected.restype = ctypes.c_int
    self._library.RTMP_Close.argtypes = [ctypes.c_void_p]
    self._library.RTMP_Free.argtypes = [ctypes.c_void_p]
    if hasattr(self._library, "RTMP_LogSetLevel"):
      self._library.RTMP_LogSetLevel.argtypes = [ctypes.c_int]
      self._library.RTMP_LogSetLevel(1)


class RtmpSink(io.RawIOBase):
  def __init__(self, client: LibrtmpClient) -> None:
    super().__init__()
    self._client = client
    self._position = 0

  def writable(self) -> bool:
    return True

  def seekable(self) -> bool:
    return False

  def write(self, data: bytes | bytearray | memoryview) -> int:
    written = self._client.write(data)
    self._position += written
    return written

  def tell(self) -> int:
    return self._position

  def flush(self) -> None:
    return None
