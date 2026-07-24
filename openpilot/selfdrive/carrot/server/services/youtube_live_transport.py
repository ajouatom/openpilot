from __future__ import annotations

import ctypes
import ctypes.util
import io
import select
import socket
import ssl
import threading
from typing import Any
from urllib.parse import urlsplit, urlunsplit


REQUIRED_RTMP_SYMBOLS = (
  "RTMP_Alloc",
  "RTMP_Init",
  "RTMP_SetupURL",
  "RTMP_SetOpt",
  "RTMP_EnableWrite",
  "RTMP_Connect",
  "RTMP_ConnectStream",
  "RTMP_Write",
  "RTMP_IsConnected",
  "RTMP_Close",
  "RTMP_Free",
)


# If forwarding to YouTube stalls this long (dead hotspot), fail fast instead of
# blocking for the TCP retransmit timeout (minutes) so the service can reconnect.
TUNNEL_STALL_TIMEOUT_SECONDS = 6.0


class LibrtmpError(RuntimeError):
  pass


class _AVal(ctypes.Structure):
  _fields_ = [("value", ctypes.c_char_p), ("length", ctypes.c_int)]


class _TlsTunnel:
  def __init__(self, host: str, port: int) -> None:
    self._host = host
    self._port = port
    self._stop_event = threading.Event()
    self._listener = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    self._listener.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
    self._listener.bind(("127.0.0.1", 0))
    self._listener.listen(1)
    self.local_port = int(self._listener.getsockname()[1])
    self.error = ""
    self._local: socket.socket | None = None
    self._remote: ssl.SSLSocket | None = None
    self._thread = threading.Thread(target=self._run, name="youtube-rtmps-tunnel", daemon=True)

  def start(self) -> None:
    self._thread.start()

  def close(self) -> None:
    self._stop_event.set()
    for sock in (self._local, self._remote, self._listener):
      if sock is not None:
        try:
          sock.shutdown(socket.SHUT_RDWR)
        except OSError:
          pass
        try:
          sock.close()
        except OSError:
          pass
    if self._thread.is_alive() and threading.current_thread() is not self._thread:
      self._thread.join(timeout=2.0)

  def _run(self) -> None:
    raw_remote: socket.socket | None = None
    try:
      self._listener.settimeout(10.0)
      self._local, _ = self._listener.accept()
      raw_remote = socket.create_connection((self._host, self._port), timeout=8.0)
      self._remote = ssl.create_default_context().wrap_socket(raw_remote, server_hostname=self._host)
      raw_remote = None
      self._local.settimeout(None)
      # Bound sends to YouTube so a dead network surfaces as a timeout in seconds
      # (select gates recv, so this only bites a genuinely stalled forward).
      self._remote.settimeout(TUNNEL_STALL_TIMEOUT_SECONDS)
      sockets = (self._local, self._remote)
      while not self._stop_event.is_set():
        readable, _, _ = select.select(sockets, [], [], 0.5)
        for source in readable:
          payload = source.recv(64 * 1024)
          if not payload:
            return
          target = self._remote if source is self._local else self._local
          target.sendall(payload)
    except Exception as exc:
      if not self._stop_event.is_set():
        self.error = str(exc)
    finally:
      if raw_remote is not None:
        raw_remote.close()
      for sock in (self._local, self._remote):
        if sock is not None:
          try:
            sock.close()
          except OSError:
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
      "rtmps_mode": "python-tls-tunnel",
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
    self._tc_url_buffer: ctypes.Array[ctypes.c_char] | None = None
    self._tunnel: _TlsTunnel | None = None
    self._handle: int | None = None
    self._lock = threading.RLock()
    self._bytes_written = 0

  @property
  def bytes_written(self) -> int:
    # Diagnostic progress reads must not wait behind a blocked RTMP_Write.
    # CPython integer replacement is atomic; a one-sample stale value is fine.
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
        setup_url, tc_url = self._prepare_url()
        self._url_buffer = ctypes.create_string_buffer(setup_url.encode("utf-8"))
        if not self._library.RTMP_SetupURL(handle, self._url_buffer):
          raise LibrtmpError("librtmp URL setup failed")
        if tc_url:
          self._set_string_option(handle, "tcUrl", tc_url)
        self._library.RTMP_EnableWrite(handle)
        if not self._library.RTMP_Connect(handle, None):
          detail = f": {self._tunnel.error}" if self._tunnel and self._tunnel.error else ""
          raise LibrtmpError(f"YouTube RTMPS connection failed{detail}")
        if not self._library.RTMP_ConnectStream(handle, 0):
          raise LibrtmpError("YouTube rejected the publish connection")
      except Exception:
        self._close_locked()
        raise

  def write(self, data: bytes | bytearray | memoryview) -> int:
    # Returns bytes CONSUMED by librtmp (may be < len). librtmp's RTMP_Write
    # parses FLV tags and reads look-ahead beyond the current tag; feeding it a
    # small single-tag buffer makes it read out of bounds and segfault, so the
    # caller must batch and carry the unconsumed remainder forward.
    payload = bytes(data)
    if not payload:
      return 0
    with self._lock:
      handle = self._handle
      if handle is None or not self._library.RTMP_IsConnected(handle):
        raise LibrtmpError("YouTube RTMPS connection is closed")
      buffer = ctypes.create_string_buffer(payload)
      written = int(self._library.RTMP_Write(handle, buffer, len(payload)))
      if written <= 0:
        raise LibrtmpError("YouTube RTMPS write failed")
      self._bytes_written += written
      return written

  def is_connected(self) -> bool:
    with self._lock:
      return bool(self._handle and self._library.RTMP_IsConnected(self._handle))

  def try_is_connected(self) -> bool | None:
    # A network write owns the same librtmp handle and may be blocked until the
    # tunnel timeout. Health polling must not wait behind that write because it
    # would stall source ingestion and defeat the bounded writer queue.
    if not self._lock.acquire(blocking=False):
      return None
    try:
      return bool(self._handle and self._library.RTMP_IsConnected(self._handle))
    finally:
      self._lock.release()

  def close(self) -> None:
    with self._lock:
      self._close_locked()

  def _close_locked(self) -> None:
    handle = self._handle
    self._handle = None
    if handle is None:
      self._url_buffer = None
      self._tc_url_buffer = None
      if self._tunnel is not None:
        self._tunnel.close()
        self._tunnel = None
      return
    try:
      self._library.RTMP_Close(handle)
    finally:
      self._library.RTMP_Free(handle)
      self._url_buffer = None
      self._tc_url_buffer = None
      if self._tunnel is not None:
        self._tunnel.close()
        self._tunnel = None

  def _prepare_url(self) -> tuple[str, str]:
    parsed = urlsplit(self._url)
    if parsed.scheme.lower() != "rtmps":
      return self._url, ""
    host = parsed.hostname or ""
    if not host:
      raise LibrtmpError("RTMPS host is missing")
    port = parsed.port or 443
    self._tunnel = _TlsTunnel(host, port)
    self._tunnel.start()
    setup_url = urlunsplit(("rtmp", f"127.0.0.1:{self._tunnel.local_port}", parsed.path, parsed.query, ""))
    path_parts = [part for part in parsed.path.split("/") if part]
    app_path = f"/{path_parts[0]}" if path_parts else "/"
    tc_url = urlunsplit(("rtmps", parsed.netloc, app_path, "", ""))
    return setup_url, tc_url

  def _set_string_option(self, handle: int, name: str, value: str) -> None:
    name_buffer = ctypes.create_string_buffer(name.encode("utf-8"))
    self._tc_url_buffer = ctypes.create_string_buffer(value.encode("utf-8"))
    option = _AVal(ctypes.cast(name_buffer, ctypes.c_char_p), len(name_buffer.value))
    argument = _AVal(ctypes.cast(self._tc_url_buffer, ctypes.c_char_p), len(self._tc_url_buffer.value))
    if not self._library.RTMP_SetOpt(handle, ctypes.byref(option), ctypes.byref(argument)):
      raise LibrtmpError(f"librtmp option failed: {name}")

  def _configure_library(self) -> None:
    missing = [name for name in REQUIRED_RTMP_SYMBOLS if not hasattr(self._library, name)]
    if missing:
      raise LibrtmpError(f"missing librtmp symbols: {', '.join(missing)}")

    self._library.RTMP_Alloc.restype = ctypes.c_void_p
    self._library.RTMP_Init.argtypes = [ctypes.c_void_p]
    self._library.RTMP_SetupURL.argtypes = [ctypes.c_void_p, ctypes.c_char_p]
    self._library.RTMP_SetupURL.restype = ctypes.c_int
    self._library.RTMP_SetOpt.argtypes = [ctypes.c_void_p, ctypes.POINTER(_AVal), ctypes.POINTER(_AVal)]
    self._library.RTMP_SetOpt.restype = ctypes.c_int
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
  # Buffer muxer output and hand librtmp large, multi-tag chunks. Feeding
  # librtmp one small FLV tag at a time makes RTMP_Write read past the buffer
  # (look-ahead) and segfault. We keep whatever librtmp does not consume and
  # prepend it to the next batch so it always has trailing context.
  FLUSH_THRESHOLD = 16384

  def __init__(self, client: LibrtmpClient) -> None:
    super().__init__()
    self._client = client
    self._position = 0
    self._pending = bytearray()
    self._drain_calls = 0
    self._partial_writes = 0

  @property
  def bytes_accepted(self) -> int:
    return self._position

  @property
  def pending_bytes(self) -> int:
    return len(self._pending)

  @property
  def drain_calls(self) -> int:
    return self._drain_calls

  @property
  def partial_writes(self) -> int:
    return self._partial_writes

  def writable(self) -> bool:
    return True

  def seekable(self) -> bool:
    return False

  def write(self, data: bytes | bytearray | memoryview) -> int:
    chunk = bytes(data)
    self._pending.extend(chunk)
    self._position += len(chunk)
    if len(self._pending) >= self.FLUSH_THRESHOLD:
      self._drain(self.FLUSH_THRESHOLD)
    return len(chunk)

  def _drain(self, floor: int) -> None:
    # Keep at least `floor` bytes buffered so librtmp always has look-ahead.
    while len(self._pending) >= floor:
      pending_before = len(self._pending)
      written = self._client.write(bytes(self._pending))
      if written <= 0:
        break
      self._drain_calls += 1
      if written < pending_before:
        self._partial_writes += 1
      del self._pending[:written]

  def tell(self) -> int:
    return self._position

  def flush(self) -> None:
    while self._pending:
      pending_before = len(self._pending)
      written = self._client.write(bytes(self._pending))
      if written <= 0:
        break
      self._drain_calls += 1
      if written < pending_before:
        self._partial_writes += 1
      del self._pending[:written]
