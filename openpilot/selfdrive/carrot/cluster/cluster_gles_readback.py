from __future__ import annotations

from collections import deque
import ctypes
from dataclasses import dataclass
import os
from pathlib import Path
import time


GL_FRAMEBUFFER = 0x8D40
GL_FRAMEBUFFER_BINDING = 0x8CA6
GL_PACK_ALIGNMENT = 0x0D05
GL_PIXEL_PACK_BUFFER = 0x88EB
GL_PIXEL_PACK_BUFFER_BINDING = 0x88ED
GL_RGBA = 0x1908
GL_UNSIGNED_BYTE = 0x1401
GL_NO_ERROR = 0
GL_STREAM_READ = 0x88E1
GL_MAP_READ_BIT = 0x0001
GL_SYNC_GPU_COMMANDS_COMPLETE = 0x9117
GL_ALREADY_SIGNALED = 0x911A
GL_TIMEOUT_EXPIRED = 0x911B
GL_CONDITION_SATISFIED = 0x911C
GL_WAIT_FAILED = 0x911D

ASYNC_READBACK_RING_SIZE = 3
ASYNC_READBACK_STALL_TIMEOUT_S = 1.0


class DirectNv12ReadbackError(RuntimeError):
  pass


@dataclass
class _PboSlot:
  buffer_id: int
  fence: int | None = None
  byte_count: int = 0
  submitted_s: float = 0.0


class GlesDirectReadback:
  """Provides synchronous direct and asynchronous PBO readback for packed TICI frames."""

  def __init__(self) -> None:
    try:
      self._gles = ctypes.CDLL("libGLESv2.so")
    except OSError as exc:
      raise DirectNv12ReadbackError(f"failed to load libGLESv2.so: {exc}") from exc

    self._gles.glGetIntegerv.argtypes = [ctypes.c_uint, ctypes.POINTER(ctypes.c_int)]
    self._gles.glGetIntegerv.restype = None
    self._gles.glBindFramebuffer.argtypes = [ctypes.c_uint, ctypes.c_uint]
    self._gles.glBindFramebuffer.restype = None
    self._gles.glBindBuffer.argtypes = [ctypes.c_uint, ctypes.c_uint]
    self._gles.glBindBuffer.restype = None
    self._gles.glPixelStorei.argtypes = [ctypes.c_uint, ctypes.c_int]
    self._gles.glPixelStorei.restype = None
    self._gles.glReadPixels.argtypes = [
      ctypes.c_int,
      ctypes.c_int,
      ctypes.c_int,
      ctypes.c_int,
      ctypes.c_uint,
      ctypes.c_uint,
      ctypes.c_void_p,
    ]
    self._gles.glReadPixels.restype = None
    self._gles.glGetError.argtypes = []
    self._gles.glGetError.restype = ctypes.c_uint

    self._async_supported = False
    self._pbo_slots: list[_PboSlot] = []
    self._pbo_free: deque[int] = deque()
    self._pbo_pending: deque[int] = deque()
    self._pbo_byte_count = 0
    if os.environ.get("CLUSTER_ASYNC_NV12_READBACK", "1") != "0":
      self._load_async_functions()

  def _load_async_functions(self) -> None:
    try:
      self._gles.glGenBuffers.argtypes = [ctypes.c_int, ctypes.POINTER(ctypes.c_uint)]
      self._gles.glGenBuffers.restype = None
      self._gles.glDeleteBuffers.argtypes = [ctypes.c_int, ctypes.POINTER(ctypes.c_uint)]
      self._gles.glDeleteBuffers.restype = None
      self._gles.glBufferData.argtypes = [ctypes.c_uint, ctypes.c_ssize_t, ctypes.c_void_p, ctypes.c_uint]
      self._gles.glBufferData.restype = None
      self._gles.glMapBufferRange.argtypes = [ctypes.c_uint, ctypes.c_ssize_t, ctypes.c_ssize_t, ctypes.c_uint]
      self._gles.glMapBufferRange.restype = ctypes.c_void_p
      self._gles.glUnmapBuffer.argtypes = [ctypes.c_uint]
      self._gles.glUnmapBuffer.restype = ctypes.c_ubyte
      self._gles.glFenceSync.argtypes = [ctypes.c_uint, ctypes.c_uint]
      self._gles.glFenceSync.restype = ctypes.c_void_p
      self._gles.glClientWaitSync.argtypes = [ctypes.c_void_p, ctypes.c_uint, ctypes.c_uint64]
      self._gles.glClientWaitSync.restype = ctypes.c_uint
      self._gles.glDeleteSync.argtypes = [ctypes.c_void_p]
      self._gles.glDeleteSync.restype = None
      self._gles.glFlush.argtypes = []
      self._gles.glFlush.restype = None
    except AttributeError:
      return
    self._async_supported = True

  @property
  def async_supported(self) -> bool:
    return self._async_supported

  def async_can_enqueue(self) -> bool:
    return self._async_supported and (not self._pbo_slots or bool(self._pbo_free))

  def _clear_gl_errors(self, operation: str) -> None:
    for _ in range(16):
      if self._gles.glGetError() == GL_NO_ERROR:
        return
    raise DirectNv12ReadbackError(f"GLES error state did not clear before {operation}")

  def _delete_async_resources(self) -> None:
    if not self._pbo_slots:
      self._pbo_free.clear()
      self._pbo_pending.clear()
      self._pbo_byte_count = 0
      return
    for slot in self._pbo_slots:
      if slot.fence is not None:
        self._gles.glDeleteSync(ctypes.c_void_p(slot.fence))
        slot.fence = None
    buffer_ids = (ctypes.c_uint * len(self._pbo_slots))(*(slot.buffer_id for slot in self._pbo_slots))
    self._gles.glDeleteBuffers(len(self._pbo_slots), buffer_ids)
    self._pbo_slots.clear()
    self._pbo_free.clear()
    self._pbo_pending.clear()
    self._pbo_byte_count = 0

  def disable_async(self) -> None:
    self._delete_async_resources()
    self._async_supported = False

  def close(self) -> None:
    self._delete_async_resources()

  def _ensure_async_buffers(self, byte_count: int) -> None:
    if self._pbo_slots and self._pbo_byte_count == byte_count:
      return
    self._delete_async_resources()
    previous_pack_buffer = ctypes.c_int()
    self._gles.glGetIntegerv(GL_PIXEL_PACK_BUFFER_BINDING, ctypes.byref(previous_pack_buffer))
    self._clear_gl_errors("asynchronous NV12 readback setup")

    buffer_ids = (ctypes.c_uint * ASYNC_READBACK_RING_SIZE)()
    try:
      self._gles.glGenBuffers(ASYNC_READBACK_RING_SIZE, buffer_ids)
      for buffer_id in buffer_ids:
        if buffer_id == 0:
          raise DirectNv12ReadbackError("GLES returned an invalid pixel-pack buffer")
        self._gles.glBindBuffer(GL_PIXEL_PACK_BUFFER, int(buffer_id))
        self._gles.glBufferData(GL_PIXEL_PACK_BUFFER, byte_count, None, GL_STREAM_READ)
      setup_error = int(self._gles.glGetError())
    except Exception:
      self._gles.glDeleteBuffers(ASYNC_READBACK_RING_SIZE, buffer_ids)
      raise
    finally:
      self._gles.glBindBuffer(GL_PIXEL_PACK_BUFFER, previous_pack_buffer.value)

    if setup_error != GL_NO_ERROR:
      self._gles.glDeleteBuffers(ASYNC_READBACK_RING_SIZE, buffer_ids)
      raise DirectNv12ReadbackError(f"GLES PBO allocation failed with GL error 0x{setup_error:04x}")
    self._pbo_slots = [_PboSlot(int(buffer_id)) for buffer_id in buffer_ids]
    self._pbo_free = deque(range(len(self._pbo_slots)))
    self._pbo_byte_count = byte_count

  def enqueue_rgba(self, framebuffer: int, width: int, height: int) -> bool:
    if not self._async_supported:
      raise DirectNv12ReadbackError("asynchronous GLES NV12 readback is not available")
    width = int(width)
    height = int(height)
    required_size = width * height * 4
    if framebuffer <= 0 or width <= 0 or height <= 0:
      raise DirectNv12ReadbackError("asynchronous NV12 readback received an invalid render target")
    self._ensure_async_buffers(required_size)
    if not self._pbo_free:
      return False

    previous_framebuffer = ctypes.c_int()
    previous_pack_alignment = ctypes.c_int()
    previous_pack_buffer = ctypes.c_int()
    self._gles.glGetIntegerv(GL_FRAMEBUFFER_BINDING, ctypes.byref(previous_framebuffer))
    self._gles.glGetIntegerv(GL_PACK_ALIGNMENT, ctypes.byref(previous_pack_alignment))
    self._gles.glGetIntegerv(GL_PIXEL_PACK_BUFFER_BINDING, ctypes.byref(previous_pack_buffer))
    self._clear_gl_errors("asynchronous NV12 readback")

    slot_index = self._pbo_free.popleft()
    slot = self._pbo_slots[slot_index]
    readback_error = GL_NO_ERROR
    try:
      self._gles.glBindFramebuffer(GL_FRAMEBUFFER, int(framebuffer))
      self._gles.glBindBuffer(GL_PIXEL_PACK_BUFFER, slot.buffer_id)
      self._gles.glPixelStorei(GL_PACK_ALIGNMENT, 4)
      self._gles.glReadPixels(0, 0, width, height, GL_RGBA, GL_UNSIGNED_BYTE, ctypes.c_void_p())
      fence = self._gles.glFenceSync(GL_SYNC_GPU_COMMANDS_COMPLETE, 0)
      if not fence:
        raise DirectNv12ReadbackError("GLES failed to create an NV12 readback fence")
      slot.fence = int(fence)
      slot.byte_count = required_size
      slot.submitted_s = time.monotonic()
      self._gles.glFlush()
      readback_error = int(self._gles.glGetError())
    except Exception:
      if slot.fence is not None:
        self._gles.glDeleteSync(ctypes.c_void_p(slot.fence))
        slot.fence = None
      self._pbo_free.appendleft(slot_index)
      raise
    finally:
      self._gles.glPixelStorei(GL_PACK_ALIGNMENT, previous_pack_alignment.value)
      self._gles.glBindBuffer(GL_PIXEL_PACK_BUFFER, previous_pack_buffer.value)
      self._gles.glBindFramebuffer(GL_FRAMEBUFFER, previous_framebuffer.value)

    if readback_error != GL_NO_ERROR:
      if slot.fence is not None:
        self._gles.glDeleteSync(ctypes.c_void_p(slot.fence))
        slot.fence = None
      self._pbo_free.appendleft(slot_index)
      raise DirectNv12ReadbackError(f"GLES PBO readback failed with GL error 0x{readback_error:04x}")
    self._pbo_pending.append(slot_index)
    return True

  def async_ready(self) -> bool:
    if not self._pbo_pending:
      return False
    slot = self._pbo_slots[self._pbo_pending[0]]
    if slot.fence is None:
      raise DirectNv12ReadbackError("asynchronous NV12 readback lost its fence")
    result = int(self._gles.glClientWaitSync(ctypes.c_void_p(slot.fence), 0, 0))
    if result in (GL_ALREADY_SIGNALED, GL_CONDITION_SATISFIED):
      return True
    if result == GL_TIMEOUT_EXPIRED:
      if time.monotonic() - slot.submitted_s > ASYNC_READBACK_STALL_TIMEOUT_S:
        raise DirectNv12ReadbackError("asynchronous NV12 readback fence stalled")
      return False
    if result == GL_WAIT_FAILED:
      error = int(self._gles.glGetError())
      raise DirectNv12ReadbackError(f"GLES NV12 fence wait failed with GL error 0x{error:04x}")
    raise DirectNv12ReadbackError(f"GLES NV12 fence returned unexpected status 0x{result:04x}")

  def copy_ready(self, destination_address: int, destination_size: int) -> bool:
    destination_address = int(destination_address)
    destination_size = int(destination_size)
    if not self.async_ready():
      return False
    slot_index = self._pbo_pending[0]
    slot = self._pbo_slots[slot_index]
    if destination_address <= 0 or destination_size < slot.byte_count:
      raise DirectNv12ReadbackError(
        f"asynchronous NV12 destination is {destination_size} bytes, expected at least {slot.byte_count}"
      )

    previous_pack_buffer = ctypes.c_int()
    self._gles.glGetIntegerv(GL_PIXEL_PACK_BUFFER_BINDING, ctypes.byref(previous_pack_buffer))
    self._clear_gl_errors("asynchronous NV12 readback copy")
    mapped = None
    copy_error = GL_NO_ERROR
    try:
      self._gles.glBindBuffer(GL_PIXEL_PACK_BUFFER, slot.buffer_id)
      mapped = self._gles.glMapBufferRange(GL_PIXEL_PACK_BUFFER, 0, slot.byte_count, GL_MAP_READ_BIT)
      if not mapped:
        raise DirectNv12ReadbackError("GLES failed to map a completed NV12 readback")
      ctypes.memmove(destination_address, mapped, slot.byte_count)
      if not self._gles.glUnmapBuffer(GL_PIXEL_PACK_BUFFER):
        raise DirectNv12ReadbackError("GLES reported a corrupted NV12 PBO mapping")
      mapped = None
      copy_error = int(self._gles.glGetError())
    finally:
      if mapped:
        self._gles.glUnmapBuffer(GL_PIXEL_PACK_BUFFER)
      self._gles.glBindBuffer(GL_PIXEL_PACK_BUFFER, previous_pack_buffer.value)

    if copy_error != GL_NO_ERROR:
      raise DirectNv12ReadbackError(f"GLES PBO copy failed with GL error 0x{copy_error:04x}")
    if slot.fence is not None:
      self._gles.glDeleteSync(ctypes.c_void_p(slot.fence))
    slot.fence = None
    slot.byte_count = 0
    slot.submitted_s = 0.0
    self._pbo_pending.popleft()
    self._pbo_free.append(slot_index)
    return True

  def read_rgba(
    self,
    framebuffer: int,
    width: int,
    height: int,
    destination_address: int,
    destination_size: int,
  ) -> None:
    width = int(width)
    height = int(height)
    destination_address = int(destination_address)
    destination_size = int(destination_size)
    required_size = width * height * 4
    if framebuffer <= 0 or width <= 0 or height <= 0:
      raise DirectNv12ReadbackError("direct NV12 readback received an invalid render target")
    if destination_address <= 0 or destination_size < required_size:
      raise DirectNv12ReadbackError(
        f"direct NV12 readback destination is {destination_size} bytes, expected at least {required_size}"
      )

    previous_framebuffer = ctypes.c_int()
    previous_pack_alignment = ctypes.c_int()
    previous_pack_buffer = ctypes.c_int()
    self._gles.glGetIntegerv(GL_FRAMEBUFFER_BINDING, ctypes.byref(previous_framebuffer))
    self._gles.glGetIntegerv(GL_PACK_ALIGNMENT, ctypes.byref(previous_pack_alignment))
    self._gles.glGetIntegerv(GL_PIXEL_PACK_BUFFER_BINDING, ctypes.byref(previous_pack_buffer))
    self._clear_gl_errors("direct NV12 readback")

    readback_error = GL_NO_ERROR
    try:
      self._gles.glBindFramebuffer(GL_FRAMEBUFFER, int(framebuffer))
      self._gles.glBindBuffer(GL_PIXEL_PACK_BUFFER, 0)
      self._gles.glPixelStorei(GL_PACK_ALIGNMENT, 4)
      self._gles.glReadPixels(
        0,
        0,
        width,
        height,
        GL_RGBA,
        GL_UNSIGNED_BYTE,
        ctypes.c_void_p(destination_address),
      )
      readback_error = int(self._gles.glGetError())
    finally:
      self._gles.glPixelStorei(GL_PACK_ALIGNMENT, previous_pack_alignment.value)
      self._gles.glBindBuffer(GL_PIXEL_PACK_BUFFER, previous_pack_buffer.value)
      self._gles.glBindFramebuffer(GL_FRAMEBUFFER, previous_framebuffer.value)

    if readback_error != GL_NO_ERROR:
      raise DirectNv12ReadbackError(f"GLES direct NV12 readback failed with GL error 0x{readback_error:04x}")


def create_tici_direct_readback() -> GlesDirectReadback | None:
  if os.environ.get("CLUSTER_DIRECT_NV12_READBACK", "1") == "0" or not Path("/TICI").is_file():
    return None
  return GlesDirectReadback()
