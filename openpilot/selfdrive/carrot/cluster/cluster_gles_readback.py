from __future__ import annotations

import ctypes
import os
from pathlib import Path


GL_FRAMEBUFFER = 0x8D40
GL_FRAMEBUFFER_BINDING = 0x8CA6
GL_PACK_ALIGNMENT = 0x0D05
GL_PIXEL_PACK_BUFFER = 0x88EB
GL_PIXEL_PACK_BUFFER_BINDING = 0x88ED
GL_RGBA = 0x1908
GL_UNSIGNED_BYTE = 0x1401
GL_NO_ERROR = 0


class DirectNv12ReadbackError(RuntimeError):
  pass


class GlesDirectReadback:
  """Reads a packed RGBA render target directly into a caller-owned TICI buffer."""

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
    for _ in range(16):
      if self._gles.glGetError() == GL_NO_ERROR:
        break
    else:
      raise DirectNv12ReadbackError("GLES error state did not clear before direct NV12 readback")

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
