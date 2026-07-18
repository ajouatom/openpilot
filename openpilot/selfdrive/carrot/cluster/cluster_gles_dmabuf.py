from __future__ import annotations

import ctypes
import os
from dataclasses import dataclass
from pathlib import Path

import pyray as rl


EGL_EXTENSIONS = 0x3055
EGL_LINUX_DMA_BUF_EXT = 0x3270
EGL_WIDTH = 0x3057
EGL_HEIGHT = 0x3056
EGL_LINUX_DRM_FOURCC_EXT = 0x3271
EGL_DMA_BUF_PLANE0_FD_EXT = 0x3272
EGL_DMA_BUF_PLANE0_OFFSET_EXT = 0x3273
EGL_DMA_BUF_PLANE0_PITCH_EXT = 0x3274
EGL_NONE = 0x3038

GL_TEXTURE_2D = 0x0DE1
GL_TEXTURE_BINDING_2D = 0x8069
GL_TEXTURE_MIN_FILTER = 0x2801
GL_TEXTURE_MAG_FILTER = 0x2800
GL_TEXTURE_WRAP_S = 0x2802
GL_TEXTURE_WRAP_T = 0x2803
GL_NEAREST = 0x2600
GL_CLAMP_TO_EDGE = 0x812F
GL_FRAMEBUFFER = 0x8D40
GL_FRAMEBUFFER_BINDING = 0x8CA6
GL_COLOR_ATTACHMENT0 = 0x8CE0
GL_FRAMEBUFFER_COMPLETE = 0x8CD5
GL_NO_ERROR = 0
GL_SYNC_GPU_COMMANDS_COMPLETE = 0x9117
GL_SYNC_FLUSH_COMMANDS_BIT = 0x00000001
GL_ALREADY_SIGNALED = 0x911A
GL_TIMEOUT_EXPIRED = 0x911B
GL_CONDITION_SATISFIED = 0x911C
GL_WAIT_FAILED = 0x911D

DRM_FORMAT_ABGR8888 = ord("A") | ord("B") << 8 | ord("2") << 16 | ord("4") << 24
GPU_WAIT_TIMEOUT_NS = 50_000_000


class DirectNv12DmabufError(RuntimeError):
  pass


@dataclass
class _ImportedTarget:
  render_texture: object
  egl_image: int
  import_fd: int
  texture_id: int
  framebuffer_id: int


class GlesDmabufRenderTargetPool:
  """Imports encoder DMA-BUFs as packed RGBA render targets."""

  def __init__(self) -> None:
    try:
      self._egl = ctypes.CDLL("libEGL.so")
      self._gles = ctypes.CDLL("libGLESv2.so")
    except OSError as exc:
      raise DirectNv12DmabufError(f"failed to load EGL/GLES libraries: {exc}") from exc

    self._configure_functions()
    self._display = self._egl.eglGetCurrentDisplay()
    if not self._display:
      raise DirectNv12DmabufError("EGL has no current display")
    extension_bytes = self._egl.eglQueryString(self._display, EGL_EXTENSIONS)
    extensions = extension_bytes.decode("ascii", errors="ignore") if extension_bytes else ""
    if "EGL_EXT_image_dma_buf_import" not in extensions:
      raise DirectNv12DmabufError("EGL_EXT_image_dma_buf_import is unavailable")

    create_address = self._egl.eglGetProcAddress(b"eglCreateImageKHR")
    destroy_address = self._egl.eglGetProcAddress(b"eglDestroyImageKHR")
    image_target_address = self._egl.eglGetProcAddress(b"glEGLImageTargetTexture2DOES")
    if not create_address or not destroy_address or not image_target_address:
      raise DirectNv12DmabufError("required EGLImage functions are unavailable")
    create_type = ctypes.CFUNCTYPE(
      ctypes.c_void_p,
      ctypes.c_void_p,
      ctypes.c_void_p,
      ctypes.c_uint,
      ctypes.c_void_p,
      ctypes.POINTER(ctypes.c_int),
    )
    destroy_type = ctypes.CFUNCTYPE(ctypes.c_uint, ctypes.c_void_p, ctypes.c_void_p)
    image_target_type = ctypes.CFUNCTYPE(None, ctypes.c_uint, ctypes.c_void_p)
    self._create_image = create_type(create_address)
    self._destroy_image = destroy_type(destroy_address)
    self._image_target = image_target_type(image_target_address)
    self._targets: dict[tuple[int, int, int], _ImportedTarget] = {}

  def _configure_functions(self) -> None:
    self._egl.eglGetCurrentDisplay.argtypes = []
    self._egl.eglGetCurrentDisplay.restype = ctypes.c_void_p
    self._egl.eglQueryString.argtypes = [ctypes.c_void_p, ctypes.c_int]
    self._egl.eglQueryString.restype = ctypes.c_char_p
    self._egl.eglGetProcAddress.argtypes = [ctypes.c_char_p]
    self._egl.eglGetProcAddress.restype = ctypes.c_void_p
    self._egl.eglGetError.argtypes = []
    self._egl.eglGetError.restype = ctypes.c_uint

    self._gles.glGetIntegerv.argtypes = [ctypes.c_uint, ctypes.POINTER(ctypes.c_int)]
    self._gles.glGetIntegerv.restype = None
    self._gles.glGenTextures.argtypes = [ctypes.c_int, ctypes.POINTER(ctypes.c_uint)]
    self._gles.glGenTextures.restype = None
    self._gles.glDeleteTextures.argtypes = [ctypes.c_int, ctypes.POINTER(ctypes.c_uint)]
    self._gles.glDeleteTextures.restype = None
    self._gles.glBindTexture.argtypes = [ctypes.c_uint, ctypes.c_uint]
    self._gles.glBindTexture.restype = None
    self._gles.glTexParameteri.argtypes = [ctypes.c_uint, ctypes.c_uint, ctypes.c_int]
    self._gles.glTexParameteri.restype = None
    self._gles.glGenFramebuffers.argtypes = [ctypes.c_int, ctypes.POINTER(ctypes.c_uint)]
    self._gles.glGenFramebuffers.restype = None
    self._gles.glDeleteFramebuffers.argtypes = [ctypes.c_int, ctypes.POINTER(ctypes.c_uint)]
    self._gles.glDeleteFramebuffers.restype = None
    self._gles.glBindFramebuffer.argtypes = [ctypes.c_uint, ctypes.c_uint]
    self._gles.glBindFramebuffer.restype = None
    self._gles.glFramebufferTexture2D.argtypes = [
      ctypes.c_uint,
      ctypes.c_uint,
      ctypes.c_uint,
      ctypes.c_uint,
      ctypes.c_int,
    ]
    self._gles.glFramebufferTexture2D.restype = None
    self._gles.glCheckFramebufferStatus.argtypes = [ctypes.c_uint]
    self._gles.glCheckFramebufferStatus.restype = ctypes.c_uint
    self._gles.glFenceSync.argtypes = [ctypes.c_uint, ctypes.c_uint]
    self._gles.glFenceSync.restype = ctypes.c_void_p
    self._gles.glClientWaitSync.argtypes = [ctypes.c_void_p, ctypes.c_uint, ctypes.c_uint64]
    self._gles.glClientWaitSync.restype = ctypes.c_uint
    self._gles.glDeleteSync.argtypes = [ctypes.c_void_p]
    self._gles.glDeleteSync.restype = None
    self._gles.glGetError.argtypes = []
    self._gles.glGetError.restype = ctypes.c_uint

  def _clear_gl_errors(self, operation: str) -> None:
    for _ in range(16):
      if self._gles.glGetError() == GL_NO_ERROR:
        return
    raise DirectNv12DmabufError(f"GLES error state did not clear before {operation}")

  def target_for(self, dmabuf_fd: int, stride: int, byte_count: int):
    dmabuf_fd = int(dmabuf_fd)
    stride = int(stride)
    byte_count = int(byte_count)
    if dmabuf_fd < 0 or stride <= 0 or stride % 4 != 0 or byte_count <= 0 or byte_count % stride != 0:
      raise DirectNv12DmabufError("encoder DMA-BUF has an invalid packed render layout")
    key = (dmabuf_fd, stride, byte_count)
    cached = self._targets.get(key)
    if cached is not None:
      return cached.render_texture

    width = stride // 4
    height = byte_count // stride
    try:
      import_fd = os.dup(dmabuf_fd)
    except OSError as exc:
      raise DirectNv12DmabufError(f"failed to duplicate encoder DMA-BUF: {exc}") from exc
    attributes = (ctypes.c_int * 13)(
      EGL_WIDTH,
      width,
      EGL_HEIGHT,
      height,
      EGL_LINUX_DRM_FOURCC_EXT,
      DRM_FORMAT_ABGR8888,
      EGL_DMA_BUF_PLANE0_FD_EXT,
      import_fd,
      EGL_DMA_BUF_PLANE0_OFFSET_EXT,
      0,
      EGL_DMA_BUF_PLANE0_PITCH_EXT,
      stride,
      EGL_NONE,
    )
    egl_image = self._create_image(self._display, None, EGL_LINUX_DMA_BUF_EXT, None, attributes)
    if not egl_image:
      error = int(self._egl.eglGetError())
      os.close(import_fd)
      raise DirectNv12DmabufError(f"EGL rejected encoder DMA-BUF with error 0x{error:04x}")

    previous_texture = ctypes.c_int()
    previous_framebuffer = ctypes.c_int()
    texture = ctypes.c_uint()
    framebuffer = ctypes.c_uint()
    self._gles.glGetIntegerv(GL_TEXTURE_BINDING_2D, ctypes.byref(previous_texture))
    self._gles.glGetIntegerv(GL_FRAMEBUFFER_BINDING, ctypes.byref(previous_framebuffer))
    self._clear_gl_errors("encoder DMA-BUF import")
    try:
      self._gles.glGenTextures(1, ctypes.byref(texture))
      self._gles.glBindTexture(GL_TEXTURE_2D, texture.value)
      self._image_target(GL_TEXTURE_2D, egl_image)
      self._gles.glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_NEAREST)
      self._gles.glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_NEAREST)
      self._gles.glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_S, GL_CLAMP_TO_EDGE)
      self._gles.glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_T, GL_CLAMP_TO_EDGE)
      self._gles.glGenFramebuffers(1, ctypes.byref(framebuffer))
      self._gles.glBindFramebuffer(GL_FRAMEBUFFER, framebuffer.value)
      self._gles.glFramebufferTexture2D(
        GL_FRAMEBUFFER,
        GL_COLOR_ATTACHMENT0,
        GL_TEXTURE_2D,
        texture.value,
        0,
      )
      status = int(self._gles.glCheckFramebufferStatus(GL_FRAMEBUFFER))
      gl_error = int(self._gles.glGetError())
      if status != GL_FRAMEBUFFER_COMPLETE or gl_error != GL_NO_ERROR:
        raise DirectNv12DmabufError(
          f"encoder DMA-BUF framebuffer failed status=0x{status:04x} error=0x{gl_error:04x}"
        )
    except Exception:
      if framebuffer.value:
        self._gles.glDeleteFramebuffers(1, ctypes.byref(framebuffer))
      if texture.value:
        self._gles.glDeleteTextures(1, ctypes.byref(texture))
      self._destroy_image(self._display, egl_image)
      try:
        os.close(import_fd)
      except OSError:
        pass
      raise
    finally:
      self._gles.glBindTexture(GL_TEXTURE_2D, previous_texture.value)
      self._gles.glBindFramebuffer(GL_FRAMEBUFFER, previous_framebuffer.value)

    texture_value = rl.Texture(
      int(texture.value),
      width,
      height,
      1,
      rl.PixelFormat.PIXELFORMAT_UNCOMPRESSED_R8G8B8A8,
    )
    depth_value = rl.Texture(0, 0, 0, 1, rl.PixelFormat.PIXELFORMAT_UNCOMPRESSED_R8G8B8A8)
    render_texture = rl.RenderTexture(int(framebuffer.value), texture_value, depth_value)
    self._targets[key] = _ImportedTarget(
      render_texture=render_texture,
      egl_image=int(egl_image),
      import_fd=import_fd,
      texture_id=int(texture.value),
      framebuffer_id=int(framebuffer.value),
    )
    return render_texture

  def wait_for_gpu(self) -> None:
    self._clear_gl_errors("encoder DMA-BUF fence")
    fence = self._gles.glFenceSync(GL_SYNC_GPU_COMMANDS_COMPLETE, 0)
    if not fence:
      raise DirectNv12DmabufError("GLES failed to create an encoder DMA-BUF fence")
    try:
      result = int(
        self._gles.glClientWaitSync(
          fence,
          GL_SYNC_FLUSH_COMMANDS_BIT,
          GPU_WAIT_TIMEOUT_NS,
        )
      )
      if result not in (GL_ALREADY_SIGNALED, GL_CONDITION_SATISFIED):
        error = int(self._gles.glGetError())
        if result == GL_TIMEOUT_EXPIRED:
          raise DirectNv12DmabufError("encoder DMA-BUF fence timed out")
        if result == GL_WAIT_FAILED:
          raise DirectNv12DmabufError(f"encoder DMA-BUF fence failed with GL error 0x{error:04x}")
        raise DirectNv12DmabufError(f"encoder DMA-BUF fence returned 0x{result:04x}")
    finally:
      self._gles.glDeleteSync(fence)

  def close(self) -> None:
    for target in self._targets.values():
      framebuffer = ctypes.c_uint(target.framebuffer_id)
      texture = ctypes.c_uint(target.texture_id)
      self._gles.glDeleteFramebuffers(1, ctypes.byref(framebuffer))
      self._gles.glDeleteTextures(1, ctypes.byref(texture))
      self._destroy_image(self._display, ctypes.c_void_p(target.egl_image))
      try:
        os.close(target.import_fd)
      except OSError:
        pass
    self._targets.clear()


def create_tici_nv12_dmabuf_pool() -> GlesDmabufRenderTargetPool | None:
  if os.environ.get("CLUSTER_NV12_DMABUF_OUTPUT", "1") == "0" or not Path("/TICI").is_file():
    return None
  return GlesDmabufRenderTargetPool()
