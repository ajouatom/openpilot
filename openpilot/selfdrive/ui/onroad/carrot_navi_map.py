from __future__ import annotations

from collections import OrderedDict
from collections.abc import Callable
from dataclasses import dataclass
import time

import pyray as rl

from openpilot.selfdrive.carrot.cluster.cluster_models import NaviMediaFrame
from openpilot.selfdrive.carrot.cluster.cluster_navi_source import NaviIpcMediaSource
from openpilot.common.params import Params
from openpilot.system.ui.lib.egl import (
  bind_egl_image_to_texture,
  create_egl_image,
  destroy_egl_image,
  init_egl,
)


NAVI_YUV_VERTEX_SHADER = """
attribute vec3 vertexPosition;
attribute vec2 vertexTexCoord;
attribute vec4 vertexColor;
varying vec2 fragTexCoord;
uniform mat4 mvp;
void main() {
  fragTexCoord = vertexTexCoord;
  gl_Position = mvp * vec4(vertexPosition, 1.0);
}
"""

NAVI_YUV_FRAGMENT_SHADER = """
#ifdef GL_ES
precision mediump float;
#endif
varying vec2 fragTexCoord;
uniform sampler2D texture0;
uniform sampler2D texture1;
uniform sampler2D texture2;
uniform vec2 uvScaleU;
uniform vec2 uvScaleV;
void main() {
  float y = 1.164383 * (texture2D(texture0, fragTexCoord).r - 0.062745);
  float u = texture2D(texture1, fragTexCoord * uvScaleU).r - 0.501961;
  float v = texture2D(texture2, fragTexCoord * uvScaleV).r - 0.501961;
  vec3 rgb = vec3(
    y + 1.596027 * v,
    y - 0.391762 * u - 0.812968 * v,
    y + 2.017232 * u
  );
  gl_FragColor = vec4(clamp(rgb, 0.0, 1.0), 1.0);
}
"""

NAVI_EXTERNAL_VERTEX_SHADER = """
#version 300 es
precision mediump float;
in vec3 vertexPosition;
in vec2 vertexTexCoord;
in vec4 vertexColor;
uniform mat4 mvp;
out vec2 fragTexCoord;
void main() {
  fragTexCoord = vertexTexCoord;
  gl_Position = mvp * vec4(vertexPosition, 1.0);
}
"""

NAVI_EXTERNAL_FRAGMENT_SHADER = """
#version 300 es
#extension GL_OES_EGL_image_external_essl3 : enable
precision mediump float;
in vec2 fragTexCoord;
uniform samplerExternalOES texture0;
out vec4 fragColor;
void main() {
  fragColor = texture(texture0, fragTexCoord);
}
"""

NAVI_EGL_IMAGE_CACHE_LIMIT = 10


@dataclass(slots=True)
class CachedNaviTexture:
  sequence: int
  size: tuple[int, int]
  mime: str
  texture: object
  u_texture: object | None = None
  v_texture: object | None = None
  uv_scale_u: object | None = None
  uv_scale_v: object | None = None
  hardware_token: tuple[int, int] | None = None


def map_frame_from_dashboard(dashboard) -> NaviMediaFrame | None:
  if dashboard is None or not dashboard.connected or dashboard.map_stream_stalled:
    return None
  return next(
    (frame for frame in dashboard.media if frame.key == "render:map_main" and frame.present),
    None,
  )


class NaviMapTextureRenderer:
  def __init__(self) -> None:
    self._cached: CachedNaviTexture | None = None
    self._yuv_shader = None
    self._yuv_locations: dict[str, int] = {}
    self._external_shader = None
    self._egl_images: OrderedDict[tuple[int, int], object] = OrderedDict()

  def draw(self, frame: NaviMediaFrame, rect: rl.Rectangle) -> bool:
    cached = self._texture_for(frame)
    if cached is None or cached.size[0] <= 0 or cached.size[1] <= 0:
      return False

    source_w, source_h = map(float, cached.size)
    scale = min(rect.width / source_w, rect.height / source_h)
    draw_w = source_w * scale
    draw_h = source_h * scale
    source = rl.Rectangle(0.0, 0.0, source_w, source_h)
    dest = rl.Rectangle(
      rect.x + (rect.width - draw_w) * 0.5,
      rect.y + (rect.height - draw_h) * 0.5,
      draw_w,
      draw_h,
    )
    rl.draw_rectangle_rec(rect, rl.BLACK)
    self._draw_cached(cached, source, dest)
    return True

  def _texture_for(self, frame: NaviMediaFrame) -> CachedNaviTexture | None:
    cached = self._cached
    has_payload = frame.hardware_buffer is not None or frame.data is not None or (
      frame.mime == "image/yuv420p" and frame.plane_data is not None and frame.plane_strides is not None
    )
    if not frame.present or not has_payload:
      self._clear_cached()
      return None
    if cached is not None and cached.sequence == frame.sequence and cached.mime == frame.mime:
      return cached
    if frame.mime == "video/nv12-dmabuf":
      return self._upload_hardware(frame)
    if frame.mime == "image/yuv420p":
      return self._upload_yuv420(frame)
    return self._upload_image(frame)

  def _upload_hardware(self, frame: NaviMediaFrame) -> CachedNaviTexture | None:
    hardware_buffer = frame.hardware_buffer
    if hardware_buffer is None:
      return self._cached
    width, height = int(frame.width), int(frame.height)
    stride = int(getattr(hardware_buffer, "stride", 0))
    uv_offset = int(getattr(hardware_buffer, "uv_offset", 0))
    fd = int(getattr(hardware_buffer, "fd", -1))
    token = tuple(getattr(hardware_buffer, "token", ()))
    if width <= 0 or height <= 0 or stride < width or uv_offset < stride * height or fd < 0 or len(token) != 2:
      return self._cached

    try:
      if not init_egl():
        raise RuntimeError("EGL DMA-BUF import is unavailable")
      normalized_token = (int(token[0]), int(token[1]))
      self._discard_other_generations(normalized_token[0])
      egl_image = self._egl_images.get(normalized_token)
      if egl_image is None:
        egl_image = create_egl_image(width, height, stride, fd, uv_offset)
        if egl_image is None:
          raise RuntimeError("EGL rejected the decoded navigation buffer")
        self._egl_images[normalized_token] = egl_image
      else:
        self._egl_images.move_to_end(normalized_token)

      cached = self._cached
      reuse = cached is not None and cached.mime == frame.mime and cached.size == (width, height)
      texture = cached.texture if reuse else self._new_external_texture(width, height)
      texture.width = width
      texture.height = height
      bind_egl_image_to_texture(texture.id, egl_image)
      replacement = CachedNaviTexture(
        frame.sequence, (width, height), frame.mime, texture, hardware_token=normalized_token,
      )
      if cached is not None and cached.texture is not texture:
        self._unload_cached(cached)
      self._cached = replacement
      self._evict_egl_images()
      return replacement
    except Exception as exc:
      mark_failed = getattr(hardware_buffer, "mark_egl_import_failed", None)
      if mark_failed is not None:
        mark_failed(str(exc))
      return self._cached

  @staticmethod
  def _new_external_texture(width: int, height: int):
    image = rl.gen_image_color(1, 1, rl.BLACK)
    try:
      texture = rl.load_texture_from_image(image)
    finally:
      rl.unload_image(image)
    if not rl.is_texture_valid(texture):
      raise RuntimeError("failed to allocate navigation texture")
    texture.width = width
    texture.height = height
    rl.set_texture_filter(texture, rl.TextureFilter.TEXTURE_FILTER_BILINEAR)
    return texture

  def _upload_yuv420(self, frame: NaviMediaFrame) -> CachedNaviTexture | None:
    if frame.plane_data is None or frame.plane_strides is None:
      return self._cached
    width, height = int(frame.width), int(frame.height)
    y_stride, u_stride, v_stride = map(int, frame.plane_strides)
    chroma_width, chroma_height = width // 2, height // 2
    if width <= 0 or height <= 0 or width & 1 or height & 1:
      return self._cached
    if y_stride < width or u_stride < chroma_width or v_stride < chroma_width:
      return self._cached
    shapes = ((y_stride, height), (u_stride, chroma_height), (v_stride, chroma_height))
    if any(len(data) < stride * rows for data, (stride, rows) in zip(frame.plane_data, shapes, strict=True)):
      return self._cached

    cached = self._cached
    reuse = bool(
      cached is not None and cached.mime == frame.mime and cached.size == (width, height)
      and cached.texture.width == y_stride and cached.texture.height == height
      and cached.u_texture is not None and cached.u_texture.width == u_stride
      and cached.v_texture is not None and cached.v_texture.width == v_stride
    )
    replacement = cached
    if not reuse:
      try:
        replacement = CachedNaviTexture(
          frame.sequence,
          (width, height),
          frame.mime,
          self._new_plane_texture(y_stride, height, rl.TextureFilter.TEXTURE_FILTER_BILINEAR),
          self._new_plane_texture(u_stride, chroma_height, rl.TextureFilter.TEXTURE_FILTER_POINT),
          self._new_plane_texture(v_stride, chroma_height, rl.TextureFilter.TEXTURE_FILTER_POINT),
          rl.ffi.new("float[]", [y_stride / (2.0 * u_stride), 1.0]),
          rl.ffi.new("float[]", [y_stride / (2.0 * v_stride), 1.0]),
        )
      except Exception:
        if replacement is not cached and replacement is not None:
          self._unload_cached(replacement)
        return cached

    if replacement is None or replacement.u_texture is None or replacement.v_texture is None:
      return cached
    try:
      for texture, plane in zip(
        (replacement.texture, replacement.u_texture, replacement.v_texture), frame.plane_data, strict=True,
      ):
        pixels = rl.ffi.cast("void *", rl.ffi.from_buffer("const unsigned char[]", plane))
        rl.update_texture(texture, pixels)
    except Exception:
      if replacement is not cached:
        self._unload_cached(replacement)
      return cached

    replacement.sequence = frame.sequence
    if replacement is not cached:
      if cached is not None:
        self._unload_cached(cached)
      self._cached = replacement
    return replacement

  def _upload_image(self, frame: NaviMediaFrame) -> CachedNaviTexture | None:
    if frame.data is None or frame.width <= 0 or frame.height <= 0:
      return self._cached
    cached = self._cached
    size = (int(frame.width), int(frame.height))
    if frame.mime == "image/rgba":
      if len(frame.data) != size[0] * size[1] * 4:
        return cached
      if cached is None or cached.mime != frame.mime or cached.size != size:
        image = rl.gen_image_color(size[0], size[1], rl.BLACK)
        texture = rl.load_texture_from_image(image)
        rl.unload_image(image)
      else:
        texture = cached.texture
      pixels = rl.ffi.cast("void *", rl.ffi.from_buffer("const unsigned char[]", frame.data))
      rl.update_texture(texture, pixels)
    else:
      extension = ".jpg" if "jpeg" in frame.mime else ".png"
      image = rl.load_image_from_memory(extension, frame.data, len(frame.data))
      if not rl.is_image_valid(image):
        return cached
      try:
        texture = rl.load_texture_from_image(image)
      finally:
        rl.unload_image(image)
    if not rl.is_texture_valid(texture):
      return cached
    rl.set_texture_filter(texture, rl.TextureFilter.TEXTURE_FILTER_BILINEAR)
    replacement = CachedNaviTexture(frame.sequence, size, frame.mime, texture)
    if cached is not None and cached.texture is not texture:
      self._unload_cached(cached)
    self._cached = replacement
    return replacement

  @staticmethod
  def _new_plane_texture(width: int, height: int, texture_filter):
    image = rl.Image(None, width, height, 1, rl.PixelFormat.PIXELFORMAT_UNCOMPRESSED_GRAYSCALE)
    texture = rl.load_texture_from_image(image)
    if not rl.is_texture_valid(texture):
      raise RuntimeError(f"failed to allocate navigation plane {width}x{height}")
    rl.set_texture_filter(texture, texture_filter)
    return texture

  def _draw_cached(self, cached: CachedNaviTexture, source: rl.Rectangle, dest: rl.Rectangle) -> None:
    if cached.mime == "video/nv12-dmabuf":
      if cached.hardware_token not in self._egl_images:
        return
      shader = self._get_external_shader()
      rl.begin_shader_mode(shader)
      try:
        bind_egl_image_to_texture(cached.texture.id, self._egl_images[cached.hardware_token])
        rl.draw_texture_pro(cached.texture, source, dest, rl.Vector2(0.0, 0.0), 0.0, rl.WHITE)
      finally:
        rl.end_shader_mode()
      return
    if cached.mime != "image/yuv420p":
      rl.draw_texture_pro(cached.texture, source, dest, rl.Vector2(0.0, 0.0), 0.0, rl.WHITE)
      return
    if cached.u_texture is None or cached.v_texture is None:
      return
    shader = self._get_yuv_shader()
    rl.begin_shader_mode(shader)
    try:
      rl.set_shader_value_texture(shader, self._yuv_locations["texture1"], cached.u_texture)
      rl.set_shader_value_texture(shader, self._yuv_locations["texture2"], cached.v_texture)
      rl.set_shader_value(shader, self._yuv_locations["uvScaleU"], cached.uv_scale_u,
                          rl.ShaderUniformDataType.SHADER_UNIFORM_VEC2)
      rl.set_shader_value(shader, self._yuv_locations["uvScaleV"], cached.uv_scale_v,
                          rl.ShaderUniformDataType.SHADER_UNIFORM_VEC2)
      rl.draw_texture_pro(cached.texture, source, dest, rl.Vector2(0.0, 0.0), 0.0, rl.WHITE)
    finally:
      rl.end_shader_mode()

  def _get_yuv_shader(self):
    if self._yuv_shader is None:
      self._yuv_shader = rl.load_shader_from_memory(NAVI_YUV_VERTEX_SHADER, NAVI_YUV_FRAGMENT_SHADER)
      if not rl.is_shader_valid(self._yuv_shader):
        raise RuntimeError("failed to load navigation YUV shader")
      self._yuv_locations = {
        name: rl.get_shader_location(self._yuv_shader, name)
        for name in ("texture1", "texture2", "uvScaleU", "uvScaleV")
      }
    return self._yuv_shader

  def _get_external_shader(self):
    if self._external_shader is None:
      self._external_shader = rl.load_shader_from_memory(NAVI_EXTERNAL_VERTEX_SHADER, NAVI_EXTERNAL_FRAGMENT_SHADER)
      if not rl.is_shader_valid(self._external_shader):
        raise RuntimeError("failed to load navigation external shader")
    return self._external_shader

  def _discard_other_generations(self, generation: int) -> None:
    for token in tuple(self._egl_images):
      if token[0] != generation:
        destroy_egl_image(self._egl_images.pop(token))

  def _evict_egl_images(self) -> None:
    while len(self._egl_images) > NAVI_EGL_IMAGE_CACHE_LIMIT:
      token, image = next(iter(self._egl_images.items()))
      if self._cached is not None and token == self._cached.hardware_token:
        self._egl_images.move_to_end(token)
        continue
      self._egl_images.pop(token)
      destroy_egl_image(image)

  @staticmethod
  def _unload_cached(cached: CachedNaviTexture) -> None:
    for texture in (cached.texture, cached.u_texture, cached.v_texture):
      if texture is not None:
        rl.unload_texture(texture)

  def _clear_cached(self) -> None:
    if self._cached is not None:
      self._unload_cached(self._cached)
      self._cached = None

  def close(self) -> None:
    self._clear_cached()
    for image in self._egl_images.values():
      destroy_egl_image(image)
    self._egl_images.clear()
    if self._external_shader is not None:
      rl.unload_shader(self._external_shader)
      self._external_shader = None
    if self._yuv_shader is not None:
      rl.unload_shader(self._yuv_shader)
      self._yuv_shader = None
      self._yuv_locations.clear()


class CarrotNaviMapView:
  def __init__(
    self,
    source_factory: Callable[..., NaviIpcMediaSource] = NaviIpcMediaSource,
    renderer_factory: Callable[[], NaviMapTextureRenderer] = NaviMapTextureRenderer,
    bootstrap_requester: Callable[[], None] | None = None,
  ) -> None:
    self._source_factory = source_factory
    self._renderer_factory = renderer_factory
    self._source: NaviIpcMediaSource | None = None
    self._renderer = renderer_factory()
    self._bootstrap_requester = bootstrap_requester or self._request_media_bootstrap
    self._enabled = False

  def render(self, rect: rl.Rectangle, enabled: bool) -> bool:
    enabled = bool(enabled)
    if not enabled and self._source is None:
      return False
    if self._source is None:
      self._source = self._source_factory(decode_media=False)
    if enabled != self._enabled:
      self._source.set_decode_media(enabled)
      self._enabled = enabled
      if enabled:
        self._bootstrap_requester()
      else:
        self._source.close()
        self._source = None
        self._renderer.close()
        self._renderer = self._renderer_factory()
        return False
    dashboard = self._source.update(None)
    if not enabled:
      return False
    frame = map_frame_from_dashboard(dashboard)
    return frame is not None and self._renderer.draw(frame, rect)

  def close(self) -> None:
    if self._source is not None:
      self._source.close()
      self._source = None
    self._enabled = False
    self._renderer.close()

  @staticmethod
  def _request_media_bootstrap() -> None:
    try:
      Params().put_nonblocking("CarrotNaviWebBootstrapRequest", f"ui-{time.time_ns():x}")
    except Exception:
      pass
