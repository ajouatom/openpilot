from __future__ import annotations

import time
from pathlib import Path

import pyray as rl


VERTEX_SHADER = """
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

EXTERNAL_FRAGMENT_SHADER = """
#version 300 es
#extension GL_OES_EGL_image_external_essl3 : enable
precision mediump float;
in vec2 fragTexCoord;
uniform samplerExternalOES texture0;
out vec4 fragColor;
void main() {
  vec3 rgb = texture(texture0, fragTexCoord).rgb;
  rgb = clamp((rgb - 0.5) * 1.08 + 0.5, 0.0, 1.0);
  fragColor = vec4(rgb, 1.0);
}
"""

NV12_FRAGMENT_SHADER = """
#version 300 es
precision mediump float;
in vec2 fragTexCoord;
uniform sampler2D texture0;
uniform sampler2D texture1;
out vec4 fragColor;
void main() {
  float y = texture(texture0, fragTexCoord).r;
  vec2 uv = texture(texture1, fragTexCoord).ra - 0.5;
  vec3 rgb = vec3(
    y + 1.402 * uv.y,
    y - 0.344 * uv.x - 0.714 * uv.y,
    y + 1.772 * uv.x
  );
  rgb = clamp((rgb - 0.5) * 1.08 + 0.5, 0.0, 1.0);
  fragColor = vec4(rgb, 1.0);
}
"""

CONNECTION_RETRY_SECONDS = 0.5
FRAME_POLL_SECONDS = 0.1
FRAME_STALE_SECONDS = 1.2
EGL_IMPORT_FAILURE_LIMIT = 3
STREAM_DISCOVERY_SECONDS = 1.0


class LiveRoadCamera:
    """Low-overhead camerad road stream renderer for Linux devices."""

    def __init__(self) -> None:
        from msgq.visionipc import VisionIpcClient, VisionStreamType

        self._zero_copy = Path("/TICI").is_file()
        if self._zero_copy:
            from openpilot.system.ui.lib.egl import init_egl

            if not init_egl():
                print("Cluster road camera EGL initialization failed; using NV12 GPU upload", flush=True)
                self._zero_copy = False

        self._client_cls = VisionIpcClient
        self._stream_type = VisionStreamType.VISION_STREAM_ROAD
        self._road_stream_type = VisionStreamType.VISION_STREAM_ROAD
        self._wide_stream_type = VisionStreamType.VISION_STREAM_WIDE_ROAD
        self._client = self._new_client()
        self._target_client = None
        self._target_stream_type = None
        self._available_streams: set[object] = set()
        self._last_stream_discovery = 0.0
        self._frame = None
        self._last_connection_attempt = 0.0
        self._connected_at = 0.0
        self._last_frame_poll = 0.0
        self._last_frame_at = 0.0
        self._connection_wait_logged = False
        self._texture_needs_update = False
        self._egl_import_failures = 0
        self._egl_images: dict[int, object] = {}
        fragment_shader = EXTERNAL_FRAGMENT_SHADER if self._zero_copy else NV12_FRAGMENT_SHADER
        self._shader = rl.load_shader_from_memory(VERTEX_SHADER, fragment_shader)
        self._texture = None
        self._texture_y = None
        self._texture_uv = None
        self._texture_shape: tuple[int, int] | None = None
        self._texture_uv_location = -1 if self._zero_copy else rl.get_shader_location(self._shader, "texture1")
        if self._zero_copy:
            image = rl.gen_image_color(1, 1, rl.BLACK)
            try:
                self._texture = rl.load_texture_from_image(image)
            finally:
                rl.unload_image(image)
            rl.set_texture_filter(self._texture, rl.TextureFilter.TEXTURE_FILTER_BILINEAR)

    def _new_client(self):
        return self._client_cls("camerad", self._stream_type, conflate=True)

    @property
    def is_wide(self) -> bool:
        return self._stream_type == self._wide_stream_type

    def select_stream(self, prefer_wide: bool) -> bool:
        now = time.monotonic()
        self._refresh_available_streams(now)
        desired_stream = self._road_stream_type
        if prefer_wide and self._wide_stream_type in self._available_streams:
            desired_stream = self._wide_stream_type
        elif self._road_stream_type not in self._available_streams and self._wide_stream_type in self._available_streams:
            desired_stream = self._wide_stream_type
        self._advance_stream_switch(desired_stream, now)
        return self.is_wide

    def _refresh_available_streams(self, now: float) -> None:
        if now - self._last_stream_discovery < STREAM_DISCOVERY_SECONDS:
            return
        self._last_stream_discovery = now
        try:
            streams = self._client_cls.available_streams("camerad", block=False)
        except Exception:
            return
        if streams:
            self._available_streams = set(streams)

    def _advance_stream_switch(self, desired_stream: object, now: float) -> None:
        if desired_stream == self._stream_type:
            self._target_client = None
            self._target_stream_type = None
            return
        if self._target_client is None or self._target_stream_type != desired_stream:
            self._target_stream_type = desired_stream
            self._target_client = self._client_cls("camerad", desired_stream, conflate=True)
        if not self._target_client.is_connected():
            if not self._target_client.connect(False) or not self._target_client.num_buffers:
                return
        target_frame = self._target_client.recv(timeout_ms=0)
        if target_frame is None:
            return

        self._destroy_egl_images()
        self._clear_copy_textures()
        self._client = self._target_client
        self._stream_type = desired_stream
        self._target_client = None
        self._target_stream_type = None
        self._frame = target_frame
        self._connected_at = now
        self._last_frame_at = now
        self._texture_needs_update = True
        print(
            f"Cluster camera switched to {'wide' if self.is_wide else 'road'} stream: {self._client.width}x{self._client.height}",
            flush=True,
        )

    def _reset_connection(self) -> None:
        self._frame = None
        self._connected_at = 0.0
        self._last_frame_at = 0.0
        self._texture_needs_update = False
        self._destroy_egl_images()
        self._clear_copy_textures()
        self._client = self._new_client()
        self._target_client = None
        self._target_stream_type = None
        self._connection_wait_logged = False

    def _ensure_connection(self, now: float) -> bool:
        if self._client.is_connected():
            if self._connected_at <= 0.0:
                self._connected_at = now
            return True
        self._frame = None
        if now - self._last_connection_attempt < CONNECTION_RETRY_SECONDS:
            return False
        self._last_connection_attempt = now
        connected = bool(self._client.connect(False) and self._client.num_buffers)
        if connected:
            self._connected_at = now
            print(
                f"Cluster road camera connected: {self._client.width}x{self._client.height} "
                f"{'EGL zero-copy' if self._zero_copy else 'NV12 GPU upload'}",
                flush=True,
            )
            self._connection_wait_logged = False
            self._refresh_available_streams(now)
        elif not self._connection_wait_logged:
            print("Cluster road camera waiting for local camerad VisionIPC", flush=True)
            self._connection_wait_logged = True
        return connected

    def _poll_frame(self, now: float) -> None:
        if now - self._last_frame_poll < FRAME_POLL_SECONDS:
            return
        self._last_frame_poll = now
        frame = self._client.recv(timeout_ms=0)
        if frame is not None:
            self._frame = frame
            self._last_frame_at = now
            self._texture_needs_update = True
        elif not self._client.is_connected():
            self._reset_connection()
        elif (
            self._connected_at > 0.0
            and now - max(self._connected_at, self._last_frame_at) > FRAME_STALE_SECONDS
        ):
            print("Cluster road camera stream stale; reconnecting VisionIPC", flush=True)
            self._reset_connection()

    def _ensure_copy_textures(self) -> None:
        if self._frame is None:
            return
        shape = (int(self._frame.stride), int(self._frame.height))
        if self._texture_shape == shape and self._texture_y is not None and self._texture_uv is not None:
            return
        self._clear_copy_textures()
        stride, height = shape
        y_image = rl.Image(None, stride, height, 1, rl.PixelFormat.PIXELFORMAT_UNCOMPRESSED_GRAYSCALE)
        uv_image = rl.Image(None, stride // 2, height // 2, 1, rl.PixelFormat.PIXELFORMAT_UNCOMPRESSED_GRAY_ALPHA)
        self._texture_y = rl.load_texture_from_image(y_image)
        self._texture_uv = rl.load_texture_from_image(uv_image)
        rl.set_texture_filter(self._texture_y, rl.TextureFilter.TEXTURE_FILTER_BILINEAR)
        rl.set_texture_filter(self._texture_uv, rl.TextureFilter.TEXTURE_FILTER_BILINEAR)
        self._texture_shape = shape
        self._texture_needs_update = True

    def _clear_copy_textures(self) -> None:
        if self._texture_y is not None and self._texture_y.id:
            rl.unload_texture(self._texture_y)
        if self._texture_uv is not None and self._texture_uv.id:
            rl.unload_texture(self._texture_uv)
        self._texture_y = None
        self._texture_uv = None
        self._texture_shape = None

    def _fallback_to_copy(self) -> None:
        if not self._zero_copy:
            return
        self._destroy_egl_images()
        if self._texture is not None and self._texture.id:
            rl.unload_texture(self._texture)
        if self._shader is not None and self._shader.id:
            rl.unload_shader(self._shader)
        self._zero_copy = False
        self._texture = None
        self._shader = rl.load_shader_from_memory(VERTEX_SHADER, NV12_FRAGMENT_SHADER)
        self._texture_uv_location = rl.get_shader_location(self._shader, "texture1")
        self._texture_needs_update = True
        print("Cluster road camera EGL import failed; falling back to NV12 GPU upload", flush=True)

    def _draw_zero_copy(self, source: "rl.Rectangle", destination: "rl.Rectangle") -> bool:
        from openpilot.system.ui.lib.egl import bind_egl_image_to_texture, create_egl_image

        if self._frame is None or self._texture is None:
            return False

        frame_index = int(self._frame.idx)
        egl_image = self._egl_images.get(frame_index)
        if egl_image is None:
            egl_image = create_egl_image(
                self._frame.width,
                self._frame.height,
                self._frame.stride,
                self._frame.fd,
                self._frame.uv_offset,
            )
            if egl_image is None:
                self._egl_import_failures += 1
                if self._egl_import_failures >= EGL_IMPORT_FAILURE_LIMIT:
                    self._fallback_to_copy()
                    return self._draw_copy(source, destination)
                return False
            self._egl_images[frame_index] = egl_image
            self._egl_import_failures = 0

        self._texture.width = self._frame.width
        self._texture.height = self._frame.height
        rl.begin_shader_mode(self._shader)
        try:
            # Navigation H264 frames also use samplerExternalOES on texture0.
            # Raylib does not restore that binding for us, so select the current
            # road-camera EGLImage immediately before every external draw.
            bind_egl_image_to_texture(self._texture.id, egl_image)
            self._texture_needs_update = False
            rl.draw_texture_pro(
                self._texture,
                source,
                destination,
                rl.Vector2(0.0, 0.0),
                0.0,
                rl.WHITE,
            )
        finally:
            rl.end_shader_mode()
        return True

    def _draw_copy(self, source: "rl.Rectangle", destination: "rl.Rectangle") -> bool:
        if self._frame is None:
            return False
        self._ensure_copy_textures()
        if self._texture_y is None or self._texture_uv is None:
            return False
        if self._texture_needs_update:
            y_data = self._frame.data[: self._frame.uv_offset]
            uv_data = self._frame.data[self._frame.uv_offset :]
            rl.update_texture(self._texture_y, rl.ffi.cast("void *", rl.ffi.from_buffer(y_data)))
            rl.update_texture(self._texture_uv, rl.ffi.cast("void *", rl.ffi.from_buffer(uv_data)))
            self._texture_needs_update = False
        rl.begin_shader_mode(self._shader)
        try:
            rl.set_shader_value_texture(self._shader, self._texture_uv_location, self._texture_uv)
            rl.draw_texture_pro(
                self._texture_y,
                source,
                destination,
                rl.Vector2(0.0, 0.0),
                0.0,
                rl.WHITE,
            )
        finally:
            rl.end_shader_mode()
        return True

    def draw(self, destination: "rl.Rectangle") -> bool:
        now = time.monotonic()
        if not self._ensure_connection(now):
            return False
        self._poll_frame(now)
        if self._frame is None:
            return False
        source = rl.Rectangle(0.0, 0.0, float(self._frame.width), float(self._frame.height))
        if self._zero_copy:
            return self._draw_zero_copy(source, destination)
        return self._draw_copy(source, destination)

    def _destroy_egl_images(self) -> None:
        if not self._zero_copy:
            self._egl_images.clear()
            return
        if self._zero_copy:
            from openpilot.system.ui.lib.egl import destroy_egl_image

            for egl_image in self._egl_images.values():
                destroy_egl_image(egl_image)
        self._egl_images.clear()

    def close(self) -> None:
        self._destroy_egl_images()
        self._clear_copy_textures()
        if self._texture is not None and self._texture.id:
            rl.unload_texture(self._texture)
        self._texture = None
        if self._shader is not None and self._shader.id:
            rl.unload_shader(self._shader)
        self._shader = None
        self._frame = None
        self._connected_at = 0.0
        self._last_frame_at = 0.0
        self._egl_import_failures = 0
        self._client = None
        self._target_client = None
        self._target_stream_type = None
        self._available_streams.clear()
