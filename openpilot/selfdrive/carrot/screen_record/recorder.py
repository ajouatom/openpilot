"""Hardware H.264 screen recorder for the onroad UI.

The UI render texture is downscaled on the GPU, packed to Venus-style NV12 by a
fragment shader, and rendered straight into the encoder's ION/V4L2 input
DMA-BUF (no glReadPixels, no PBO, no CPU frame copy). The encoder emits H.264
access units which are muxed into MP4 by a small self-contained ISO-BMFF
writer (the device's ffmpeg build cannot demux raw H.264).

Self-contained: cluster feature sources are neither imported nor modified. The
only shared runtime asset is the compiled encoder bridge, which is instantiated
per handle.
"""
from __future__ import annotations

import ctypes
import queue
import threading
import time
from pathlib import Path

import pyray as rl

from openpilot.common.swaglog import cloudlog
from openpilot.selfdrive.carrot.screen_record.dmabuf_pool import create_packed_nv12_pool
from openpilot.selfdrive.carrot.screen_record.encoder import HwEncoderUnavailable, HwNv12Encoder
from openpilot.selfdrive.carrot.screen_record.mp4_mux import Mp4H264Writer
from openpilot.selfdrive.carrot.screen_record.pack import Nv12Packer

_MAX_CATCH_UP_FRAMES = 2
_MAX_CONSECUTIVE_FAILURES = 5
_STATS_LOG_INTERVAL = 5.0


def _even(value: float) -> int:
  return max(2, (int(value) // 2) * 2)


class ScreenRecordHw:
  def __init__(self, out_path, display_size, fps: int = 20, scale: float = 0.5,
               bitrate: int = 6_000_000, gop: int = 30):
    self._out_path = Path(out_path)
    self._display_size = (int(display_size[0]), int(display_size[1]))
    self._fps = max(5, int(fps))
    self._scale = min(max(float(scale), 0.25), 1.0)
    self._size = (
      _even(self._display_size[0] * self._scale),
      _even(self._display_size[1] * self._scale),
    )
    self._bitrate = int(bitrate)
    self._gop = int(gop)

    self._encoder: HwNv12Encoder | None = None
    self._packer: Nv12Packer | None = None
    self._upload: rl.RenderTexture | None = None
    self._cpu_target: rl.RenderTexture | None = None
    self._pool = None
    self._use_dmabuf = False

    self._muxer: Mp4H264Writer | None = None
    self._writer: threading.Thread | None = None
    self._write_queue: queue.Queue = queue.Queue()

    self._frame_index = 0
    self._next_t = 0.0
    self._failures = 0
    self._frames = 0
    self._dropped = 0
    self._packets = 0
    self._bytes = 0
    self._last_stats_t = 0.0
    self.failed = False
    self.started = False
    self.dmabuf = False

  # -- lifecycle ----------------------------------------------------------

  def start(self) -> None:
    encoder = None
    try:
      encoder = HwNv12Encoder(self._size[0], self._size[1], self._fps, self._bitrate, gop=self._gop)
      self._encoder = encoder
      self._packer = Nv12Packer()
      self._upload = rl.load_render_texture(self._size[0], self._size[1])
      rl.set_texture_filter(self._upload.texture, rl.TextureFilter.TEXTURE_FILTER_BILINEAR)

      self._pool = create_packed_nv12_pool()
      if self._pool is not None and encoder.has_dmabuf_input:
        self._use_dmabuf = True
      else:
        if self._pool is not None:
          self._pool.close()
          self._pool = None
        layout = encoder.layout
        self._cpu_target = rl.load_render_texture(layout.pack_width, layout.pack_height)
        rl.set_texture_filter(self._cpu_target.texture, rl.TextureFilter.TEXTURE_FILTER_POINT)

      self._muxer = Mp4H264Writer(self._out_path, self._fps, self._size[0], self._size[1])
      self._start_writer()
    except Exception:
      self._close_resources()
      raise

    layout = encoder.layout
    cloudlog.info(" ".join([
      f"[REC hw] start {self._out_path.name} {self._size[0]}x{self._size[1]}@{self._fps}",
      f"bitrate={self._bitrate} stride={layout.stride} y={layout.y_scanlines} uv={layout.uv_scanlines}",
      f"pack={layout.pack_width}x{layout.pack_height} dmabuf={self._use_dmabuf}",
    ]))
    self.started = True
    self.dmabuf = self._use_dmabuf
    self._last_stats_t = time.monotonic()

  def tick(self, source_texture, source_width: int, source_height: int) -> None:
    if not self.started or self.failed:
      return
    try:
      now = time.monotonic()
      if self._next_t <= 0.0:
        self._next_t = now
      interval = 1.0 / self._fps
      frames = 0
      while now >= self._next_t and frames < _MAX_CATCH_UP_FRAMES:
        self._encode_frame(source_texture, int(source_width), int(source_height))
        self._next_t += interval
        frames += 1
      if self._next_t < now - interval:
        # The encoder fell behind; resync instead of banking a burst of frames.
        self._next_t = now
      self._flush_packets()
      self._log_stats(now)
    except Exception as exc:
      self._failures += 1
      if self._failures == 1:
        cloudlog.error(f"[REC hw] encode failed: {exc}")
      if self._failures >= _MAX_CONSECUTIVE_FAILURES:
        self.failed = True
        cloudlog.error(f"[REC hw] stopping after {self._failures} consecutive failures")

  def stop(self) -> None:
    if not self.started and self._encoder is None:
      return
    self.started = False
    self._close_resources()
    cloudlog.info(" ".join([
      f"[REC hw] stop {self._out_path.name} frames={self._frames} dropped={self._dropped}",
      f"packets={self._packets} bytes={self._bytes} dmabuf={self._use_dmabuf}",
    ]))

  def stats(self) -> dict:
    return {
      "frames": self._frames,
      "dropped": self._dropped,
      "packets": self._packets,
      "bytes": self._bytes,
      "dmabuf": self._use_dmabuf,
    }

  # -- internals ----------------------------------------------------------

  def _encode_frame(self, source_texture, source_width: int, source_height: int) -> None:
    encoder = self._encoder
    if encoder is None:
      raise HwEncoderUnavailable("encoder is not running")
    layout = encoder.layout

    # Downscale the UI into the upload target. The negative source height is the
    # same idiom the on-screen blit uses, so the upload target holds the UI
    # upright and the pack shader emits upright NV12 planes.
    rl.begin_texture_mode(self._upload)
    rl.clear_background(rl.BLACK)
    rl.draw_texture_pro(
      source_texture,
      rl.Rectangle(0.0, 0.0, float(source_width), -float(source_height)),
      rl.Rectangle(0.0, 0.0, float(self._size[0]), float(self._size[1])),
      rl.Vector2(0.0, 0.0),
      0.0,
      rl.WHITE,
    )
    rl.end_texture_mode()

    buffer = encoder.acquire()
    try:
      if self._use_dmabuf and buffer.dmabuf_fd >= 0:
        target = self._pool.target_for(buffer.dmabuf_fd, layout.stride, layout.render_bytes)
        self._pack_planes(target, layout)
        self._pool.wait_for_gpu()
      else:
        self._pack_planes(self._cpu_target, layout)
        image = rl.load_image_from_texture(self._cpu_target.texture)
        try:
          pixel_bytes = int(image.width) * int(image.height) * 4
          if pixel_bytes < layout.render_bytes:
            raise HwEncoderUnavailable(
              f"packed readback is {pixel_bytes} bytes, expected {layout.render_bytes}"
            )
          ctypes.memmove(
            ctypes.c_void_p(buffer.address),
            bytes(rl.ffi.buffer(image.data, layout.render_bytes)),
            layout.render_bytes,
          )
        finally:
          rl.unload_image(image)
      encoder.submit(buffer, self._frame_index * 1_000_000 // self._fps)
      self._frame_index += 1
      self._frames += 1
      self._failures = 0
    except Exception:
      if buffer.active:
        encoder.cancel(buffer.index)
      raise

  def _pack_planes(self, target, layout) -> None:
    self._packer.render_plane(
      self._upload.texture,
      target,
      self._size[0],
      self._size[1],
      0,
      packed_width=layout.pack_width,
      packed_height=layout.y_scanlines,
      dest_y=layout.y_pack_y,
      clear_target=True,
      clear_color=(128, 128, 128, 128),
    )
    self._packer.render_plane(
      self._upload.texture,
      target,
      self._size[0],
      self._size[1],
      1,
      packed_width=layout.pack_width,
      packed_height=layout.uv_scanlines,
      dest_y=layout.uv_pack_y,
      clear_target=False,
    )

  def _start_writer(self) -> None:
    self._writer = threading.Thread(target=self._writer_loop, name="screen-record-writer", daemon=True)
    self._writer.start()

  def _writer_loop(self) -> None:
    muxer = self._muxer
    while muxer is not None:
      packet = self._write_queue.get()
      if packet is None:
        break
      try:
        muxer.add_access_unit(packet)
      except Exception:
        cloudlog.exception("screen record: muxing failed")
        break
    if muxer is not None:
      try:
        muxer.close()
      except Exception:
        cloudlog.exception("screen record: muxer close failed")

  def _flush_packets(self) -> None:
    if self._encoder is None:
      return
    packets = self._encoder.take_packets()
    if not packets:
      return
    self._packets += len(packets)
    for packet in packets:
      self._bytes += len(packet)
      self._write_queue.put(packet)

  def _log_stats(self, now: float) -> None:
    if now - self._last_stats_t < _STATS_LOG_INTERVAL:
      return
    self._last_stats_t = now
    cloudlog.info(" ".join([
      f"[REC hw] stats {self._out_path.name} frames={self._frames} dropped={self._dropped}",
      f"packets={self._packets} bytes={self._bytes}",
    ]))

  def _close_resources(self) -> None:
    if self._encoder is not None:
      try:
        self._encoder.drain(500)
        self._flush_packets()
      except Exception:
        pass
      try:
        self._encoder.close()
      except Exception:
        pass
      self._encoder = None

    self._write_queue.put(None)
    if self._writer is not None:
      self._writer.join(timeout=10)
      self._writer = None

    if self._muxer is not None:
      try:
        self._muxer.close()
      except Exception:
        pass
      self._muxer = None

    if self._cpu_target is not None:
      rl.unload_render_texture(self._cpu_target)
      self._cpu_target = None
    if self._upload is not None:
      rl.unload_render_texture(self._upload)
      self._upload = None
    if self._packer is not None:
      self._packer.close()
      self._packer = None
    if self._pool is not None:
      self._pool.close()
      self._pool = None
