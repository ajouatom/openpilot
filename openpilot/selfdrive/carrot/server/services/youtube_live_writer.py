from __future__ import annotations

import asyncio
import time
from concurrent.futures import ThreadPoolExecutor
from functools import partial
from typing import Any


class RtmpFrameWriter:
  """Serialize RTMP muxing behind a small, bounded frame queue."""

  def __init__(self, muxer: Any, *, max_frames: int, max_bytes: int = 4 * 1024 * 1024) -> None:
    self._muxer = muxer
    self._queue: asyncio.Queue[tuple[bytes, bool] | None] = asyncio.Queue(maxsize=max(1, int(max_frames)))
    self._max_bytes = max(1, int(max_bytes))
    self._pending_bytes = 0
    self._high_watermark_bytes = 0
    self._executor = ThreadPoolExecutor(max_workers=1, thread_name_prefix="youtube-rtmp")
    self._executor_shutdown = False
    self._task: asyncio.Task[None] | None = None
    self._error = ""
    self._last_rejection = ""
    self._high_watermark = 0
    self._frames_written = 0
    self._last_write_ms = 0
    self._max_write_ms = 0

  @property
  def capacity(self) -> int:
    return self._queue.maxsize

  @property
  def pending_frames(self) -> int:
    return self._queue.qsize()

  @property
  def high_watermark(self) -> int:
    return self._high_watermark

  @property
  def capacity_bytes(self) -> int:
    return self._max_bytes

  @property
  def pending_bytes(self) -> int:
    return self._pending_bytes

  @property
  def high_watermark_bytes(self) -> int:
    return self._high_watermark_bytes

  @property
  def frames_written(self) -> int:
    return self._frames_written

  @property
  def last_write_ms(self) -> int:
    return self._last_write_ms

  @property
  def max_write_ms(self) -> int:
    return self._max_write_ms

  @property
  def error(self) -> str:
    return self._error

  @property
  def last_rejection(self) -> str:
    return self._last_rejection

  def start(self) -> None:
    if self._task is None or self._task.done():
      self._task = asyncio.create_task(self._run(), name="carrot-youtube-rtmp-writer")

  def enqueue(self, payload: bytes, *, keyframe: bool) -> bool:
    task = self._task
    self._last_rejection = ""
    if self._error:
      self._last_rejection = self._error
      return False
    if task is None or task.done():
      self._last_rejection = "RTMP writer is not running"
      return False
    frame = bytes(payload)
    next_pending_bytes = self._pending_bytes + len(frame)
    if next_pending_bytes > self._max_bytes:
      self._last_rejection = f"RTMP frame backlog reached {self._max_bytes} bytes"
      return False
    try:
      self._queue.put_nowait((frame, bool(keyframe)))
    except asyncio.QueueFull:
      self._last_rejection = f"RTMP frame backlog reached {self.capacity} frames"
      return False
    self._pending_bytes = next_pending_bytes
    self._high_watermark = max(self._high_watermark, self.pending_frames)
    self._high_watermark_bytes = max(self._high_watermark_bytes, self._pending_bytes)
    return True

  async def stop(self) -> int:
    task = self._task
    self._task = None
    if task is None:
      self._shutdown_executor()
      return 0

    discarded = 0
    while True:
      try:
        item = self._queue.get_nowait()
      except asyncio.QueueEmpty:
        break
      if item is not None:
        discarded += 1
        self._pending_bytes = max(0, self._pending_bytes - len(item[0]))
      self._queue.task_done()

    if not task.done():
      self._queue.put_nowait(None)
    try:
      await task
    finally:
      self._shutdown_executor()
    return discarded

  async def _run(self) -> None:
    while True:
      item = await self._queue.get()
      try:
        if item is None:
          return
        payload, keyframe = item
        self._pending_bytes = max(0, self._pending_bytes - len(payload))
        started = time.monotonic()
        loop = asyncio.get_running_loop()
        await loop.run_in_executor(
          self._executor,
          partial(self._muxer.mux, payload, keyframe=keyframe),
        )
        elapsed_ms = max(0, int((time.monotonic() - started) * 1_000))
        self._last_write_ms = elapsed_ms
        self._max_write_ms = max(self._max_write_ms, elapsed_ms)
        self._frames_written += 1
      except Exception as exc:
        self._error = str(exc)
        return
      finally:
        self._queue.task_done()

  def _shutdown_executor(self) -> None:
    if self._executor_shutdown:
      return
    self._executor_shutdown = True
    self._executor.shutdown(wait=False, cancel_futures=True)
