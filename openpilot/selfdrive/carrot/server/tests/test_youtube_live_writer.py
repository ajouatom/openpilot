from __future__ import annotations

import asyncio
import threading
import time

from openpilot.selfdrive.carrot.server.services.youtube_live_writer import RtmpFrameWriter


class BlockingMuxer:
  def __init__(self) -> None:
    self.entered = threading.Event()
    self.release = threading.Event()
    self.frames: list[tuple[bytes, bool]] = []
    self.thread_names: list[str] = []

  def mux(self, payload: bytes, *, keyframe: bool) -> None:
    self.entered.set()
    assert self.release.wait(timeout=2.0)
    self.thread_names.append(threading.current_thread().name)
    self.frames.append((payload, keyframe))


class FailingMuxer:
  def mux(self, _payload: bytes, *, keyframe: bool) -> None:
    del keyframe
    raise RuntimeError("publish failed")


async def _wait_until(predicate, *, timeout: float = 2.0) -> None:
  deadline = time.monotonic() + timeout
  while not predicate():
    assert time.monotonic() < deadline
    await asyncio.sleep(0.001)


def test_writer_bounds_backlog_without_reordering_frames():
  async def scenario() -> None:
    muxer = BlockingMuxer()
    writer = RtmpFrameWriter(muxer, max_frames=1)
    writer.start()

    assert writer.enqueue(b"first", keyframe=True)
    assert await asyncio.to_thread(muxer.entered.wait, 1.0)
    assert writer.enqueue(b"second", keyframe=False)
    assert not writer.enqueue(b"third", keyframe=False)
    assert writer.pending_frames == 1
    assert writer.pending_bytes == len(b"second")
    assert writer.high_watermark == 1
    assert writer.high_watermark_bytes == len(b"second")
    assert "backlog" in writer.last_rejection

    muxer.release.set()
    await _wait_until(lambda: writer.frames_written == 2)
    assert await writer.stop() == 0
    assert muxer.frames == [(b"first", True), (b"second", False)]
    assert all(name.startswith("youtube-rtmp") for name in muxer.thread_names)

  asyncio.run(scenario())


def test_writer_bounds_pending_memory_independently_of_frame_count():
  async def scenario() -> None:
    muxer = BlockingMuxer()
    writer = RtmpFrameWriter(muxer, max_frames=4, max_bytes=5)
    writer.start()

    assert writer.enqueue(b"a", keyframe=True)
    assert await asyncio.to_thread(muxer.entered.wait, 1.0)
    assert writer.enqueue(b"bb", keyframe=False)
    assert writer.enqueue(b"ccc", keyframe=False)
    assert not writer.enqueue(b"d", keyframe=False)
    assert writer.pending_frames == 2
    assert writer.pending_bytes == 5
    assert writer.high_watermark_bytes == 5
    assert "5 bytes" in writer.last_rejection

    muxer.release.set()
    await _wait_until(lambda: writer.frames_written == 3)
    assert await writer.stop() == 0

  asyncio.run(scenario())


def test_writer_surfaces_mux_failures_and_rejects_new_frames():
  async def scenario() -> None:
    writer = RtmpFrameWriter(FailingMuxer(), max_frames=2)
    writer.start()

    assert writer.enqueue(b"frame", keyframe=True)
    await _wait_until(lambda: bool(writer.error))
    assert writer.error == "publish failed"
    assert not writer.enqueue(b"next", keyframe=False)
    assert writer.last_rejection == "publish failed"
    assert await writer.stop() == 0

  asyncio.run(scenario())
