"""
Copyright (c) 2026-, Zeph Leggett.

This file is part of jetlink and is licensed under the MIT License.
See the LICENSE file in the root directory for more details.
"""
from __future__ import annotations

import time
from abc import ABC, abstractmethod
from dataclasses import dataclass

from jetlink import protocol as P

# Stops a corrupt length field making RxBuffer allocate gigabytes. A frame is
# ~460 KB.
MAX_MESSAGE = 16 << 20
_PAD = bytes(P.GADGET_TX_ALIGN)


class LinkError(IOError):
  """The link is unusable. Callers treat this as 'fall back to the small model'."""


class LinkTimeout(LinkError):
  """No complete message arrived in time. The stream is still in sync."""


@dataclass
class Message:
  msg_type: int
  seq: int
  flags: int
  payload: memoryview  # valid only until the next recv() on this transport


class Transport(ABC):
  """One framed, ordered, reliable message channel.

  Implementations must preserve message boundaries and ordering. recv() hands
  back a view into a reusable buffer: copy anything you need to keep.
  """

  @abstractmethod
  def send(self, msg_type: int, seq: int, parts=(), flags: int = 0, timeout: float | None = None) -> None:
    """Send one message. `parts` is an iterable of buffers, sent as one message."""

  @abstractmethod
  def recv(self, timeout: float | None = None) -> Message:
    """Block for one message. Raises LinkTimeout if `timeout` elapses."""

  @abstractmethod
  def close(self) -> None:
    ...

  def send_json(self, msg_type: int, seq: int, obj, flags: int = 0) -> None:
    import json
    self.send(msg_type, seq, (json.dumps(obj).encode(),), flags)

  @property
  def lendable(self) -> bool:
    """Could another process take over the IO on this link right now? Only a
    gadget can be handed over; see FfsTransport.lendable."""
    return False

  def rebind(self) -> bool:
    """Bounce the link so the peer sees it arrive again, if that means
    anything here. Only the gadget transport can; see FfsTransport.rebind."""
    return False

  def release_endpoints(self) -> bool:
    """Give up the IO without giving up the link, where the two are separable.
    Only the gadget transport can; see FfsTransport.release_endpoints."""
    return False


class RxBuffer:
  """Receive buffer for a byte stream carrying framed messages.

  Reads land in here and messages are handed out as views, so the steady state
  neither allocates nor copies. Resumable: a read that times out mid-message
  leaves the bytes in place, so a missed deadline costs a frame, not the stream.
  """

  def __init__(self, size: int = 1 << 20):
    self.buf = bytearray(size)
    self.view = memoryview(self.buf)
    self.start = 0  # first byte not yet consumed
    self.end = 0    # one past the last byte read

  @property
  def available(self) -> int:
    return self.end - self.start

  def reserve(self, need: int) -> None:
    """Guarantee room for `need` unconsumed bytes, compacting or growing."""
    if self.start and self.start + need > len(self.buf):
      # Through the bytearray, not the view: the ranges overlap and a
      # memoryview slice assignment is a memcpy. Compaction is rare.
      self.buf[:self.available] = self.buf[self.start:self.end]
      self.end -= self.start
      self.start = 0
    if need > len(self.buf):
      grown = bytearray(max(need, len(self.buf) * 2))
      grown[:self.available] = self.view[self.start:self.end]
      self.buf, self.view = grown, memoryview(grown)
      self.end -= self.start
      self.start = 0

  def writable(self) -> memoryview:
    return self.view[self.end:]

  def committed(self, n: int) -> None:
    self.end += n

  def take(self, n: int) -> memoryview:
    out = self.view[self.start:self.start + n]
    self.start += n
    return out

  def consumed(self) -> None:
    """Call once a message has been fully handed out."""
    if self.start == self.end:
      self.start = self.end = 0


class StreamTransport(Transport):
  """Framing over any ordered byte stream.

  TCP, USB bulk and FunctionFS are all streams, so they all need exactly this.
  Subclasses supply only the two primitives that differ.
  """

  # Extra capacity past the current message, for transports whose reads need a
  # minimum buffer size (a bulk OUT endpoint wants a whole packet).
  read_slack = 0
  # Bulk endpoints reject a read whose buffer is not a whole number of packets.
  # 0 means "no constraint" (TCP).
  packet_size = 0
  read_chunk = 1 << 20
  # Pad sent messages to a multiple of this and expect the peer's padded
  # likewise; 0 uses the one-byte PADDED rule. See protocol.GADGET_TX_ALIGN.
  tx_align = 0
  rx_align = 0
  # Largest single write. FunctionFS allocates a contiguous buffer per writev,
  # so a 4 MB upload chunk hits ENOMEM on a fragmented device where a few
  # hundred KB of inference does not. 0 means no cap (TCP).
  write_chunk = 0

  def __init__(self, rx_size: int = 1 << 20):
    self.rx = RxBuffer(rx_size)
    self._desynced = False
    self._send_deadline: float | None = None

  def _write_timeout(self, default: float | None = None) -> float | None:
    if self._send_deadline is None:
      return default
    remaining = self._send_deadline - time.monotonic()
    if remaining <= 0:
      raise LinkError('send timed out; link abandoned')
    return remaining

  # -- primitives a subclass must provide ----------------------------------

  @abstractmethod
  def _write(self, bufs: list[memoryview]) -> int:
    """Write from one or more buffers. Returns bytes written (may be partial)."""

  @abstractmethod
  def _read_into(self, dest: memoryview, timeout: float | None) -> int:
    """Read up to len(dest) bytes, returning how many arrived.

    May return short, including 0. Must not raise on a timeout, and must not
    drop what did arrive: that is how a stream silently desyncs.
    """

  # -- framing -------------------------------------------------------------

  def send(self, msg_type: int, seq: int, parts=(), flags: int = 0, timeout: float | None = None) -> None:
    # cast('B'): slicing a float32 view in advance() would step by elements
    bufs = [memoryview(p).cast('B') for p in parts]
    length = sum(b.nbytes for b in bufs)
    if self.tx_align:
      # Never a short packet in this direction; see protocol.GADGET_TX_ALIGN.
      pad = -(P.HEADER_SIZE + length) % self.tx_align
      if pad:
        bufs.append(memoryview(_PAD)[:pad])
    elif (P.HEADER_SIZE + length) % P.PACKET_MULTIPLE == 0:
      # A bulk transfer only ends on a short packet; see protocol.PACKET_MULTIPLE.
      flags |= P.Flag.PADDED
      bufs.append(memoryview(_PAD)[:1])
    header = P.pack_header(msg_type, seq, length, flags)
    bufs.insert(0, memoryview(header))
    self._send_deadline = None if timeout is None else time.monotonic() + timeout
    try:
      while bufs:
        self._write_timeout()
        n = self._write(take(bufs, self.write_chunk) if self.write_chunk else bufs)
        if n <= 0:
          raise LinkError("peer went away during send")
        bufs = advance(bufs, n)
    finally:
      self._send_deadline = None

  def _clamp_read(self, dest: memoryview) -> int:
    """How many bytes this transport may ask for in one read."""
    n = min(dest.nbytes, self.read_chunk)
    return (n // self.packet_size) * self.packet_size if self.packet_size else n

  def _fill(self, need: int, timeout: float | None) -> None:
    """Read until `need` bytes are buffered, or the deadline passes.

    The deadline is per message, not per read, or one response could take
    several times the caller's budget. Partial reads are kept, so a missed
    deadline costs a frame and leaves the stream in sync.
    """
    self.rx.reserve(need + self.read_slack)
    end = None if timeout is None else time.monotonic() + timeout
    while self.rx.available < need:
      dest = self.rx.writable()[:self._read_limit(need - self.rx.available)]
      if self._clamp_read(dest) == 0:
        # No room for a whole packet: every read returns 0 and this loop spins
        # while the peer blocks. read_slack is too small; say so, do not hang.
        raise LinkError(f"no room to read the rest of a {need} byte message "
                         f"({self.rx.available} in hand); read_slack too small")
      remaining = None
      if end is not None:
        remaining = end - time.monotonic()
        if remaining <= 0:
          raise LinkTimeout(f"only {self.rx.available} of {need} bytes arrived in time")
      n = self._read_into(dest, remaining)
      if n:
        self.rx.committed(n)

  def _read_limit(self, missing: int) -> int:
    """`missing` bytes of the current message, rounded up to a whole packet.

    Keeps the USB host in sync: the gadget's messages are burst-aligned, so a
    read for exactly what is left never stays outstanding past the end of a
    message. Reading further desynced about once in 400 frames.
    """
    if self.packet_size:
      return -(-missing // self.packet_size) * self.packet_size
    return missing

  def drain(self, timeout: float) -> None:
    """After a desync, swallow what the peer is still sending until the link
    drops or it goes quiet for `timeout`.

    The peer is blocked mid-message. Reopening instead drains a packet per
    session, so its frame timeout never fires and it never reconnects.
    """
    scratch = memoryview(bytearray(self.read_chunk))
    last = time.monotonic()
    while time.monotonic() - last < timeout:
      try:
        n = self._read_into(scratch, timeout)
      except LinkError:
        return
      if n:
        last = time.monotonic()

  def recv(self, timeout: float | None = None) -> Message:
    if self._desynced:
      raise LinkError("stream desynced; the link must be reopened")
    end = None if timeout is None else time.monotonic() + timeout
    self._fill(P.HEADER_SIZE, timeout)
    try:
      _, _, msg_type, seq, flags, length, _reserved = P.unpack_header(
        self.rx.view[self.rx.start:self.rx.start + P.HEADER_SIZE])
      if length > MAX_MESSAGE:
        raise P.ProtocolError(f"message claims {length} bytes, over the {MAX_MESSAGE} cap")
    except P.ProtocolError as e:
      # Nothing resynchronises a byte stream mid-message. Latch it and report a
      # LinkError, which callers reconnect on; a ProtocolError escaping here
      # unwinds the server's accept loop and kills the process.
      self._desynced = True
      raise LinkError(f"protocol error, link unusable: {e}") from e
    # the remainder of the budget, so one recv cannot block for twice what it
    # was given
    if self.rx_align:
      pad = -(P.HEADER_SIZE + length) % self.rx_align
    else:
      pad = 1 if flags & P.Flag.PADDED else 0
    self._fill(P.HEADER_SIZE + length + pad,
               None if end is None else max(0.0, end - time.monotonic()))
    self.rx.take(P.HEADER_SIZE)
    payload = self.rx.take(length)
    self.rx.take(pad)
    self.rx.consumed()
    return Message(msg_type, seq, flags, payload)


def take(bufs: list[memoryview], n: int) -> list[memoryview]:
  """The first `n` bytes across a list of buffers, without copying."""
  out: list[memoryview] = []
  for mv in bufs:
    if n <= 0:
      break
    out.append(mv if mv.nbytes <= n else mv[:n])
    n -= out[-1].nbytes
  return out


def advance(bufs: list[memoryview], n: int) -> list[memoryview]:
  """Drop the first `n` bytes across a list of buffers, returning what is left."""
  out: list[memoryview] = []
  for mv in bufs:
    if n:
      if n >= mv.nbytes:
        n -= mv.nbytes
        continue
      mv = mv[n:]
      n = 0
    out.append(mv)
  return out
