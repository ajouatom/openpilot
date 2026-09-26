"""Bounded, ordered navigation media fragments alongside Jetlink inference."""
import os
import json
from pathlib import Path
import socket
import struct
import time

CAPABILITY = 'carrot_navi_v1'
MESSAGE = 0x4001
ADDRESS = '\0carrot-jetlink-navi'
HEADER = struct.Struct('<QQII')  # Publisher epoch, event id, offset, event length.
CHUNK = 32 * 1024
MAX_EVENT = 1024 * 1024
# Only the low-priority navigation child waits. The USB owner still consumes
# one fragment per inference window without waiting for this producer.
# 32 fragments at 20 Hz take 1.6 s; stay below the receiver's 2 s assembly limit.
EVENT_SEND_TIMEOUT = 1.8
STATUS = Path('/dev/shm/carrot-jetlink-navi-status.json')


def fragments(raw, epoch, sequence):
  if not 0 < len(raw) <= MAX_EVENT:
    return
  for offset in range(0, len(raw), CHUNK):
    yield HEADER.pack(epoch, sequence, offset, len(raw)) + raw[offset:offset + CHUNK]


def send_event(sock, raw, epoch, sequence, *, timeout=EVENT_SEND_TIMEOUT):
  """Retain an unsent fragment across backpressure, with one event deadline.

  SOCK_SEQPACKET sends are atomic. A timed-out readiness poll sent nothing,
  so retry the same fragment rather than truncating the H.264 access unit.
  A stopped consumer still expires this bounded, display-only transaction.
  """
  if not 0 < len(raw) <= MAX_EVENT:
    return False
  deadline = time.monotonic() + timeout
  for packet in fragments(raw, epoch, sequence):
    while True:
      remaining = deadline - time.monotonic()
      if remaining <= 0:
        return False
      sock.settimeout(min(.5, remaining))
      try:
        if sock.send(packet) != len(packet):
          raise OSError('partial navigation SEQPACKET send')
        break
      except TimeoutError:
        continue
  return True


class Assembler:
  def __init__(self):
    self.key = None
    self.data = bytearray()
    self.started = 0.

  def feed(self, packet, now):
    if not HEADER.size < len(packet) <= HEADER.size + CHUNK:
      self.key = None
      return None
    epoch, sequence, offset, total = HEADER.unpack_from(packet)
    key = (epoch, sequence, total)
    data = packet[HEADER.size:]
    if not 0 < total <= MAX_EVENT or offset + len(data) > total:
      self.key = None
      return None
    if offset == 0:
      self.key, self.data, self.started = key, bytearray(), now
    if key != self.key or offset != len(self.data) or not 0 <= now - self.started < 2:
      self.key = None
      return None
    self.data.extend(data)
    if len(self.data) != total:
      return None
    self.key = None
    return epoch, sequence, bytes(self.data)


class HostForwarder:
  def __init__(self):
    self.sock = socket.socket(socket.AF_UNIX, socket.SOCK_DGRAM)
    self.sock.setblocking(False)

  def send(self, packet):
    if HEADER.size < len(packet) <= HEADER.size + CHUNK:
      try:
        self.sock.sendto(packet, ADDRESS)
      except OSError:
        pass  # Missing/slow renderer cannot block inference.

  def close(self):
    self.sock.close()


class RemoteMediaSocket:
  def __init__(self):
    self.sock = socket.socket(socket.AF_UNIX, socket.SOCK_DGRAM)
    self.sock.setsockopt(socket.SOL_SOCKET, socket.SO_RCVBUF, 1024 * 1024)
    self.sock.bind(ADDRESS)
    self.sock.setblocking(False)
    self.assembler = Assembler()
    self.previous = None
    self.waiting = set()
    self.sequences = {}

  def drain(self):
    from openpilot.cereal import log
    result = []
    for _ in range(64):
      try:
        packet = self.sock.recv(CHUNK + HEADER.size + 1)
      except BlockingIOError:
        break
      complete = self.assembler.feed(packet, time.monotonic())
      if complete is None:
        continue
      epoch, sequence, raw = complete
      if self.previous != (epoch, sequence - 1):
        self.waiting.update(self.sequences)
      self.previous = (epoch, sequence)
      try:
        with log.Event.from_bytes(raw) as reader:
          if reader.which() != 'carrotNaviMedia':
            continue
          event = reader.as_builder()
        d = event.carrotNaviMedia
        key = (str(d.sessionId), str(d.kind), str(d.name))
        if key not in self.sequences and len(self.sequences) >= 64:
          self.sequences.clear()
          self.waiting.clear()
        previous = self.sequences.get(key)
        self.sequences[key] = d.sequence
        if d.messageType == 3:
          if previous is None or d.sequence != previous + 1:
            self.waiting.add(key)
          if d.flags & 1:
            self.waiting.discard(key)
          elif key in self.waiting:
            continue
        result.append(event)
      except Exception:
        self.waiting.update(self.sequences)
    return result

  def close(self):
    self.sock.close()


def publish(fd):
  import ctypes
  import signal
  from openpilot.cereal import messaging
  from openpilot.common.display_scheduling import DisplayScheduler
  from openpilot.common.params import Params
  ctypes.CDLL(None).prctl(1, signal.SIGTERM, 0, 0, 0)
  if os.getppid() == 1:
    return
  scheduler = DisplayScheduler(7, enabled=True)
  params = Params()
  sock = socket.socket(fileno=fd)
  source = messaging.sub_sock('carrotNaviMedia', conflate=False, timeout=100)
  epoch = time.monotonic_ns()
  sequence = 0
  stats = dict(pid=os.getpid(), received=0, sent=0, stale=0, abandoned=0, send_max_ms=0.)
  next_status = 0.
  while True:
    now = time.monotonic()
    if now >= next_status:
      temporary = STATUS.with_suffix('.tmp')
      try:
        temporary.write_text(json.dumps(dict(updated=now, **stats)))
        os.replace(temporary, STATUS)
      except OSError:
        pass  # Diagnostics must not stop navigation delivery.
      next_status = now + 1
    scheduler.update(params.get_bool('IsOnroad'))
    # recv_one_or_none explicitly overrides the socket timeout and busy-spins
    # when navigation is idle. This wait wakes immediately on incoming media.
    event = messaging.recv_one(source)
    if event is None:
      continue
    sequence += 1
    stats['received'] += 1
    if not 0 <= time.monotonic_ns() - event.logMonoTime < 1_000_000_000:
      stats['stale'] += 1
      continue
    raw = event.as_builder().to_bytes()
    started = time.monotonic()
    complete = send_event(sock, raw, epoch, sequence)
    stats['sent' if complete else 'abandoned'] += 1
    stats['send_max_ms'] = max(stats['send_max_ms'], (time.monotonic() - started) * 1000)


if __name__ == '__main__':
  import sys
  publish(int(sys.argv[1]))
