"""Bounded C4 DM phase alignment with completion of the current image upload."""
import socket
import struct
import time

ADDRESS = '\0carrot-jetlink-dm-phase'
PACKET = struct.Struct('<QQ')  # Source camera SOF and upload completion, monotonic ns.
FRESH_NS = 75_000_000
CAMERA_SKEW_NS = 5_000_000
WAIT_SECONDS = .025
MAX_DRAIN = 16


class Publisher:
  def __init__(self, address=ADDRESS):
    self.address = address
    self.sock = None
    try:
      self.sock = socket.socket(socket.AF_UNIX, socket.SOCK_DGRAM)
      self.sock.setblocking(False)
    except OSError:
      self.close()

  def sent(self, source_sof):
    if self.sock is not None and source_sof > 0:
      try:
        self.sock.sendto(PACKET.pack(source_sof, time.monotonic_ns()), self.address)
      except OSError:
        pass  # DM absent, restarting or behind: never block inference.

  def close(self):
    if self.sock is not None:
      self.sock.close()
      self.sock = None


class Gate:
  def __init__(self, address=ADDRESS):
    self.sock = None
    self.source_sof = self.sent_at = 0
    try:
      self.sock = socket.socket(socket.AF_UNIX, socket.SOCK_DGRAM)
      self.sock.bind(address)
    except OSError:
      self.close()  # A phase helper must never prevent driver monitoring.

  def accept(self, packet, now):
    if len(packet) != PACKET.size:
      return
    source_sof, sent_at = PACKET.unpack(packet)
    if 0 < source_sof <= sent_at <= now and now - sent_at < FRESH_NS and sent_at > self.sent_at:
      self.source_sof, self.sent_at = source_sof, sent_at

  def wait(self, source_sof):
    if self.sock is None or source_sof <= 0:
      return
    started = time.monotonic()
    try:
      self.sock.setblocking(False)
      for _ in range(MAX_DRAIN):
        try:
          self.accept(self.sock.recv(PACKET.size + 1), time.monotonic_ns())
        except BlockingIOError:
          break
      # No fresh external upload means native operation or a lost publisher.
      # Keep DM independent of USB availability, including its first frame.
      if not 0 <= time.monotonic_ns() - self.sent_at < FRESH_NS:
        return
      deadline = started + WAIT_SECONDS
      while self.source_sof < source_sof - CAMERA_SKEW_NS:
        remaining = deadline - time.monotonic()
        if remaining <= 0:
          break
        self.sock.settimeout(remaining)
        try:
          self.accept(self.sock.recv(PACKET.size + 1), time.monotonic_ns())
        except TimeoutError:
          break
    except OSError:
      self.close()

  def close(self):
    if self.sock is not None:
      self.sock.close()
      self.sock = None
