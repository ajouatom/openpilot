"""Opt-in Hyundai CAN-FD SPAS indicator session, with acknowledged ECU handoff.

No vehicle-name allowlist: exact ECAN contracts and advancing stock counters are
required. All timing is monotonic. This module has no sockets, Params or threads.
"""
from dataclasses import dataclass
import math

from opendbc.car import CanData
from opendbc.car.crc import CRC16_XMODEM

SPAS = {0x165: 24, 0x16A: 32}
APRK, RESPONSE = 0x7B1, 0x7B9
EXTENDED = bytes.fromhex('0210030000000000')
PATCH_DISABLE = bytes.fromhex('0328830100000000')
DISABLE = bytes.fromhex('0328030100000000')  # request positive acknowledgement
ENABLE = bytes.fromhex('0328000100000000')
DEFAULT = bytes.fromhex('0210010000000000')
PRESENT = bytes.fromhex('023e800000000000')


def checksum(address, data):
  crc = 0
  for b in bytes(data[2:]) + address.to_bytes(2, 'little'):
    crc = ((crc << 8) ^ CRC16_XMODEM[(crc >> 8) ^ b]) & 0xffff
  return crc ^ {24: 0x819d, 32: 0x9f5b}[len(data)]


def compatible(fingerprint, bus):
  return all(fingerprint.get(bus, {}).get(a) == size for a, size in SPAS.items())


def frame(address, counter, direction, bus):
  data = bytearray(SPAS[address])
  data[2] = counter
  if address == 0x16A:
    data[16] = {None: 0, 'left': 0x60, 'right': 0x80}[direction]
  data[:2] = checksum(address, data).to_bytes(2, 'little')
  return CanData(address, bytes(data), bus)


@dataclass
class Stock:
  stamp: float = -math.inf
  counter: int = -1
  consecutive: int = 0


class Session:
  def __init__(self, bus):
    self.bus = bus
    self.stock = {a: Stock() for a in SPAS}
    self.state = 'idle'
    self.reason = ''
    self.changed = self.started = self.next_send = self.keepalive = 0.
    self.request = None
    self.previous = None
    self.direction = None
    self.counter = 0
    self.reply = None
    self.last_update = None
    self.restore_attempts = 0
    self.restore_ack = False
    self.stock_resumed = False
    self.stock_direction = None

  def observe(self, address, bus, data, now):
    if bus != self.bus:
      return
    if address in SPAS:
      st = self.stock[address]
      if len(data) != SPAS[address] or int.from_bytes(data[:2], 'little') != checksum(address, data):
        if address == 0x16A:
          self.stock_direction = None
        st.consecutive = 0
        if self.state not in ('idle', 'failed', 'restore_enable', 'restore_default', 'restore_stock'):
          self.reason = 'stock_contract'
        return
      st.consecutive = min(20, st.consecutive+1) if 0 <= now-st.stamp < .1 and data[2] == (st.counter+1) % 256 else 1
      st.stamp, st.counter = now, data[2]
      if address == 0x16A:
        self.stock_direction = data[16] >> 5
      if self.state == 'active':
        self.reason = 'stock_conflict'
    elif address == RESPONSE and len(data) == 8:
      # Ignore multi-frame/unsolicited replies. Only these short positive
      # responses advance the state; negative/pending replies never do.
      n = data[0]
      if 2 <= n <= 7:
        self.reply = (now, bytes(data[1:1+n]))

  def stock_ready(self, now):
    return all(s.consecutive >= 10 and 0 <= now-s.stamp < .1 for s in self.stock.values())

  def acknowledged(self, prefix):
    return self.reply is not None and self.changed <= self.reply[0] and self.reply[1].startswith(prefix)

  def enter(self, state, now):
    self.state, self.changed, self.reply = state, now, None

  def restore(self, now, reason):
    self.reason = reason
    self.restore_attempts = 1
    self.restore_ack = False
    self.stock_resumed = False
    self.enter('restore_enable', now)
    return [CanData(APRK, ENABLE, self.bus)]

  def update(self, now, *, healthy, request=None, direction=None, duration=300., driver_override=False):
    continuous = self.last_update is not None and 0 <= now-self.last_update < .2
    self.last_update = now
    changed_request = request != self.previous
    self.previous = request
    allowed = (healthy and continuous and not driver_override and direction in ('left', 'right') and
               isinstance(request, str) and 0 < len(request) <= 100)
    if self.state == 'idle':
      if changed_request and allowed and self.stock_ready(now):
        self.request, self.direction, self.started = request, direction, now
        self.duration = max(.1, min(300., duration))
        self.reason = ''
        self.enter('extended', now)
        return [CanData(APRK, EXTENDED, self.bus)]
      return []
    if self.state == 'failed':
      # No automatic reacquisition after a failed restore; require a restart
      # and fresh stock reception. In particular a held request is never replayed.
      return []
    if self.state.startswith('restore_'):
      if self.state == 'restore_enable':
        if self.acknowledged(b'\x68\x00') or now-self.changed >= .4:
          self.restore_ack = self.acknowledged(b'\x68\x00')
          self.enter('restore_default', now)
          return [CanData(APRK, DEFAULT, self.bus)]
      elif self.state == 'restore_default':
        if self.acknowledged(b'\x50\x01'):
          self.restore_ack = True
          self.enter('restore_stock', now)
        elif now-self.changed >= .4:
          self.enter('restore_stock', now)
      elif self.stock_ready(now):
        self.stock_resumed = True
        if self.restore_ack:
          self.request = self.direction = None
          self.enter('idle', now)
          return []
      if now-self.changed >= 1.:
        if self.restore_attempts >= 3:
          self.enter('failed', now)
          self.reason = 'restore_unconfirmed'
        else:
          self.restore_attempts += 1
          self.enter('restore_enable', now)
          return [CanData(APRK, ENABLE, self.bus)]
      return []
    if not allowed or request != self.request or direction != self.direction or now-self.started >= self.duration or self.reason:
      return self.restore(now, 'driver_override' if driver_override else self.reason or 'request_ended')
    if self.reply is not None and self.reply[0] >= self.changed and self.reply[1][:1] == b'\x7f':
      return self.restore(now, 'ecu_rejected')
    if self.state == 'extended':
      if self.acknowledged(b'\x50\x03'):
        self.enter('disable', now)
        return [CanData(APRK, DISABLE, self.bus)]
      if now-self.changed >= .4:
        return self.restore(now, 'session_timeout')
    elif self.state == 'disable':
      if self.acknowledged(b'\x68\x03'):
        self.enter('quiet', now)
      elif now-self.changed >= .4:
        return self.restore(now, 'disable_timeout')
    elif self.state == 'quiet':
      if now-self.changed >= .1 and all(now-s.stamp >= .1 for s in self.stock.values()):
        self.counter = (self.stock[0x16A].counter+1) % 256
        self.enter('active', now)
        self.next_send, self.keepalive = now, now+1.
      elif now-self.changed >= .4:
        return self.restore(now, 'ecu_not_quiet')
    if self.state == 'active':
      sends = []
      if now >= self.keepalive:
        sends.append(CanData(APRK, PRESENT, self.bus))
        self.keepalive = now+1.
      if now >= self.next_send:
        sends.extend(frame(a, self.counter, self.direction, self.bus) for a in SPAS)
        self.counter = (self.counter+1) % 256
        self.next_send = now+.02
      return sends
    return []


class PatchSequence:
  """Contributor f8448120/04dd4119 wire sequence; caller owns parked guards.

  Uses the actual controller frame and native create_spas_messages callback.
  No response/stock-silence condition is added to this sequence.
  """
  def __init__(self, bus, make_spas):
    self.bus, self.make_spas = bus, make_spas
    self.state, self.started_frame = 0, 0

  def step(self, controller_frame, want, direction):
    if not want:
      sends = [CanData(APRK, DEFAULT, self.bus)] if self.state else []
      self.state = 0
      return sends
    if self.state == 0:
      self.state, self.started_frame = 1, controller_frame
      return [CanData(APRK, EXTENDED, self.bus)]
    if self.state == 1:
      if controller_frame-self.started_frame >= 5:
        self.state = 2
        return [CanData(APRK, PATCH_DISABLE, self.bus)]
      return []
    sends = [CanData(APRK, PRESENT, self.bus)] if controller_frame % 100 == 0 else []
    if controller_frame % 2 == 0:
      sends.extend(self.make_spas(controller_frame, direction))
    return sends
