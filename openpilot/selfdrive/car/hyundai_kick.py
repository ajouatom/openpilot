"""Coordinate native Kick output with external adapters and model lamp isolation."""
from collections import deque

from opendbc.car.hyundai.values import HyundaiSafetyFlags
from opendbc.car.hyundai.hyundaicanfd import CanBus
from opendbc.car.hyundai.stalk_trial import crc
from opendbc.car.hyundai.spas import Session, SPAS
from openpilot.selfdrive.carrot.bluetooth.model import atomic_json, read_json, RUNTIME
from tools.jetlink.blink_owner import Owner  # noqa: TID251 -- shared physical output lease
from tools.jetlink.blink_protocol import Process, fresh  # noqa: TID251 -- common indicator policy worker


class KickOutput:
  def __init__(self, cp, params, *, event=None):
    self.configured = (cp.brand == 'hyundai' and not cp.passive and
                       any(cfg.safetyParam & HyundaiSafetyFlags.CANFD_BLINKER_KICK for cfg in cp.safetyConfigs))
    self.params = params
    self.event = event
    self.owner = Owner('CANFD Kick')
    self.bus = CanBus(cp).ECAN if self.configured else None
    self.contract = Session(self.bus)
    self.contract_admitted = False
    self.next_update = self.last_journal = 0.
    self.direction = 0
    self.driver_stamp = self.driver_pressed_at = -1.
    self.driver_direction = None
    self.driver_counts = [0, 0]
    self.source_counts = None
    self.worker = None
    self.worker_retry = self.feedback_at = 0.
    self.record = {}
    self.seen = deque(maxlen=256)
    self.request_id = None
    self.request_side = 0
    self.accepted_id = None
    self.last_sent = -1.
    self.first_sent = None
    self.request_started = self.tap_until = 0.
    self.tapping = False
    self.ready = False

  def observe_driver(self, frames, cs, now):
    if not self.configured:
      return
    # USB/BLE can synthesize 0x3C1. Trust this independent lever message only
    # after Kick has acquired the shared lease and drained their commands.
    exclusive = (self.owner.held and 0 <= now - self.owner.renewed < .2 and
                 self.params.get_bool('CanfdBlinkerKick') and self.params.get_bool('IsOnroad') and cs.canValid)
    source_counts = (getattr(cs, 'leftBlinkerStalkCount', 0), getattr(cs, 'rightBlinkerStalkCount', 0))
    # Preserve ordinary lever events while Kick is off or awaiting ownership.
    # Apply deltas so changing the source cannot manufacture a lever press.
    if self.source_counts is None:
      self.driver_counts = list(source_counts)
    elif not exclusive:
      for index in range(2):
        self.driver_counts[index] = (self.driver_counts[index] + source_counts[index] - self.source_counts[index]) % 256
    self.source_counts = source_counts
    if not exclusive or not 0 <= now - self.driver_stamp < .35:
      self.driver_direction = None
    if exclusive:
      for stamp, packets in frames:
        for address, data, bus in packets:
          if address in SPAS and bus == self.bus:
            if 0 <= now - stamp / 1e9 < .1:
              self.contract.observe(address, bus, data, stamp / 1e9)
              if self.contract.stock[address].consecutive == 0 or (
                  self.request_id is not None and self.contract.stock[address].consecutive < 2):
                self.contract_admitted = False
            else:
              self.contract.stock[address].consecutive = 0
              self.contract_admitted = False
          if address != 0x3C1 or bus != self.bus:
            continue
          if len(data) != 8 or data[0] != crc(data[1:]) or not 0 <= now - stamp / 1e9 < .35:
            self.driver_direction, self.driver_stamp = None, -1.
            continue
          direction = int(bool(data[3] & 0x40)) | (int(bool(data[4] & 1)) << 1)
          # Establish neutral after startup, loss or an invalid frame before
          # accepting a new press. Count even a press/release in one CAN batch.
          if self.driver_direction is not None:
            if direction in (1, 2) and not self.driver_direction & direction:
              index = direction - 1
              self.driver_counts[index] = (self.driver_counts[index] + 1) % 256
              self.driver_pressed_at = now
            self.driver_direction = direction
          elif direction == 0:
            self.driver_direction = 0
          self.driver_stamp = stamp / 1e9
    cs.turnSignalDriverValid = exclusive and self.driver_direction is not None and 0 <= now - self.driver_stamp < .35
    cs.turnSignalDriverDirection = self.driver_direction if cs.turnSignalDriverValid else 0
    cs.leftBlinkerStalkCount, cs.rightBlinkerStalkCount = self.driver_counts

  def update(self, valid, now):
    enabled = self.configured and valid and self.params.get_bool('IsOnroad') and self.params.get_bool('CanfdBlinkerKick')
    if not enabled or now >= self.next_update:
      self.owner.acquire(enabled)
      self.next_update = now + .05
    if not enabled or not self.owner.held:
      self.contract = Session(self.bus)
      self.contract_admitted = False
    # Require ten advancing CRC-valid stock frames before a new request.
    # During an admitted session the ECU may deliberately go quiet; the
    # independent driver message must remain fresh throughout that session.
    stock_ready = self.contract.stock_ready(now)
    if self.request_id is None:
      self.contract_admitted = stock_ready
    driver_active = self.driver_direction not in (None, 0) or 0 <= now - self.driver_pressed_at < .2
    driver_valid = self.driver_direction is not None and 0 <= now - self.driver_stamp < .35
    return enabled and self.owner.held and self.contract_admitted and driver_valid and not driver_active

  def request(self, cs, now, initialized, ready, *, clock=None):
    """Translate the existing source's fresh hold/tap into a leased SPAS direction."""
    enabled = self.configured and self.params.get_bool('IsOnroad') and self.params.get_bool('CanfdBlinkerKick')
    if self.worker is not None and (not enabled or self.worker.process.poll() is not None):
      self.worker.close(wait=False)
      self.worker = None
      self.record = {}
      self.worker_retry = now + 1.
    if enabled and initialized and self.worker is None and now >= self.worker_retry:
      try:
        self.worker = Process('blink_source.py', 'kick')
      except OSError:
        self.worker_retry = now + 5.
    record = self.worker.latest() if self.worker is not None else None
    # The producer may publish while latest() drains the socket. Judge its
    # timestamp after the read, using the caller's live or replay clock.
    if clock is not None:
      now = clock()
    if isinstance(record, dict):
      self.record = record
    record = self.record
    self.ready = bool(enabled and ready and initialized and cs.canValid and str(cs.gearShifter) == 'drive' and
                      not cs.blinkerLever and not (cs.leftBlinker and cs.rightBlinker))
    healthy = self.ready and fresh(record.get('stamp'), now, .2) and record.get('ok') is True
    hold = record.get('hold')
    candidate = hold if isinstance(hold, dict) else None
    if candidate is None:
      events = record.get('events', [])
      if isinstance(events, list):
        candidate = next((event for event in reversed(events[-4:]) if isinstance(event, dict) and
                          fresh(event.get('time'), now, .4)), None)
    candidate = candidate or {}
    ident = candidate.get('id')
    side = {'left': 3, 'right': 4}.get(candidate.get('direction'), 0)
    valid_request = isinstance(ident, str) and 0 < len(ident) <= 100 and side != 0
    if self.request_id is not None:
      continuing = (self.tapping and now < self.tap_until or
                    not self.tapping and valid_request and ident == self.request_id and side == self.request_side)
      opposite = cs.rightBlinker if self.request_side == 3 else cs.leftBlinker
      if not healthy or not continuing or opposite or now - self.request_started >= 300.:
        if self.event is not None:
          self.event('canfdIndicatorRequestEnd', request=self.request_id, side=self.request_side,
                     now=now, started=self.request_started, first_sent=self.first_sent,
                     healthy=healthy, continuing=bool(continuing), opposite=bool(opposite), ready=self.ready,
                     source_stamp=record.get('stamp'), source_ok=record.get('ok'),
                     source_origin=record.get('indicator_origin'))
        self.request_id, self.request_side, self.accepted_id = None, 0, None
    if valid_request and ident not in self.seen:
      # Consume rejected/expired requests too. Recovery requires a new policy edge.
      self.seen.append(ident)
      if healthy and self.request_id is None and fresh(candidate.get('time'), now, .4) and not (cs.leftBlinker or cs.rightBlinker):
        self.request_id, self.request_side = ident, side
        self.request_started, self.tap_until = now, now + 1.6
        self.first_sent = None
        self.tapping = not isinstance(hold, dict)
    return self.request_side if healthy else 0

  def feedback(self, cs, now, frames):
    """Acknowledge submitted SPAS frames; the common source separately observes the lamps."""
    sent = self.request_id is not None and any(
      address == 0x16A and len(data) == 32 and (data[16] >> 5) & 7 == self.request_side
      for address, data, _ in frames)
    if sent:
      if self.first_sent is None:
        self.first_sent = now
      if self.tapping and self.accepted_id != self.request_id:
        self.tap_until = now + 1.
      self.accepted_id = self.request_id
      self.last_sent = now
    elif not fresh(self.last_sent, now, .2):
      self.accepted_id = None
    if self.worker is not None and now >= self.feedback_at:
      driver = cs.blinkerLever or self.driver_direction not in (None, 0) or now - self.driver_pressed_at < .2
      state = 'driver_override' if driver else 'ready' if self.ready else 'waiting'
      self.worker.send({'stamp': now, 'state': state, 'available': self.ready,
                        'accepted_id': self.accepted_id if self.ready else None,
                        'first_sent': self.first_sent if self.ready and self.accepted_id else None})
      self.feedback_at = now + .05

  def isolate(self, frames, model, now):
    if not self.configured:
      return frames
    directions = {data[16] >> 5 for addr, data, _ in frames if addr == 0x16A and len(data) == 32}
    direction = 1 if 3 in directions else 2 if 4 in directions else 0
    if not direction:
      return frames
    if self.owner.held:
      try:
        if direction != self.direction or now - self.last_journal >= .1:
          previous = read_json(RUNTIME/'boundary-indicator.json', {})
          previous = previous if isinstance(previous, dict) else {}
          mask = previous.get('mask', 0)
          if type(mask) is not int or mask not in (1, 2, 3) or not 0 <= now - previous.get('time', -100.) < 20:
            mask = 0
          atomic_json(RUNTIME/'boundary-indicator.json', {'time':now, 'direction':direction, 'mask':mask | direction})
          self.direction, self.last_journal = direction, now
        if model is not None and model.meta.indicatorIsolationMask & direction:
          return frames
      except (OSError, TypeError, ValueError):
        pass
    # Wait for modeld to acknowledge ownership before physical lamp feedback can arrive.
    return [frame for frame in frames if frame[0] not in (0x165, 0x16A)]
