"""Comma SPAS transport for the existing physical-indicator intent policy."""
import math

from opendbc.car.hyundai.spas import Session, frame, SPAS
from opendbc.car.hyundai.values import HyundaiFlags
from opendbc.car.hyundai.hyundaicanfd import CanBus
from tools.jetlink.blink_protocol import Process, fresh  # noqa: TID251 -- shared standalone indicator worker


class SpasSignals:
  def __init__(self, cp, params):
    self.enabled = cp.brand == 'hyundai' and bool(cp.flags & HyundaiFlags.CANFD_SPAS_BLINKERS) and not cp.passive
    self.params = params
    self.session = Session(CanBus(cp).ECAN) if self.enabled else None
    self.worker = None
    self.feedback_at = self.settings_at = 0.
    self.worker_retry = 0.
    self.test_setting = 0
    self.test_request = None
    self.test_until = 0.
    self.test_direction = None
    self.test_pulse = False
    self.test_direct = False
    self.test_hold = False
    self.direct_next = self.direct_started = 0.
    self.direct_cooldown = 0.
    self.direct_counter = {}
    self.phase_sent = {}
    self.stock_received = {}
    self.last_update = None
    self.test_blocked = ()
    self.test_serial = 0
    self.stalks = None
    self.driver_override = False
    self.hold_value = 1
    self.hold_supported = cp.carFingerprint == 'HYUNDAI_IONIQ_5_PE'
    self.pulse_started = None
    self.lamp_confirmed = False
    self.intent = None
    self.driver_at = -math.inf
    self.driver_counter = -1
    self.driver_count = 0
    self.driver_active = False
    self.status = 'disabled' if not self.enabled else 'waiting_stock'

  def observe(self, can_list, now):
    if self.enabled:
      for stamp, packets in can_list:
        for address, data, bus in packets:
          self.session.observe(address, bus, data, now)
          if address in SPAS and bus == self.session.bus:
            received = stamp / 1e9
            if 0 <= now-received < .1:
              self.stock_received[address] = received
          if address == 0x3C1 and bus == self.session.bus:
            crc = 0
            for byte in data[1:]:
              crc ^= byte
              for _ in range(8):
                crc = ((crc << 1) ^ (0x1d if crc & 0x80 else 0)) & 255
            if len(data) == 8 and data[0] == crc ^ 0x65:
              count = data[1] >> 4
              # The event counter stays constant while idle. Repeated valid
              # 5 Hz CAN reception is fresh; this is not a SPAS cycle counter.
              continuous = 0 <= now-self.driver_at < .5
              self.driver_count = min(10, self.driver_count+1) if continuous else 1
              self.driver_at, self.driver_counter = now, count
              self.driver_active = bool(data[3] & 0x40 or data[4] & 0x51)
              self.driver_override |= self.driver_active
            else:
              self.driver_count = 0

  def update(self, cs, sm, now, initialized):
    if not self.enabled:
      return []
    continuous = self.last_update is not None and 0 <= now-self.last_update < .2
    self.last_update = now
    if self.worker is None and initialized and now >= self.worker_retry:
      try:
        self.worker = Process('blink_source.py', 'spas')
      except OSError:
        self.worker_retry = now+5.
    healthy = (initialized and cs.canValid and math.isfinite(cs.vEgo) and
               self.driver_count >= 3 and 0 <= now-self.driver_at < .5 and
               str(cs.gearShifter) in ('park', 'neutral', 'drive') and
               all(sm.valid[k] and sm.alive[k] and 0 <= now-sm.recv_time[k] < .35 for k in ('carControl', 'selfdriveState')))
    stalks = (cs.leftBlinkerStalkCount, cs.rightBlinkerStalkCount)
    override = bool(cs.blinkerLever or (cs.leftBlinker and cs.rightBlinker) or self.stalks is not None and stalks != self.stalks)
    self.stalks = stalks
    if override:
      self.driver_override = True
    if self.session.state == 'idle' and not override and not self.driver_active and self.driver_count >= 3:
      self.driver_override = False
    if now >= self.settings_at:
      setting = self.params.get_int('CanfdBlinkerTest')
      if setting != self.test_setting and setting in range(1, 13):
        # Consume the edge even when the parked guard fails; never run later.
        cc, ss = sm['carControl'], sm['selfdriveState']
        parked = (healthy and str(cs.gearShifter) == 'park' and cs.standstill and abs(cs.vEgo) < .01 and
                  not (cc.latActive or cc.longActive or ss.active or ss.enabled) and
                  sm.valid['pandaStates'] and sm.alive['pandaStates'] and 0 <= now-sm.recv_time['pandaStates'] < .75 and
                  len(sm['pandaStates']) > 0 and not any(p.controlsAllowed for p in sm['pandaStates']))
        direct = setting >= 5
        direct_ready = (not direct or (now >= self.direct_cooldown and self.session.stock_ready(now) and
                        self.session.stock_direction == 0 and not (cs.leftBlinker or cs.rightBlinker)))
        if parked and (setting <= 2 or self.hold_supported) and self.session.state == 'idle' and direct_ready:
          self.test_serial += 1
          self.test_request = f'park:{self.test_serial}'
          # The extended comparison sends only the initial 0.2 s SPAS pulse;
          # the existing lamp-confirmed hold supplies the remaining time.
          self.test_until = now+(10. if setting in (7, 8, 11, 12) else 3.)
          self.test_direction = 'left' if setting % 2 else 'right'
          self.test_pulse = setting in (3, 4, 7, 8, 9, 10, 11, 12)
          self.test_direct = direct
          self.test_hold = setting in (3, 4, 7, 8, 11, 12)
          self.test_blocked = tuple(getattr(p, 'safetyTxBlocked', 0) for p in sm['pandaStates'])
          if direct:
            self.direct_started = self.direct_next = now
            self.phase_sent.clear()
            self.direct_cooldown = now+30.
            self.direct_counter = {a: (s.counter+1) % 256 for a, s in self.session.stock.items()}
          self.pulse_started = None
          self.lamp_confirmed = False
      self.test_setting = setting
      self.settings_at = now+.1
    record = self.worker.latest() if self.worker is not None else None
    if record is not None:
      self.intent = record
    record = self.intent or {}
    hold = record.get('hold')
    request = direction = None
    if fresh(record.get('stamp'), now, .2) and record.get('ok') is True:
      if isinstance(hold, dict) and hold.get('direction') in ('left', 'right'):
        request, direction = hold.get('id'), hold['direction']
      else:
        taps = record.get('events', [])
        taps = [e for e in taps if isinstance(e, dict) and fresh(e.get('time'), now, .4)]
        if taps:
          request, direction = taps[-1].get('id'), taps[-1].get('direction')
    if self.test_request is not None:
      cc, ss = sm['carControl'], sm['selfdriveState']
      healthy = (healthy and str(cs.gearShifter) == 'park' and cs.standstill and abs(cs.vEgo) < .01 and
                 not (cc.latActive or cc.longActive or ss.active or ss.enabled) and
                 sm.valid['pandaStates'] and sm.alive['pandaStates'] and 0 <= now-sm.recv_time['pandaStates'] < .75 and
                 len(sm['pandaStates']) > 0 and not any(p.controlsAllowed for p in sm['pandaStates']))
      if now < self.test_until and self.test_setting != 0:
        request, direction = self.test_request, self.test_direction
      else:
        request = direction = None
    direct = self.test_request is not None and self.test_direct
    # A selected parked direct test never falls through to automatic/UDS intent.
    parked_selected = direct or self.test_setting >= 5 or now < self.direct_cooldown
    sends = self.session.update(now, healthy=healthy, request=None if parked_selected else request,
                                direction=None if parked_selected else direction,
                                duration=3. if self.test_request is not None else 300., driver_override=self.driver_override)
    self.hold_value = 1  # suppress the old USB2CAN/automatic-lane hold while selected
    direct_active = (direct and request == self.test_request and healthy and not self.driver_override and
                     continuous and self.test_blocked == tuple(getattr(p, 'safetyTxBlocked', 0) for p in sm['pandaStates']) and
                     self.session.stock_ready(now) and self.session.stock_direction == 0)
    if direct and not direct_active:
      self.test_until = min(self.test_until, now)  # no resumption after guard loss
    if direct_active and now >= self.direct_next and (not self.test_pulse or now-self.direct_started < .2):
      if self.test_setting in (11, 12):
        # At most one packet per new stock RX during the initial 0.2 s.
        # This is a bounded parked phase trial, not a wire-time guarantee.
        for address in SPAS:
          received = self.stock_received.get(address, -math.inf)
          previous_rx, previous_tx, count = self.phase_sent.get(address, (-math.inf, -math.inf, 0))
          if (count < 10 and max(self.direct_started, previous_rx) < received <= now and
              now-received <= .005 and now-previous_tx >= .015 and (address == 0x165 or 0x165 in self.phase_sent)):
            counter = (self.session.stock[address].counter+1) % 256 if count == 0 else self.direct_counter[address]
            sends.append(frame(address, counter, direction, self.session.bus))
            self.direct_counter[address] = (counter+1) % 256
            self.phase_sent[address] = (received, now, count+1)
      else:
        sends.extend(frame(a, self.direct_counter[a], direction, self.session.bus) for a in SPAS)
        self.direct_counter = {a: (c+1) % 256 for a, c in self.direct_counter.items()}
        self.direct_next = now+.02
    if (direct_active or self.session.state == 'active') and self.test_request is not None and self.test_pulse:
      if self.pulse_started is None:
        self.pulse_started = now
      self.lamp_confirmed |= ((direction == 'left' and cs.leftBlinker and not cs.rightBlinker) or
                              (direction == 'right' and cs.rightBlinker and not cs.leftBlinker))
      if self.lamp_confirmed and self.test_hold:
        self.hold_value = 2 if direction == 'left' else 3
      if not direct and now-self.pulse_started >= .2:
        # Keep the SPAS session alive at its normal cadence, but remove the
        # directional command. Only the existing confirmed 0x1EA hold remains.
        sends = [frame(m.address, m.dat[2], None, m.src) if m.address == 0x16A else m for m in sends]
    self.status = ('parked_direct' if direct_active else 'parked_direct_ended') if direct else self.session.reason or self.session.state
    if self.worker is not None and now >= self.feedback_at:
      self.worker.send({'stamp': now, 'accepted_id': self.session.request if self.session.state == 'active' else None,
                        'state': 'driver_override' if self.driver_override else 'ready' if self.session.state == 'active' else 'waiting'})
      self.feedback_at = now+.05
    if self.test_request is not None and self.session.state in ('idle', 'failed') and now >= self.test_until:
      self.test_request = None
    return sends
