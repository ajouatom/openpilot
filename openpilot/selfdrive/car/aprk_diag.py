"""One-shot parked APRK session probe. Never disables communication or sends SPAS."""
import json
import math

from opendbc.car import CanData
from opendbc.car.hyundai.spas import APRK, RESPONSE, EXTENDED, DEFAULT
from opendbc.car.hyundai.values import HyundaiFlags

KEY = 'CanfdAprkDiag'
RESULT = 'CanfdAprkDiagResult'
CONFLICTS = ('CanfdBlinkerControl', 'CanfdBlinkerKick', 'CanfdBlinkerTest', 'CanfdBlinkerMenuTest',
             'CanfdBlinkerStalk', 'SpasSteerTest', 'SteerAngleTest')
RESERVED = (APRK, 0x165, 0x16A, 0x3C1)


class AprkDiag:
  def __init__(self, cp, params, device_type, log_event):
    flags = HyundaiFlags.CANFD | HyundaiFlags.CANFD_HDA2 | HyundaiFlags.CANFD_CAMERA_SCC
    self.supported = (device_type == 'mici' and cp.brand == 'hyundai' and
                      cp.flags & flags == flags and not cp.passive and
                      not cp.flags & (HyundaiFlags.CANFD_SPAS_BLINKERS | HyundaiFlags.ENABLE_BLINKERS) and
                      len(cp.safetyConfigs) == 1 and str(cp.safetyConfigs[0].safetyModel) == 'hyundaiCanfd')
    self.safety_param = cp.safetyConfigs[0].safetyParam if self.supported else None
    self.params, self.log_event = params, log_event
    self.phase = 'idle'
    self.result = {}
    self.bus = 0
    self.requested = False
    self.conflict = False
    self.poll_at = 0.
    self.sent_at = self.deadline = 0.
    self.reply = None
    self.health = None
    self.cancelled = False
    self.clock_offset = 0.

  @property
  def busy(self):
    return self.phase != 'idle'

  def save(self):
    self.result['phase'] = self.phase
    self.params.put_nonblocking(RESULT, json.dumps(self.result, ensure_ascii=False))
    self.log_event('canfd_aprk_diag', **self.result)

  def finish(self, reason):
    self.result['status'] = reason
    self.phase = 'idle'
    self.requested = False
    self.params.put_bool(KEY, False)
    self.save()

  def send(self, now, phase):
    self.phase, self.sent_at, self.deadline, self.reply = phase, now, now + .8, None
    entry = self.result['buses'][self.bus]
    entry[phase] = {'status': 'waiting', 'sent_boot': now, 'echo': 0, 'rejected': 0}
    self.save()
    return [CanData(APRK, EXTENDED if phase == 'session' else DEFAULT, self.bus)]

  def observe(self, can_list, now):
    if not self.busy:
      return
    entry = self.result['buses'][self.bus]
    for stamp, packets in can_list:
      received = stamp / 1e9
      if not self.sent_at <= received <= now or now - received > .5:
        continue
      for address, data, bus in packets:
        if address in (0x165, 0x16A) and bus == self.bus:
          entry['stock_frames'] += 1
        payload = EXTENDED if self.phase == 'session' else DEFAULT
        if address == APRK and bytes(data) == payload:
          if bus == self.bus + 128:
            entry[self.phase]['echo'] += 1
          elif bus == self.bus + 192:
            entry[self.phase]['rejected'] += 1
        if address != RESPONSE or bus != self.bus or len(data) != 8 or self.reply is not None:
          continue
        n = data[0]
        if not 2 <= n <= 7:
          continue
        reply = bytes(data[1:1+n])
        sub = 3 if self.phase == 'session' else 1
        if reply[:2] == bytes((0x50, sub)):
          self.reply = {'status': 'positive', 'raw': reply.hex(), 'at': received}
        elif len(reply) >= 3 and reply[:2] == b'\x7f\x10':
          if reply[2] == 0x78:
            entry[self.phase]['pending'] = reply.hex()
            # Pending responses extend one request only up to a bounded deadline.
            self.deadline = min(self.sent_at + 2., max(self.deadline, received + .8))
          else:
            self.reply = {'status': 'negative', 'nrc': reply[2], 'raw': reply.hex(), 'at': received}

  @staticmethod
  def can_health(pandas):
    values = []
    if len(pandas) != 1:
      return None
    for panda in pandas:
      values.append(panda.safetyTxBlocked)
      for i in range(3):
        state = getattr(panda, f'canState{i}', None)
        if state is None or any(getattr(state, k, True) for k in ('busOff', 'errorWarning', 'errorPassive')):
          return None
        for k in ('busOffCnt', 'totalErrorCnt', 'totalTxLostCnt', 'totalRxLostCnt'):
          value = getattr(state, k, None)
          if not isinstance(value, int) or value < 0:
            return None
          values.append(value)
    return tuple(values)

  def guard(self, cs, sm, now, initialized, owner_busy):
    if not self.supported:
      return 'unsupported_vehicle_or_device'
    if self.conflict or owner_busy:
      return 'other_spas_test_active'
    if not initialized or not cs.canValid:
      return 'vehicle_not_ready'
    for name, age in (('carControl', .35), ('selfdriveState', .35), ('pandaStates', .75)):
      if not (sm.valid[name] and sm.alive[name] and 0 <= now - sm.recv_time[name] < age):
        return 'stale_' + name
    if str(cs.gearShifter) != 'park' or not cs.standstill or not math.isfinite(cs.vEgo) or abs(cs.vEgo) >= .01:
      return 'park_and_standstill_required'
    cc, ss = sm['carControl'], sm['selfdriveState']
    if cc.enabled or cc.latActive or cc.longActive or ss.enabled or ss.active or cs.gasPressed:
      return 'controls_or_accelerator_active'
    if any(p.controlsAllowed for p in sm['pandaStates']):
      return 'panda_controls_allowed'
    if any(str(p.safetyModel) != 'hyundaiCanfd' or p.safetyParam != self.safety_param for p in sm['pandaStates']):
      return 'panda_safety_not_ready'
    if self.can_health(sm['pandaStates']) is None:
      return 'can_unhealthy'
    return None

  def update(self, cs, sm, now, can_now, initialized, owner_busy=False):
    if now >= self.poll_at:
      self.requested = self.params.get_bool(KEY)
      self.conflict = any(self.params.get_int(k) != 0 for k in CONFLICTS) if self.requested or self.busy else False
      self.poll_at = now + .1
    if not self.requested and not self.busy:
      return []
    reason = self.guard(cs, sm, now, initialized, owner_busy)
    health = self.can_health(sm['pandaStates'])
    if not self.busy:
      self.result = {'started_mono': now, 'status': 'running', 'buses': [], 'scope': 'session_only'}
      if reason:
        self.finish(reason)  # consume invalid requests, never start later while driving
        return []
      self.bus, self.health, self.cancelled = 0, health, False
      self.clock_offset = can_now - now
      self.result['buses'] = [{'bus': b, 'stock_frames': 0} for b in (0, 1)]
      return self.send(can_now, 'session')

    if health != self.health:
      reason = reason or 'can_counters_changed'
    if abs(can_now - now - self.clock_offset) > .1:
      reason = reason or 'suspend_during_probe'
    if (not self.requested or reason) and not self.cancelled:
      self.result['status'] = reason or 'cancelled'
      self.cancelled = True
      if self.phase == 'session':
        entry = self.result['buses'][self.bus]['session']
        entry.update(self.reply or {'status': 'cancelled'})
        return self.send(can_now, 'restore')

    if self.reply is None and can_now < self.deadline:
      return []
    entry = self.result['buses'][self.bus][self.phase]
    entry.update(self.reply or {'status': 'timeout'})
    if self.phase == 'session':
      # Restore even after a timeout: the ECU might have entered the session silently.
      return self.send(can_now, 'restore')
    self.save()
    if self.cancelled:
      self.finish(self.result['status'])
      return []
    if self.bus == 0:
      # A confirmed session with unconfirmed restoration must not acquire another ECU.
      if self.result['buses'][0]['session']['status'] == 'positive' and entry['status'] != 'positive':
        self.finish('restore_unconfirmed')
        return []
      self.bus = 1
      return self.send(can_now, 'session')
    self.finish('complete' if all(b['restore']['status'] == 'positive' for b in self.result['buses']) else 'complete_restore_unconfirmed')
    return []
