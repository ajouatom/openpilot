"""Read-only manual boundary approach experiment. No lamp/steering output API.

All thresholds are observation hypotheses, not calibrated vehicle/tire geometry.
A predicted crossing does not identify whether the driver is changing lanes or
avoiding an obstacle. Post-event outcomes are geometric observations only.
"""
from collections import Counter, deque
import math

HALF_WIDTH_PROXY = .9  # Explicit hypothesis; CarParams has no measured tire span.


class BoundaryObserver:
  def __init__(self, emit):
    self.emit = emit
    self.previous = None
    self.confident = [None, None]
    self.rates = [0., 0.]
    self.torque_direction = 0
    self.torque_since = 0.
    self.candidate = None
    self.serial = 0
    self.rearm_until = 0.
    self.next_report = 0.
    self.counts = Counter()
    self.events = deque(maxlen=8)
    self.dropped_events = 0
    self.last_sample = None

  def _event(self, now, reason, **values):
    self.counts[reason] += 1
    if len(self.events) == self.events.maxlen:
      self.dropped_events += 1
    self.events.append(dict(time=round(now, 3), reason=reason, **values))

  def _reset(self, now, reason):
    if self.candidate is not None:
      values = {**(self.last_sample or {}), 'candidate': self.candidate['id'], 'direction': self.candidate['direction']}
      self._event(now, reason, **values)
    self.candidate, self.previous, self.last_sample = None, None, None
    self.rates, self.confident = [0., 0.], [None, None]
    self.torque_direction = 0
    self.counts['gate_' + reason] += 1

  def observe(self, now, cs, md, *, enabled, valid, busy, legacy_direction=0):
    # Logging or malformed observations must never interrupt model publication.
    try:
      self._observe(now, cs, md, enabled=enabled, valid=valid, busy=busy, legacy_direction=legacy_direction)
    except Exception:
      self._reset(now, 'observer_error')
    if now >= self.next_report:
      self.next_report = now + 1.
      if self.events or (enabled and self.counts):
        payload = {'version': 1, 'observation_only': True, 'half_width_proxy_m': HALF_WIDTH_PROXY,
                       'counts': dict(self.counts), 'events': list(self.events), 'dropped_events': self.dropped_events,
                       'sample': self.last_sample}
        self.counts.clear()
        self.events.clear()
        self.dropped_events = 0
        try:
          self.emit('manualBoundaryObservation', **payload)
        except Exception:
          pass
      elif not enabled:
        self.counts.clear()

  def _observe(self, now, cs, md, *, enabled, valid, busy, legacy_direction):
    if not enabled or not valid or busy:
      self._reset(now, 'disabled' if not enabled else 'invalid' if not valid else 'automatic_maneuver')
      return
    if (not cs.canValid or str(cs.gearShifter) != 'drive' or cs.trailerConnected or
        cs.steerFaultTemporary or cs.steerFaultPermanent or not math.isfinite(cs.vEgo) or cs.vEgo < 30/3.6):
      self._reset(now, 'vehicle')
      return
    if cs.leftBlinker or cs.rightBlinker or cs.blinkerLever:
      self._reset(now, 'existing_signal')
      return
    left, right = md.laneLines[1].y[0], md.laneLines[2].y[0]
    probs = (md.laneLineProbs[1], md.laneLineProbs[2])
    if not all(math.isfinite(v) for v in (left, right, *probs, cs.steeringTorque)) or not 2.5 < right-left < 4.8:
      self._reset(now, 'geometry')
      return
    y = (-left, right)
    old, self.previous = self.previous, (md.frameId, now, y)
    for i in (0, 1):
      if probs[i] > .5:
        self.confident[i] = now
    if old is None:
      return
    frame, stamp, previous_y = old
    dt = now-stamp
    if md.frameId <= frame or not .02 <= dt <= .15:
      self._reset(now, 'frame_gap')
      return
    if max(abs(y[i]-previous_y[i]) for i in (0, 1)) > .4:
      self._reset(now, 'boundary_jump')
      self.rearm_until = now+2.
      return
    self.rates = [.5*self.rates[i] + .5*(previous_y[i]-y[i])/dt for i in (0, 1)]
    direction = (1 if cs.steeringTorque > 0 else 2 if cs.steeringTorque < 0 else 0) if cs.steeringPressed else 0
    if direction != self.torque_direction:
      self.torque_direction, self.torque_since = direction, now
    confirmed = direction if direction and now-self.torque_since >= .15 else 0
    if self.candidate is None and confirmed and now >= self.rearm_until and left < 0 < right:
      self.serial += 1
      self.candidate = {'id': self.serial, 'direction': direction, 'started': now, 'intent': now,
                            'initial': y[direction-1], 'minimum': y[direction-1], 'emitted': False}
      self._event(now, 'candidate', candidate=self.serial, direction=direction)
    c = self.candidate
    if c is None:
      self.counts['no_candidate'] += 1
      return
    i = c['direction']-1
    if confirmed == c['direction']:
      c['intent'] = now
    clearance = y[i]-HALF_WIDTH_PROXY
    rate = self.rates[i]
    ttc = max(0., clearance)/rate if rate > .1 else None
    self.last_sample = {'frame': int(md.frameId), 'sample_time': round(now, 3), 'candidate': c['id'], 'direction': c['direction'],
                            'clearance_m': round(clearance, 3), 'approach_mps': round(rate, 3),
                            'ttc_s': round(min(ttc, 99), 3) if ttc is not None else None,
                            'torque_direction': direction, 'intent_age_s': round(now-c['intent'], 3),
                            'confidence_age_s': round(now-self.confident[i], 3) if self.confident[i] is not None else None,
                            'width_rate_mps': round(sum(self.rates), 3), 'legacy_direction': int(legacy_direction),
                            'left_bsd': bool(getattr(cs, 'leftBlindspot', False)),
                            'right_bsd': bool(getattr(cs, 'rightBlindspot', False))}
    # Countersteer/torque release does not erase a still-approaching candidate.
    # Intent is nevertheless bounded; stationary corrections cannot linger forever.
    if now-c['intent'] > 1.5 or now-c['started'] > 4:
      self._reset(now, 'intent_expired')
      self.rearm_until = now+.5
      return
    if self.confident[i] is None or now-self.confident[i] > 1:
      self._reset(now, 'confidence_lost')
      return
    if abs(sum(self.rates)) > .35:
      self._reset(now, 'width_change')
      return
    c['minimum'] = min(c['minimum'], y[i])
    self.counts['candidate_frames'] += 1
    if y[i] <= 0:
      self._event(now, 'centre_crossed', candidate=c['id'], direction=c['direction'], predicted=c['emitted'])
      self._reset(now, 'observed_end')
      self.rearm_until = now+3.
      return
    if y[i]-c['minimum'] >= .25 and rate < -.1:
      self._event(now, 'retreated', candidate=c['id'], direction=c['direction'], predicted=c['emitted'])
      self._reset(now, 'observed_end')
      self.rearm_until = now+.5
      return
    if (not c['emitted'] and c['initial']-y[i] >= .15 and rate > .1 and
        clearance <= .5 and ttc is not None and ttc <= .6):
      c['emitted'] = True
      self._event(now, 'would_signal', **self.last_sample)
