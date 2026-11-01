"""Observe decisions after policy evaluation; never participate in actuation."""
import json
import os

from openpilot.common.indicator_display import PATH, REASONS, clean


class Publisher:
  def __init__(self, path=PATH):
    self.path, self.next_write = path, 0.
    self.previous, self.ended, self.ended_at = None, None, 0.

  def update(self, now, *, mode, ok, cs, meta, hold, activated, peer, automatic, tbt, tbt_enabled,
             front_clear, observed_source, auto_valid, atc_type):
    # All failures are confined to optional display metadata. No commands or leases are modified.
    try:
      value = None
      if mode and hold:
        origin = hold['source']
        if origin == 'observation':
          origin = observed_source
        elif origin == 'tbt_display' or origin == 'lane' and tbt.phase in ('waiting', 'changing', 'finishing'):
          origin = 'tbt'
        reason = automatic.reason if origin == 'turn' else tbt.reason if origin == 'tbt' else 'active'
        value = {'origin': origin, 'direction': hold['direction'], 'state': 'confirmed' if activated else 'request',
                     'reason': reason if activated or peer == 'ready' else 'peer'}
      elif mode and ok and meta is not None:
        direction = str(getattr(meta, 'indicatorPrepareDirection', 'none'))
        source = str(getattr(meta, 'indicatorPrepareSource', ''))
        if direction in ('left', 'right') and source in ('manual', 'model_path', 'model_turn', 'model_desire'):
          value = {'origin': source, 'direction': direction, 'state': 'prepare', 'reason': 'boundary' if source == 'manual' else 'confirming'}
        elif str(getattr(meta, 'observedIndicator', 'none')) in ('left', 'right') and observed_source in ('manual', 'model_path', 'model_turn', 'model_desire'):
          value = {'origin': observed_source, 'direction': str(meta.observedIndicator), 'state': 'prepare',
                   'reason': 'peer' if peer != 'ready' else 'admission'}
        elif auto_valid and str(meta.laneChangeState) == 'preLaneChange' and atc_type in ('atc left', 'atc right', 'fork left', 'fork right'):
          value = {'origin': 'lane', 'direction': atc_type.split()[-1], 'state': 'prepare', 'reason': 'waiting_conditions'}
        elif tbt_enabled and tbt.phase in ('idle', 'cooldown') and tbt.reason in ('confirming', 'waiting_conditions'):
          reason = tbt.reason
          if reason == 'waiting_conditions':
            reason = 'bsd' if cs.rightBlindspot else 'front' if not front_clear else reason
          value = {'origin': 'tbt', 'direction': 'right', 'state': 'prepare', 'reason': reason}
      if value is not None:
        self.previous, self.ended = value, None
      elif self.previous is not None:
        previous, self.previous = self.previous, None
        origin = previous['origin']
        reason = automatic.reason if origin == 'turn' else tbt.reason if origin == 'tbt' else observed_source
        if not mode:
          reason = 'disabled'
        elif not ok:
          reason = 'inputs'
        elif peer == 'driver_override':
          reason = 'driver_override'
        self.ended, self.ended_at = dict(previous, state='ended', reason=reason if reason in REASONS else 'ended'), now
      if value is None and self.ended is not None and now-self.ended_at < 2:
        value = self.ended
      if now < self.next_write:
        return
      self.next_write = now+.2
      lamp = ('both' if cs.leftBlinker and cs.rightBlinker else 'left' if cs.leftBlinker else
              'right' if cs.rightBlinker else 'off') if ok else 'unknown'
      value = clean(dict(value, version=1, stamp=now, lamp=lamp), now) if value else None
      temporary = self.path.with_suffix('.tmp')
      temporary.write_text(json.dumps(value, separators=(',', ':')), encoding='utf8')
      os.replace(temporary, self.path)
    except (OSError, ValueError, TypeError, AttributeError, KeyError):
      pass


class Remote:
  def __init__(self, snapshot, clock):
    self.snapshot, self.clock = snapshot, clock
    self.key, self.stamp = None, 0.

  def read(self):
    try:
      snapshot = self.snapshot()
      if snapshot is None:
        return None
      received, record = snapshot
      now, sent = self.clock(), record['sent']
      if not 0 <= now-received < .5:
        return None
      value = clean(record.get('indicator_display'), sent)
      if value is None:
        return None
      stamp = received-(sent-value['stamp'])
      # Repeated packets never renew the source observation's lifetime.
      key = value['stamp']
      if key == self.key:
        stamp = min(stamp, self.stamp)
      self.key, self.stamp = key, stamp
      return clean(dict(value, stamp=stamp), now)
    except (KeyError, TypeError, ValueError):
      return None
