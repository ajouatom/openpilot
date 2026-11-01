"""Comma-side transmission context and bounded early lamp evidence."""
from blink_protocol import fresh


class ObservationFeedback:
  def __init__(self):
    self.ident = None
    self.tap = None
    self.value = {}
    self.seen = None

  def prepare(self, now, hold, meta, cs):
    ident = hold['id'] if hold and hold['source'] == 'observation' else None
    if ident != self.ident:
      self.ident, self.tap, self.value, self.seen = ident, None, {}, None
      if ident and not cs.blinkerLever and not cs.leftBlinker and not cs.rightBlinker:
        direction = {'left':1, 'right':2}[hold['direction']]
        counts = [getattr(cs, 'leftBlinkerStalkCount', None), getattr(cs, 'rightBlinkerStalkCount', None)]
        if all(type(x) is int and 0 <= x < 256 for x in counts):
          self.tap = {'id':ident, 'direction':direction, 'request':getattr(meta, 'observedIndicatorMonoTime', 0)/1e9,
                      'start':now, 'counts':counts}

  def sent(self, now, peer, ok):
    # Published only after successful local socket submission. Never extend the
    # initial 0.6 s window, including reconnects or a queued receiver command.
    healthy = fresh(peer.get('stamp'), now, .35) and peer.get('state') in ('ready', 'sending', 'waiting', 'waiting_previous_signal')
    self.value = {'stamp':now, 'tap':self.tap} if ok and healthy and self.tap and 0 <= now-self.tap['start'] < .6 else {}
    return self.value.get('tap')

  def activation(self, now, hold, cs, peer, acknowledged, activated_id, ok):
    if not hold or hold['source'] != 'observation':
      self.seen = None
      return activated_id
    healthy = ok and fresh(peer.get('stamp'), now, .35) and peer.get('state') in ('ready', 'sending', 'waiting', 'waiting_previous_signal')
    opposite = cs.rightBlinker if hold['direction'] == 'left' else cs.leftBlinker
    lamp = cs.leftBlinker if hold['direction'] == 'left' else cs.rightBlinker
    if not healthy or opposite or self.ident != hold['id']:
      self.seen = None
      return None
    if lamp:
      self.seen = now
    # Lamp feedback may precede the receiver's completed-send ACK. Remember it
    # briefly, but never enable the physical hold before that exact-ID ACK.
    if acknowledged and (activated_id == hold['id'] or self.seen is not None and 0 <= now-self.seen < .75):
      return hold['id']
    return None
