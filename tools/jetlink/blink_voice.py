"""Bind a voice maneuver to one existing physical-indicator hold."""
from collections import deque
from secrets import token_urlsafe
from openpilot.selfdrive.controls.lib.desire_lib.assistant_lane import IDENTITY


class VoiceLaneSignal:
  def __init__(self):
    self.seen = deque(maxlen=128)
    self.route = token_urlsafe(32)
    self.advertised_at = -1.
    self.saw_idle = False
    self.request = ''
    self.direction = None
    self.hold_id = None

  def advertise(self, now, fingerprint, available, save):
    if available and now - self.advertised_at >= .15:
      save({'fingerprint':fingerprint, 'route':self.route, 'time':now})
      self.advertised_at = now

  def select(self, meta, cs, *, allowed, peer_ready, hold, busy):
    request = getattr(meta, 'assistantLaneSignalId', '')
    direction = str(getattr(meta, 'laneChangeDirection', 'none'))
    phase = str(getattr(meta, 'laneChangeState', 'off'))
    if not request and meta is not None:
      self.saw_idle = True
    allowed = allowed and getattr(meta, 'assistantLaneSignalRoute', '') == self.route
    if request != self.request:
      self.request, self.hold_id, self.direction = request, None, None
      fresh = self.saw_idle and request not in self.seen and IDENTITY.fullmatch(request)
      if request:
        self.seen.append(request)
      if (fresh and allowed and peer_ready and not busy and hold is None and
          phase in ('preLaneChange', 'laneChangeStarting') and direction in ('left', 'right') and not (cs.leftBlinker or cs.rightBlinker)):
        self.direction = direction
    if (not allowed or busy or direction != self.direction or phase not in
        ('preLaneChange', 'laneChangeStarting', 'laneChangeFinishing') or
        (hold is not None and hold['id'] != self.hold_id) or (self.hold_id and not hold) or
        (cs.rightBlinker if direction == 'left' else cs.leftBlinker)):
      self.direction = None
    return ('lane', self.direction) if self.direction else None

  def bind(self, hold):
    if self.direction and self.hold_id is None:
      if hold and hold['source'] == 'lane' and hold['direction'] == self.direction:
        self.hold_id = hold['id']
        return True
      self.direction = None
    return False

  def confirmed(self, hold, activated_id, feedback, cs):
    # B1 boards sample the physical lamps even on ports without lamp-based carState.
    board_lamps = feedback.get('lamps')
    if board_lamps is not None:
      lit = (board_lamps == (1 if self.direction == 'left' else 2) and
             not (cs.rightBlinker if self.direction == 'left' else cs.leftBlinker))
    else:
      lit = (cs.leftBlinker, cs.rightBlinker) == (self.direction == 'left', self.direction == 'right')
    return bool(self.direction and hold and hold['source'] == 'lane' and hold['direction'] == self.direction and
                self.hold_id == hold['id'] == activated_id == feedback.get('accepted_id') and lit)
