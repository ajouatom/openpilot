"""Bind a PE voice maneuver to one existing physical-indicator hold."""
from collections import deque
from openpilot.selfdrive.controls.lib.desire_lib.assistant_lane import IDENTITY


class VoiceLaneSignal:
  def __init__(self):
    self.seen = deque(maxlen=128)
    self.saw_idle = False
    self.request = ''
    self.direction = None
    self.hold_id = None

  def select(self, meta, cs, *, allowed, peer_ready, hold, busy):
    request = getattr(meta, 'assistantLaneSignalId', '')
    direction = str(getattr(meta, 'laneChangeDirection', 'none'))
    phase = str(getattr(meta, 'laneChangeState', 'off'))
    if not request and meta is not None:
      self.saw_idle = True
    if request != self.request:
      self.request, self.hold_id, self.direction = request, None, None
      fresh = self.saw_idle and request not in self.seen and IDENTITY.fullmatch(request)
      if request:
        self.seen.append(request)
      if (fresh and allowed and peer_ready and not busy and hold is None and
          phase == 'preLaneChange' and direction in ('left', 'right') and not (cs.leftBlinker or cs.rightBlinker)):
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
    return bool(self.direction and hold and hold['source'] == 'lane' and hold['direction'] == self.direction and
                self.hold_id == hold['id'] == activated_id == feedback.get('accepted_id') and
                (cs.leftBlinker, cs.rightBlinker) == (self.direction == 'left', self.direction == 'right'))
