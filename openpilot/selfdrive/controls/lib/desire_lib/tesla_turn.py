import math


def turn_signal_context(sm, state, now, boot_ns, always_lateral):
  def fresh(service):
    return (sm.seen[service] and sm.valid[service] and sm.alive[service] and
            0 <= now - sm.recv_time[service] <= .35)

  pose = sm['livePose']
  stamp = sm.logMonoTime['livePose']
  heading = None
  if (fresh('livePose') and 0 < stamp <= boot_ns and boot_ns - stamp <= 350_000_000 and
      pose.inputsOK and pose.sensorsOK and pose.posenetOK and pose.orientationNED.valid and math.isfinite(pose.orientationNED.z)):
    heading = (stamp, pose.orientationNED.z)
  allowed = (heading is not None and fresh('selfdriveState') and fresh('carControl') and
             state.latEnabled and (sm['selfdriveState'].active or always_lateral) and
             not state.steerFaultTemporary and not state.steerFaultPermanent)
  return allowed, heading


class TeslaTurnJourney:
  """Lamp-only lifetime; initiation still belongs to the existing turn desire."""

  def __init__(self):
    self.direction = 0
    self.started = 0.
    self.peak_angle = self.rotation = 0.
    self.entered = False
    self.pose = None
    self.nav = ""
    self.distance = None
    self.pending = None
    self.consumed = False

  def _geometry(self, state, pose):
    previous, self.pose = self.pose, pose
    if pose is None or not all(math.isfinite(v) for v in (state.vEgo, state.steeringAngleDeg, pose[1])):
      self.pose = None
      self.peak_angle = self.rotation = 0.
      self.entered = False
      return False
    if previous is None:
      return False
    if pose[0] == previous[0]:
      return False
    delta = (pose[1] - previous[1] + math.pi) % (2 * math.pi) - math.pi
    if not 0 < pose[0] - previous[0] <= 350_000_000 or abs(delta) > .2:
      self.peak_angle = self.rotation = 0.
      self.entered = False
      return False
    if state.vEgo < 1 / 3.6:
      return False
    sign = 1 if self.direction == 1 else -1
    self.peak_angle = max(self.peak_angle, state.steeringAngleDeg * sign)
    self.rotation = max(0., self.rotation - delta * sign)
    self.entered |= self.peak_angle >= 30 and self.rotation >= math.radians(20)
    return self.entered and self.peak_angle > 30 and abs(state.steeringAngleDeg) <= 15

  def update(self, now, state, pose, nav, distance, desire, permitted, lateral_active):
    nav_direction = 1 if nav == "turn left" else 2 if nav == "turn right" else 0
    distance = distance if isinstance(distance, (int, float)) and math.isfinite(distance) and 0 <= distance <= 250 else None
    nav_changed = nav != self.nav
    new_cue = nav_changed or (distance is not None and self.distance is not None and distance > self.distance + 10)
    self.nav, self.distance = nav, distance
    if not self.direction and nav_changed:
      self.consumed = False
    if self.direction and new_cue and nav_direction:
      self.pending = (nav_direction, now)
    if not permitted:
      self.consumed |= bool(self.direction)
      self.direction = 0
      self.pending = None
      return 0
    if self.direction:
      complete = self._geometry(state, pose)
      sign = 1 if self.direction == 1 else -1
      driver_abort = (self.peak_angle >= 30 and state.vEgo >= 1 / 3.6 and state.steeringPressed and
                      state.steeringTorque * sign < 0 and state.steeringAngleDeg * sign < 0)
      nav_exit = not nav_direction and nav not in ("", "none", "none prepare", "straight", "straight prepare",
                                                  "turn left prepare", "turn right prepare")
      if driver_abort or nav_exit or not 0 <= now - self.started < 60:
        self.direction = 0
        self.pending = None
        self.consumed = True
      elif complete:
        self.direction = 0
        self.consumed = True
        self.peak_angle = self.rotation = 0.
        self.entered = False
        if self.pending and 0 <= now - self.pending[1] <= 3 and desire == self.pending[0] and lateral_active:
          # A same-side handoff retains the outer request's original 60 s bound.
          self.direction = self.pending[0]
          self.pose = pose
        self.pending = None
      elif not nav_direction and not self.entered:
        self.direction = 0
        self.consumed = True
    elif not self.consumed and lateral_active and desire and desire == nav_direction:
      self.direction = desire
      self.started = now
      self.peak_angle = self.rotation = 0.
      self.entered = False
      self.pose = pose
      self.pending = None
    return self.direction
