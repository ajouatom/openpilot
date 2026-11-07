"""Tesla-only lamp observations. Never writes model desires or control commands."""
import math

from openpilot.selfdrive.controls.lib.desire_lib.boundary_indicator import BoundaryIndicator
from openpilot.selfdrive.controls.lib.desire_lib.indicator_observer import Movement, path_direction


class TeslaLaneSignals:
  def __init__(self, *, manual=False, desire=False, path=False):
    self.manual, self.desire, self.path = manual, desire, path
    self.dashed = [None, None]
    self.stamp = 0
    self.frame = None
    self.frame_time = 0.
    self.geometry = None
    self.stalk_counts = None
    self.dark_since = None
    self.prediction_consumed = False
    self.boundary = BoundaryIndicator(lane_admission=self.adjacent)
    self.active = None
    self.request_id = 1 << 31
    self.candidate = None
    self.reason = "idle"

  def _vision(self, vision, now):
    valid = (vision is not None and vision.lane_valid and
             0 < vision.lane_received_nanos <= now and now - vision.lane_received_nanos <= 800_000_000)
    stamp = vision.lane_received_nanos if valid else 0
    if not valid or stamp < self.stamp or stamp - self.stamp > 800_000_000:
      self.dashed = [None, None]
    for i, value in enumerate((vision.left_lane, vision.right_lane) if valid else (-1, -1)):
      if value != 0:
        self.dashed[i] = None
      elif stamp > self.stamp:
        first, count = self.dashed[i] or (stamp, 0)
        self.dashed[i] = (first, count + 1)
    self.stamp = stamp

  def adjacent(self, cs, md, direction, now):
    """Uncoloured dashed classification plus visible adjacent lane/road support."""
    if direction not in (1, 2):
      return False
    evidence = self.dashed[direction - 1]
    if evidence is None or evidence[1] < 2 or self.stamp - evidence[0] < 200_000_000:
      return False
    try:
      inner, outer, edge = (1, 0, 0) if direction == 1 else (2, 3, 1)
      sign = -1 if direction == 1 else 1
      if not all(math.isfinite(md.laneLineProbs[i]) and md.laneLineProbs[i] >= .5 for i in (inner, outer)):
        return False
      if not math.isfinite(md.roadEdgeStds[edge]) or not 0 <= md.roadEdgeStds[edge] <= 1:
        return False
      for index in (0, 4, 8):
        boundary, outside, road = (md.laneLines[inner].y[index], md.laneLines[outer].y[index], md.roadEdges[edge].y[index])
        if not all(math.isfinite(v) for v in (boundary, outside, road)):
          return False
        if not 2.5 <= sign * (outside - boundary) <= 4.8 or sign * (road - outside) < .3:
          return False
      return not (cs.leftBlindspot if direction == 1 else cs.rightBlindspot)
    except (AttributeError, IndexError, TypeError, ValueError):
      return False

  def _stop(self, reason):
    if self.active is not None:
      self.boundary.block_until = max(self.boundary.block_until, self.frame_time + 3.)
    self.active = self.candidate = None
    self.boundary._reset_geometry()
    self.boundary.request_direction = 0
    self.reason = reason
    return None

  def update(self, cs, cc, md, vision, now_ns, *, allowed, busy=False):
    now = now_ns / 1e9  # Xiaoge uses CLOCK_MONOTONIC, not CAN's BOOTTIME.
    self._vision(vision, now_ns)
    if cs.leftBlinker or cs.rightBlinker or not allowed:
      self.dark_since = None
    elif self.dark_since is None:
      self.dark_since = now
    counts = cs.leftBlinkerStalkCount, cs.rightBlinkerStalkCount
    pressed = self.stalk_counts is not None and counts != self.stalk_counts
    self.stalk_counts = counts
    if pressed:
      self.boundary.block_until = now + 3.
    common = (allowed and not busy and cs.canValid and not cs.vehicleSensorsInvalid and
              str(cs.gearShifter) == "drive" and not cs.trailerConnected and
              not cs.steerFaultTemporary and not cs.steerFaultPermanent and
              cs.turnSignalDriverValid and not cs.turnSignalDriverDirection and
              not pressed and not cs.blinkerLever and not (cs.leftBlinker and cs.rightBlinker) and
              math.isfinite(cs.vEgo) and cs.vEgo >= 30 / 3.6 and math.isfinite(cs.steeringTorque))
    if not common or not (self.manual or self.desire or self.path):
      return self._stop("context")
    if self.active and self.active.source != "manual" and not cc.latActive:
      return self._stop("lateral_inactive")
    if self.frame is not None and md.frameId == self.frame:
      if not 0 <= now - self.frame_time <= .15:
        return self._stop("model_stale")
      if self.active and self.adjacent(cs, md, self.active.direction, now):
        return self.active.direction, self.request_id
      if self.active:
        return self._stop("lane_evidence")
      return None
    if self.frame is not None and (md.frameId < self.frame or not .02 <= now - self.frame_time <= .15):
      self.frame, self.frame_time = md.frameId, now
      return self._stop("model_discontinuity")
    self.frame, self.frame_time = md.frameId, now
    if self.active:
      active = self.active
      opposite = cs.rightBlinker if active.direction == 1 else cs.leftBlinker
      enabled = self.manual if active.source == "manual" else cc.latActive and (self.desire if active.source == "model_desire" else self.path)
      if not enabled or opposite or not self.adjacent(cs, md, active.direction, now) or not active.update(now, cs, md):
        return self._stop("completed_or_cancelled")
      self.reason = active.source
      return active.direction, self.request_id
    try:
      geometry = md.laneLines[1].y[0], md.laneLines[2].y[0]
      old_geometry, self.geometry = self.geometry, geometry
      if (not all(math.isfinite(v) for v in geometry) or not geometry[0] < 0 < geometry[1] or
          not 2.5 < geometry[1] - geometry[0] < 4.8 or
          old_geometry is not None and max(abs(a-b) for a, b in zip(geometry, old_geometry, strict=True)) > .4):
        return self._stop("geometry")
    except (AttributeError, IndexError, TypeError, ValueError):
      return self._stop("geometry")
    self.boundary.update(now, cs, md, enabled=self.manual, valid=True, busy=False)
    direction, source = self.boundary.request_direction, "manual"
    if not direction and cc.latActive and not cs.steeringPressed and not (cs.leftBlinker or cs.rightBlinker) and now >= self.boundary.block_until:
      candidate = None
      probs = list(md.meta.desireState)
      if self.desire and len(probs) >= 5 and all(math.isfinite(p) and 0 <= p <= 1 for p in probs):
        i = 3 if probs[3] > probs[4] else 4
        if probs[i] >= .5 and probs[i] >= max(probs[:3]) and probs[i] - probs[7-i] >= .3:
          candidate = (1 if i == 3 else 2, "model_desire")
      if candidate is None and self.path:
        predicted = path_direction(md, cc.actuators.curvature)
        if predicted:
          candidate = predicted, "model_path"
      if candidate is None and self.dark_since is not None and now - self.dark_since >= 1.:
        self.prediction_consumed = False
      if candidate and not self.adjacent(cs, md, candidate[0], now):
        candidate = None
      old = self.candidate
      self.candidate = (candidate, now, 1) if old is None or old[0] != candidate else (candidate, old[1], old[2] + 1)
      if candidate and not self.prediction_consumed and self.candidate[2] >= 4 and now - self.candidate[1] >= .2:
        direction, source = candidate
    else:
      self.candidate = None
    if direction:
      self.active = Movement(now, direction, source, cs, md)
      self.request_id += 1
      self.prediction_consumed = True
      self.boundary.own(now, direction)
      self.reason = source
      self.candidate = None
      return direction, self.request_id
    self.reason = "observing"
    return None
