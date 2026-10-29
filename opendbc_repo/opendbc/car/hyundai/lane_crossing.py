# Manual lane change without the blinker: the car starting to cross a lane line, from modelV2 lane lines.
# Tuned offline on our rlogs (log_analyze/reports/auto_blinker_20261027/cross_sim.py): 10-26 2 h drive caught 35/38
# lamp-free crossings above 30 km/h, median 1.05 s before the centre crossed, 7 false; 10-24 holdout 2/2, 3 false.
NEAR_M = 0.9          # near line this close to the car centre: the tyre is on it
RATE_MPS = 0.4        # moving toward it, while the far line moves away at least as fast
SEEN_S = 3.0          # line probs fall to ~0.01 while crossing, so both lines only have to have been seen (>0.5) lately;
                      # a real lane has both (highway: 0.95-1.0 3 s before), the Azera alley's right line never passed 0.15
RELABEL_M = 0.4       # a bigger one-frame jump is the model relabelling lines after a crossing, not motion
RELABEL_HOLD_S = 2.0  # the relabelled line shows up close and "approaching" right after a crossing
COOLDOWN_S = 5.0
FROM_M = 1.2          # the near line was this far within FROM_S: a real approach from inside the lane, not a line
FROM_S = 2.0          # that just stays close (Azera 10-27 alley: left line 0.3-0.5 m for 15 s, kicked every 5 s)
# No lane beyond the near line: the road edge is right behind it (10-29 IC ramp exit, driver drifting onto the right line:
# edge 1.3 m past it). 13 drives 10-24..29 (log_analyze/reports/kick_edge_20261029/kick_sim.py): 108 kicks -> 96, drops
# 5 of 74 real crossings (IC ramp, lanes still opening) and 8 of 34 that never crossed (1 other now fires 1.3 s later).
EDGE_ROOM_M = 2.0       # edge this close past the line: never a lane there
EDGE_ROOM_UNSEEN_M = 3.0  # ...or this close while the line beyond has not been seen (>0.3) for OUTER_UNSEEN_S
OUTER_UNSEEN_S = 5.0
# Only when the driver is steering there: torque toward the crossed line within the last TORQUE_S (angle control -
# this is the driver's own push on the wheel). 13 drives: keeps 65 of 69 real crossings, drops 8 of 27 non-crossings.
TORQUE_MIN = 250  # CAN-FD steeringPressed threshold
TORQUE_S = 1.0


class LaneCrossing:
  def __init__(self):
    self.frame_id = None
    self.prev = {}
    self.hist = {1: [], 2: []}
    self.rate = {1: 0., 2: 0.}
    self.seen = {1: -99., 2: -99.}
    self.outer_seen = {1: -99., 2: -99.}  # laneLines[0] beyond the left line, [3] beyond the right
    self.hold = self.last = -99.
    self.torque = []  # (t, driver torque, + = left)

  def update(self, md, t, allowed, torque=None):
    """3 (left) or 4 (right) on the model frame a crossing starts while allowed, else 0. laneLines[1] left, [2] right.
    torque: carState.steeringTorque; given, a crossing also needs the driver pushing toward it."""
    if torque is not None:
      self.torque = [(ht, hq) for ht, hq in self.torque if t - ht < TORQUE_S] + [(t, torque)]
    if md is None or len(md.laneLines) < 3 or not len(md.laneLines[1].y) or md.frameId == self.frame_id:
      return 0
    self.frame_id = md.frameId
    y = {i: abs(md.laneLines[i].y[0]) for i in (1, 2)}
    for i in (1, 2):
      if i in self.prev:
        py, pt = self.prev[i]
        if abs(py - y[i]) > RELABEL_M:
          self.rate[i], self.hold = 0., t
        else:
          self.rate[i] = 0.5 * self.rate[i] + 0.5 * (py - y[i]) / max(t - pt, 1e-3)
      self.prev[i] = (y[i], t)
      self.hist[i] = [(ht, hy) for ht, hy in self.hist[i] if t - ht < FROM_S] + [(t, y[i])]
      if md.laneLineProbs[i] > 0.5:
        self.seen[i] = t
      if len(md.laneLineProbs) > 3 and md.laneLineProbs[3 * (i - 1)] > 0.3:
        self.outer_seen[i] = t
    if not allowed or t - self.hold < RELABEL_HOLD_S or t - self.last < COOLDOWN_S:
      return 0
    if t - min(self.seen.values()) >= SEEN_S:
      return 0
    for i in (1, 2):
      if y[i] < NEAR_M and self.rate[i] > RATE_MPS and -self.rate[3 - i] > RATE_MPS and max(hy for _, hy in self.hist[i]) >= FROM_M \
         and not self._no_lane_beyond(md, i, y[i], t) and self._pushed(i, torque is not None):
        self.last = t
        return 3 if i == 1 else 4
    return 0

  def _pushed(self, i, gated):
    sign = 1 if i == 1 else -1
    return not gated or any(sign * q >= TORQUE_MIN for _, q in self.torque)

  def _no_lane_beyond(self, md, i, y, t):
    edges = getattr(md, "roadEdges", None)
    if edges is None or len(edges) < 2 or not len(edges[i - 1].y):
      return False
    room = abs(edges[i - 1].y[0]) - y
    return room < EDGE_ROOM_M or (room < EDGE_ROOM_UNSEEN_M and t - self.outer_seen[i] >= OUTER_UNSEEN_S)
