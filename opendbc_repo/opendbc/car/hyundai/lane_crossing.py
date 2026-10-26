# Manual lane change without the blinker: the car starting to cross a lane line, from modelV2 lane lines.
# Tuned offline on our rlogs (log_analyze/reports/auto_blinker_20261027/cross_sim.py): 10-26 2 h drive caught 35/38
# lamp-free crossings above 30 km/h, median 1.05 s before the centre crossed, 7 false; 10-24 holdout 2/2, 3 false.
NEAR_M = 0.9          # near line this close to the car centre: the tyre is on it
RATE_MPS = 0.4        # moving toward it, while the far line moves away at least as fast
SEEN_S = 3.0          # line probs fall to ~0.01 while crossing, so the line only has to have been seen (>0.5) lately
RELABEL_M = 0.4       # a bigger one-frame jump is the model relabelling lines after a crossing, not motion
RELABEL_HOLD_S = 2.0  # the relabelled line shows up close and "approaching" right after a crossing
COOLDOWN_S = 5.0


class LaneCrossing:
  def __init__(self):
    self.frame_id = None
    self.prev = {}
    self.rate = {1: 0., 2: 0.}
    self.seen = {1: -99., 2: -99.}
    self.hold = self.last = -99.

  def update(self, md, t, allowed):
    """3 (left) or 4 (right) on the model frame a crossing starts while allowed, else 0. laneLines[1] left, [2] right."""
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
      if md.laneLineProbs[i] > 0.5:
        self.seen[i] = t
    if not allowed or t - self.hold < RELABEL_HOLD_S or t - self.last < COOLDOWN_S:
      return 0
    for i in (1, 2):
      if t - self.seen[i] < SEEN_S and y[i] < NEAR_M and self.rate[i] > RATE_MPS and -self.rate[3 - i] > RATE_MPS:
        self.last = t
        return 3 if i == 1 else 4
    return 0
