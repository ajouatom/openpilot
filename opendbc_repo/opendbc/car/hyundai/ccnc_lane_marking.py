"""CCNC lane line colour/shape from the display-only lane marking recogniser (Scanner/StripNet).

Applied per side to the existing lane code only where the existing policy shows a normal line (white 2 or
lane-change-available green 6). Priority: recognised yellow -> orange solid 4 (also over green);
otherwise the existing green condition keeps green with the recognised shape (solid 6 / dashed 10);
otherwise white with the recognised shape (solid 2 / dashed 8). UNKNOWN, expired or disabled leaves the
existing code untouched (SideDisplay's lapsed grey and start-up hide aside), as do grey 0 (low confidence), hidden lines,
departure blinking and the existing warning orange, which are decided outside this function. Display only: lane-change permission is not read or changed.

``marking`` is card's latest parsed packet: {'left': {'type': (value, expires_ns)|None, 'color': ...}, ...}.
"""
WHITE_SOLID, ORANGE_SOLID, GREEN_SOLID, WHITE_DASHED, GREEN_DASHED = 2, 4, 6, 8, 10
HOLD_S, GREEN_ON_S, STOP_SPEED, HOLD_STOP_SPEED = 5., 1., .5, 2.
LOW_PROB_OFF, LOW_PROB_S, MAX_DT = .5, 1.5, .2
HIDDEN = 14
# rollback1026 H (replay switch only, off in the product; not adopted 10-26: grey +47 s on drive 425, visible<->hidden +45 %; None = off): while changing lanes, once the lane prob has stayed < LOW_PROB_OFF for LC_HOLD_LIMIT_S
# (1.5 s in the A/B) the unlimited lane-change hold of each head ends (lapsed grey) until that head is confirmed fresh again or the
# change ends. The slow / stopped hold (v < HOLD_STOP_SPEED) and the driving HOLD_S are unchanged.
LC_HOLD_LIMIT_S = None


def _head(marking, side, head, now_ns):
  entry = (marking.get(side) or {}).get(head) if marking else None
  return entry[0] if entry is not None and now_ns <= entry[1] else None


def heads(marking, side, now_ns):
  """Fresh (type, colour) of one side; None for a missing or expired head."""
  return _head(marking, side, 'type', now_ns), _head(marking, side, 'color', now_ns)


def code(existing, kind, color):
  if existing not in (WHITE_SOLID, GREEN_SOLID):
    return existing
  if color == 'yellow':
    return ORANGE_SOLID
  if color != 'white' or kind not in ('dashed', 'solid'):
    return existing
  if existing == GREEN_SOLID:
    return GREEN_DASHED if kind == 'dashed' else GREEN_SOLID
  return WHITE_DASHED if kind == 'dashed' else WHITE_SOLID


def lane_code(existing, marking, side, now_ns):
  return code(existing, *heads(marking, side, now_ns)) if marking else existing


class SideDisplay:
  """Display memory of one side (display only; no control input is read or written).

  - hold: the last fresh type and the last fresh colour (each head separately) fill a missing head for HOLD_S while
    driving and without a time limit while stopped or creeping (v < HOLD_STOP_SPEED, 2 m/s: slow queues) or changing
    lanes (blinker, desire, laneChangeState: the last confirmed line stays through the change; the HOLD_S clock
    restarts when it ends);
    a new fresh value replaces it at once.
  - lapsed grey (``enabled``, the recogniser option on): once a head's hold has run out while driving, the line is
    shown grey 0 instead of the existing solid-looking base until the recogniser confirms that head again. Only a
    head the code needs counts: the colour always, the type only for a white line (yellow is orange solid either
    way). While changing lanes a head whose hold already ran out shows its last confirmed value again (DaBin, 10-20).
  - unseen (``enabled``): no colour confirmed since start (card start) -> hidden 14 instead of the solid-looking base
    (DaBin, 10-20: no white solid before an orange line is recognised). No timeout: a recogniser that never confirms
    keeps the line hidden. ``enabled`` False leaves the existing base.
  - green (lane change available) turns on only after GREEN_ON_S of continuous availability, off at once.
  - low: lane prob < LOW_PROB_OFF for LOW_PROB_S (1.5 s) of continuous driving -> grey 0; released at >= LOW_PROB_OFF;
    frozen while stopped (v < STOP_SPEED, 0.5 m/s). Was < 0.3 for 1 s: in intersections the prob wanders 0.1-0.45 and
    rarely stayed below 0.3 for 1 s, so the last line stayed on through them (10-20 review, item 3).
    Not while changing lanes once a colour has been confirmed (no speed condition, DaBin 10-20): low is off and does not
    accumulate, so the last confirmed line stays through the change, also when grey was already on as it started.
    Before any confirmation low applies as before. There is no creeping-distance hide here: ccnc_parking.ParkingLaneHide
    already hides lines in a car park.
  """
  def __init__(self):
    self.hold = {'type': None, 'color': None}  # head -> (value, last fresh time)
    self.lapsed = {'type': False, 'color': False}  # head -> hold ran out, no fresh value since
    self.last = {'type': None, 'color': None}  # head -> last confirmed value, kept for the card's lifetime
    self.green, self.green_s, self.low, self.low_s, self.last_t = None, 0., False, 0., None
    self.lc_low_s, self.lc_expired = 0., {'type': False, 'color': False}  # LC_HOLD_LIMIT_S (A/B H only)
    self.display_reset_ns = 0

  def update(self, t, v_ego, lane_change_available, prob, changing, kind, color, enabled=True, display_reset_ns=None):
    """Returns (green, low, kind, colour, lapsed, unseen) for this frame; lapsed True -> show grey 0, unseen True ->
    hidden 14."""
    if (enabled and isinstance(display_reset_ns, int) and not isinstance(display_reset_ns, bool)
        and display_reset_ns > self.display_reset_ns):
      # Optional recogniser lifecycle reset: once per generation, only the recognised paint memory.
      self.display_reset_ns = display_reset_ns
      for head in ('type', 'color'):
        self.hold[head], self.last[head], self.lapsed[head] = None, None, False
    dt = 0. if self.last_t is None else min(max(t - self.last_t, 0.), MAX_DT)
    self.last_t = t
    stopped = v_ego < STOP_SPEED
    slow = v_ego < HOLD_STOP_SPEED  # hold without a time limit

    available = bool(lane_change_available)
    self.green_s = self.green_s + dt if available else 0.
    if self.green is None or not available or self.green_s >= GREEN_ON_S:
      self.green = available

    if changing and self.last['color'] is not None:
      self.low, self.low_s = False, 0.
    elif prob is not None and not stopped:
      self.low_s = self.low_s + dt if prob < LOW_PROB_OFF else 0.
      if self.low_s >= LOW_PROB_S:
        self.low = True
      elif prob >= LOW_PROB_OFF:
        self.low = False

    limit = LC_HOLD_LIMIT_S
    if limit is not None and changing and not slow:
      self.lc_low_s = self.lc_low_s + dt if prob is not None and prob < LOW_PROB_OFF else 0.
      if self.lc_low_s >= limit:
        self.lc_expired = {'type': True, 'color': True}
    else:
      self.lc_low_s, self.lc_expired = 0., {'type': False, 'color': False}
    shown = {}
    for head, value, classes in (('type', kind, ('dashed', 'solid')), ('color', color, ('white', 'yellow'))):
      held = self.hold[head]
      if value in classes:
        held = (value, t)
        self.lapsed[head] = False
        self.last[head] = value
        self.lc_expired[head] = False
      elif self.lc_expired[head]:  # A/B H: the lane-change hold ran out (lane prob low for LC_HOLD_LIMIT_S) -> lapsed grey
        held, value = None, None
        self.lapsed[head] = True
      elif held is not None and (slow or changing or t - held[1] <= HOLD_S):
        value = held[0]
        if slow or changing:
          held = (value, t)
      elif changing and enabled and self.last[head] is not None:
        value = self.last[head]
        held = (value, t)
        self.lapsed[head] = False
      elif held is not None:
        held = None
        self.lapsed[head] = True
      self.hold[head], shown[head] = held, value
    lapsed = bool(enabled) and (
      (shown['color'] not in ('white', 'yellow') and self.lapsed['color']) or
      (shown['color'] == 'white' and shown['type'] not in ('dashed', 'solid') and self.lapsed['type']))
    unseen = bool(enabled) and self.last['color'] is None
    return self.green, self.low, shown['type'], shown['color'], lapsed, unseen
