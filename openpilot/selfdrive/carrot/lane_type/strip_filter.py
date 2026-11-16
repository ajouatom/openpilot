"""Temporal filter for StripNet type and colour results.

The StripNet path always uses it: ``marking_daemon.Recogniser(2)`` builds the asymmetric type filter with LONG_M /
SHORT_M (A8/4) plus the colour window COLOR_M and the latch (LATCH) by default, with the type confidence TYPE_CONF and the type
latch confirmation TYPE_LATCH_M (replay variant 'A8/4L' = 'A8/4L-legacy'; F1d1 'A8/4L-F1+d1' was the default on 10-26 only); the
``strip_filter`` argument (another StripFilter, or False for none) is for tests and replays.
It runs inside ``Recogniser.process`` (StripNet branch), so it sits between the model and
``display.DisplayPolicy.confirm``:
per side it takes the raw ``filter_input`` result plus the model's P(dashed) and P(yellow) and returns a result
dict of the same form. ``DisplayPolicy`` and the Scanner path are not touched.

Colour (``color_m``, None = colour passes through untouched, as in every ``make`` variant): certainty-weighted mean
P(yellow) over the last ``color_m`` metres, 0.5 split, one symmetric window (a yellow->white error is as bad as a
white->yellow one). It has its own window but shares the type filter's resets, 0.3 s gap rule and 3 s cap. Its
confidence is the winning side's weighted mean probability (mean P(yellow), or 1 - mean), held to the same 0.90
threshold as a raw colour decision (DaBin 10-19; |mean - 0.5| * 2 >= 0.90 meant mean >= 0.95). Unlearned
yellow-dashed frames do not enter it (their colour passes through as the model left it, unknown). The type result
is computed exactly as without the colour window.

Variants (``StripFilter(kind, **params)``):
  none            raw result unchanged (baseline)
  median n        majority of the last n confident type labels (unknown frames are not entered)
  distance m      certainty-weighted mean P(dashed) over the last m metres of travel, 0.5 split
  asym long/short dashed->solid decided on the short window, solid->dashed only when the long window's mean
                  P(dashed) exceeds 0.5; when the state turns solid the long window is emptied down to the short
                  one, so dashed frames still in the long window cannot turn it straight back to dashed
  latch           (``latch=True``; the StripNet default, DaBin 10-19; replay variant 'A8/4L') type and colour keep
                  their last confident label and its confidence while the window is not certain enough (no unknown
                  between confident labels); a label changes only on a confident result of the other class (window
                  mean past 0.5 and confidence >= 0.90). The colour head changes only after COLOR_LATCH_FRAMES
                  consecutive confident frames of the other colour (DaBin 10-19: a window that touches white >= 0.90
                  for one frame on a yellow line must not show white); until then the stored colour is shown, and a
                  frame that is not confident of the other colour (uncertain, or confident of the stored one) starts
                  the count again. Frames without probabilities neither count nor break it. With no stored colour
                  (after a reset, or the first one) the first confident colour is stored at once. The type head
                  (F1d1 'A8/4L-F1+d1' and the type_latch_* replay variants; the default changes on the first confident frame) changes to the other type only once its consecutive confident
                  frames span type_latch_m (1 m in d1) of travel; a frame not confident of the other type restarts that run,
                  a frame without probabilities (model abstention, lane_prob_dip, lead_close) pauses it (the travel up to
                  such a frame is not counted; codex review1026 M1), and the first type after a reset is stored at once ('A8/4L-legacy': it changed on the first confident
                  frame). Resets empty the latch and the counts too. Frames without
                  probabilities (no inference, lane_prob_dip) still pass through as they are. 'A8/4' (no latch) is
                  kept for comparison.
The other variants and the ``make`` name table are kept for the replay harness.
Type confidence: the StripNet default ``type_conf=TYPE_CONF='abs'`` is |mean - 0.5| * 2 held to 0.90 (a mean >= 0.95); 'max' (F1,
replays) gives the type window the colour window's confidence max(mean, 1 - mean). DaBin 10-26: F1d1 (max + 1 m type change
confirmation, filterab1026) was adopted in the morning and returned from the same day after the on-road drive 425 (dashed shown
solid more often; reason: rollback1026), so the default is the earlier behaviour again ('A8/4L' = 'A8/4L-legacy').
A/B switches (filterab1026; replays only): ``type_conf`` / ``type_latch_frames`` / ``type_latch_m`` as above; ``color_latch_frames`` (F2: 3) or
``color_latch_m`` (F2b: the other colour must stay confident over this many metres of travel, frames not counted); ``hold_latch``
(F3b): a model-abstained side (reason 'unknown', no P(dashed)) shows the latched type instead of unknown, so the display's type
head is refreshed by its own latch while the model abstains (the colour head likewise when no colour probability arrives).
Windows are emptied on the same reset reasons DisplayPolicy uses, on trackingReset, and on a camera stamp gap
or step back (DistanceEvidence's 0.3 s rule). A 'lane_prob_dip' frame (a short lane prob dip, see
marking_daemon.LANE_PROB_MIN) is not a reset: it comes without probabilities, so it passes through as unknown,
enters no window and keeps the camera stamp chain (no gap). Distance windows are also capped at
DistanceEvidence's 3 s, so a stopped car does not hold old frames indefinitely. The model's 0.90 head threshold is
applied to the filtered confidence (type_conf: max(mean, 1 - mean), or |mean - 0.5| * 2 when 'abs'), as StripModel.decide
applies it to the raw one.
Graded latch (``graded=True``; latchdecay1105, replays only, variants 'A8/4L-G' / '-Gs' / '-Gf'; off by default): per side and head
(type, colour) a latch strength in [0, 1] is 1 when the head (re)latches or a frame is confident (the normal 0.90 rule) of the
latched label, and otherwise decays by travel, strength *= exp(-d / L): L = g_same while the head's window mean is on the latched
label's side by >= g_band, g_neutral within g_band of 0.5 (and on frames without probabilities: lane_prob_dip, model abstention),
g_opp on the other side by >= g_band; no travel, no decay. The type head's mean is the asym window mean it decided on (P(dashed)),
the colour head's the colour window mean P(yellow). Colour: the other colour replaces the latched one once its window confidence
reaches g_cmin + (0.90 - g_cmin) * strength on g_nmin + (COLOR_LATCH_FRAMES - g_nmin) * strength (rounded half up) consecutive
frames (at strength 1 exactly the default rule). Type: the change rule is unchanged; the latch expires when its strength is below
g_expire and the window mean is on the other side: the side shows type unknown and the type latch is emptied (so a 0.4-0.55 window
cannot bring the stale type back; the next confident type latches at once, as after a reset). Colour expiry (strength below
g_expire, window on the other side, not yet switchable) shows colour unknown but keeps the colour latch, so the relaxed switch can
still happen. ``g_type_expire=False`` (variant 'A8/4L-GsC' = Gs colour only; latchcombo1105): the graded rule is applied to the
colour head only; the type head uses the plain latch (no type decay, no type expiry).
"""
from collections import deque
import math

from openpilot.selfdrive.carrot.lane_type.temporal import WINDOW_NS

RESET_REASONS = ('lane_change', 'tracking_lost', 'tracking_lost_or_stale', 'invalid_lane_geometry', 'not_detected')
MAX_GAP_NS = 300_000_000
THRESHOLD = 0.90
LONG_M, SHORT_M = 8., 4.  # the StripNet path's default asymmetric filter (A8/4)
COLOR_M = 8.  # the StripNet path's default colour window
LATCH = True  # the StripNet path keeps the last confident type / colour (A8/4L)
COLOR_LATCH_FRAMES = 10  # consecutive confident frames of the other colour before the latched colour changes (0.5 s at 20 fps)
# DaBin 10-26: F1d1 (max(mean, 1 - mean) type confidence, TYPE_LATCH_M = 1 m change confirmation) adopted, then returned the same
# day after the on-road check (drive 425, rollback1026): |mean - 0.5| * 2 and no confirmation again. F1d1 stays as 'A8/4L-F1+d1'.
TYPE_CONF = 'abs'
TYPE_LATCH_M = None
GRADED = dict(g_same=60., g_neutral=30., g_opp=12., g_band=.1, g_cmin=.65, g_nmin=3, g_expire=.2)  # latchdecay1105 base set (replays only)


def p_dashed(model_result):
  """P(dashed) from StripModel.decide's output (two-class softmax), or None without a model output."""
  if not isinstance(model_result, dict) or model_result.get('typeCandidate') not in ('dashed', 'solid'):
    return None
  c = float(model_result['typeConfidence'])
  return c if model_result['typeCandidate'] == 'dashed' else 1. - c


def p_yellow(model_result):
  """P(yellow) from StripModel.decide's output (two-class softmax), or None without a model output."""
  if not isinstance(model_result, dict) or model_result.get('colorCandidate') not in ('white', 'yellow'):
    return None
  c = float(model_result['colorConfidence'])
  return c if model_result['colorCandidate'] == 'yellow' else 1. - c


class _Side:
  def __init__(self):
    self.items = deque()  # (stamp, odometer, label_or_p, weight)
    self.state = None     # asym: last decided type
    self.colors = deque()  # (stamp, odometer, P(yellow), weight)
    self.latched = {}  # latch: head -> (label, confidence)
    self.flips = 0  # latch: consecutive confident frames of a colour other than the latched one
    self.flip_odometer = None  # color_latch_m: odometer of the first frame of that run
    self.tflips, self.tflip_odometer, self.odometer = 0, None, None  # type_latch_frames / type_latch_m (A/B only)
    self.last_stamp = None
    self.strength, self.g_mean, self.g_odometer, self.g_dist = {}, {}, None, 0.  # graded latch (replays only)

  def clear(self):
    self.items.clear()
    self.state = None
    self.colors.clear()
    self.latched.clear()
    self.flips = 0
    self.tflips = 0
    self.strength.clear()
    self.g_mean.clear()


class StripFilter:
  def __init__(self, kind='none', n=5, m=12., long_m=12., short_m=4., threshold=THRESHOLD, color_m=None, latch=False,
               type_conf='abs', color_latch_frames=None, color_latch_m=None, hold_latch=False, type_latch_frames=None, type_latch_m=None,
               graded=False, g_same=60., g_neutral=30., g_opp=12., g_band=.1, g_cmin=.65, g_nmin=3, g_expire=.2, g_type_expire=True):
    if kind not in ('none', 'median', 'distance', 'asym') or type_conf not in ('abs', 'max'):
      raise ValueError(kind)
    self.kind, self.n, self.m, self.long_m, self.short_m, self.threshold = kind, n, m, long_m, short_m, threshold
    self.color_m, self.latch = color_m, latch
    self.color_latch_frames = COLOR_LATCH_FRAMES if color_latch_frames is None else color_latch_frames  # replays set 1 (7c5ba9cb)
    self.type_conf, self.color_latch_m, self.hold_latch = type_conf, color_latch_m, hold_latch
    self.type_latch_frames, self.type_latch_m = type_latch_frames, type_latch_m  # A/B F1h/F1d: None = change on the first frame
    if graded and not latch:
      raise ValueError('graded needs latch')
    self.graded, self.g_same, self.g_neutral, self.g_opp, self.g_band = graded, g_same, g_neutral, g_opp, g_band
    self.g_cmin, self.g_nmin, self.g_expire, self.g_type_expire = g_cmin, g_nmin, g_expire, g_type_expire
    self.sides = {}

  def reset(self):
    self.sides.clear()

  def _latch(self, s, head, label, confidence, confident, odometer=None):
    """latch: a confident label is stored; otherwise the stored one is returned (label, confidence, confident).
    The colour head stores a different colour only on its color_latch_frames-th consecutive confident frame
    (color_latch_m: once the run of consecutive confident frames of the other colour spans that many metres)."""
    if not self.latch:
      return label, confidence, confident
    if self.graded and (head == 'color' or self.g_type_expire):  # g_type_expire False: the type head keeps the plain latch
      return self._latch_graded(s, head, label, confidence, confident)
    if head == 'color':
      if confident and head in s.latched and s.latched[head][0] != label:
        s.flips += 1
        if self.color_latch_m is None:
          if s.flips < self.color_latch_frames:
            return (*s.latched[head], True)
        else:
          if s.flips == 1:
            s.flip_odometer = odometer
          if odometer - s.flip_odometer < self.color_latch_m:
            return (*s.latched[head], True)
      s.flips = 0
    elif head == 'type' and (self.type_latch_frames is not None or self.type_latch_m is not None):
      # A/B F1h/F1d: a different type is stored only on the type_latch_frames-th consecutive confident frame of it (or once that run
      # spans type_latch_m metres); a frame not confident of the other type restarts the count; the first latch is immediate.
      if confident and head in s.latched and s.latched[head][0] != label:
        s.tflips += 1
        if s.tflips == 1:
          s.tflip_odometer = s.odometer
        if (s.tflips < self.type_latch_frames if self.type_latch_m is None else s.odometer - s.tflip_odometer < self.type_latch_m):
          return (*s.latched[head], True)
      s.tflips = 0
    if confident:
      s.latched[head] = (label, confidence)
    elif head in s.latched:
      return (*s.latched[head], True)
    return label, confidence, confident

  def _decay(self, s, head, m):
    """graded: decay the head's latch strength by this frame's travel; -> 1 / 0 / -1 = window mean m on the latched label's side,
    neutral (or m None: no probabilities), on the other side; None without a latch."""
    if head not in s.latched:
      return None
    signed = 0. if m is None else (m - .5 if s.latched[head][0] in ('dashed', 'yellow') else .5 - m)
    side = 1 if signed >= self.g_band else -1 if signed <= -self.g_band else 0
    if s.g_dist > 0:
      s.strength[head] = s.strength.get(head, 1.)*math.exp(-s.g_dist/(self.g_neutral, self.g_same, self.g_opp)[side])
    return side

  def _latch_graded(self, s, head, label, confidence, confident):
    """graded latch (see the module docstring); the type head changes as the default latch does."""
    side = self._decay(s, head, s.g_mean.get(head))
    strength = s.strength.get(head, 1.)
    if head == 'color':
      if head in s.latched and label in ('white', 'yellow') and label != s.latched[head][0]:
        if confidence >= self.g_cmin + (self.threshold - self.g_cmin)*strength:
          s.flips += 1
          if s.flips >= int(self.g_nmin + (self.color_latch_frames - self.g_nmin)*strength + .5):
            s.flips = 0
            s.latched[head], s.strength[head] = (label, confidence), 1.
            return label, confidence, True
        else:
          s.flips = 0
        if side == -1 and strength < self.g_expire:
          return 'unknown', 0., False  # expired: the latch is kept for the relaxed switch
        return (*s.latched[head], True)
      s.flips = 0
    if confident:
      s.latched[head], s.strength[head] = (label, confidence), 1.
    elif head in s.latched:
      if head == 'type' and side == -1 and strength < self.g_expire:
        del s.latched[head]
        s.strength.pop(head, None)
        return 'unknown', 0., False
      return (*s.latched[head], True)
    return label, confidence, confident

  def _typed(self, result, kind, confidence, stamp, s=None):
    out = dict(result)
    confident = kind in ('dashed', 'solid') and confidence >= self.threshold
    if s is not None:
      kind, confidence, confident = self._latch(s, 'type', kind, confidence, confident)
    out['type'] = kind if confident else 'unknown'
    out['typeConfidence'] = float(confidence) if confident else 0.
    out['typeMonoTimeNanos'] = stamp
    out['dashedPatternObserved'] = out['type'] == 'dashed'
    out['dashedPatternMonoTimeNanos'] = stamp
    return out

  def update(self, side, result, p, stamp, odometer, py=None):
    """result: filter_input dict for this side; p / py: P(dashed) / P(yellow) of the model output used for it
    (None if no inference ran, or the frame must not enter the windows); stamp: camera EOF ns; odometer: travel m."""
    if self.kind == 'none' and self.color_m is None:
      return result
    s = self.sides.setdefault(side, _Side())
    if (result.get('trackingReset') or result.get('reason') in RESET_REASONS or
        (s.last_stamp is not None and not 0 < stamp - s.last_stamp <= MAX_GAP_NS)):
      s.clear()
    s.last_stamp = stamp
    if p is None and s.tflips and s.odometer is not None:
      s.tflip_odometer += odometer - s.odometer  # paused: travel to a frame without P(dashed) does not count toward type_latch_m
    s.odometer = odometer
    if self.graded:
      s.g_dist = 0. if s.g_odometer is None else max(0., odometer - s.g_odometer)
      s.g_odometer = odometer
    if result.get('reason') in RESET_REASONS:
      return result
    out = result if self.kind == 'none' else self._type(s, result, p, stamp, odometer)
    if self.graded and p is None:
      self._decay(s, 'type', None)
    if self.hold_latch and p is None and result.get('reason') == 'unknown' and 'type' in s.latched:
      out = self._typed(out, 'unknown', 0., stamp, s)  # F3b: the abstaining model's side shows the latched type
    out = out if self.color_m is None else self._color(s, out, py, stamp, odometer)
    if self.graded and py is None:
      self._decay(s, 'color', None)
    if self.hold_latch and py is None and result.get('reason') == 'unknown' and 'color' in s.latched:
      out = dict(out, color=s.latched['color'][0], colorConfidence=float(s.latched['color'][1]), colorMonoTimeNanos=stamp)
    return out

  def _color(self, s, result, py, stamp, odometer):
    if py is not None:
      s.colors.append((stamp, odometer, py, abs(py - .5)*2))
    while s.colors and (odometer - s.colors[0][1] > self.color_m or stamp - s.colors[0][0] > WINDOW_NS):
      s.colors.popleft()
    if py is None:
      return result
    total = sum(w for _, _, _, w in s.colors)
    m = sum(q*w for _, _, q, w in s.colors)/total if total > 0 else .5
    confidence = max(m, 1. - m)  # the winning side's mean probability, as a raw decision's confidence
    color = 'yellow' if m > .5 else 'white' if m < .5 else 'unknown'
    s.g_mean['color'] = m
    confident = color != 'unknown' and confidence >= self.threshold
    color, confidence, confident = self._latch(s, 'color', color, confidence, confident, odometer)
    out = dict(result)
    out['color'] = color if confident else 'unknown'
    out['colorConfidence'] = float(confidence) if confident else 0.
    out['colorMonoTimeNanos'] = stamp
    return out

  def _type(self, s, result, p, stamp, odometer):
    if self.kind == 'median':
      if result.get('type') in ('dashed', 'solid'):
        s.items.append((stamp, odometer, result['type'], float(result['typeConfidence'])))
        while len(s.items) > self.n:
          s.items.popleft()
      if p is None or not s.items:
        return result if p is None else self._typed(result, 'unknown', 0., stamp, s)
      votes = {k: [w for _, _, v, w in s.items if v == k] for k in ('dashed', 'solid')}
      kind = max(votes, key=lambda k: len(votes[k]))
      if 2*len(votes[kind]) <= len(s.items):
        return self._typed(result, 'unknown', 0., stamp, s)
      return self._typed(result, kind, sum(votes[kind])/len(votes[kind]), stamp, s)
    if p is not None:
      s.items.append((stamp, odometer, p, abs(p - .5)*2))
    span = self.m if self.kind == 'distance' else max(self.long_m, self.short_m)
    while s.items and (odometer - s.items[0][1] > span or stamp - s.items[0][0] > WINDOW_NS):
      s.items.popleft()
    if p is None:
      return result

    def mean(metres):
      xs = [(q, w) for _, o, q, w in s.items if odometer - o <= metres]
      total = sum(w for _, w in xs)
      return sum(q*w for q, w in xs)/total if total > 0 else .5
    if self.kind == 'distance':
      m = mean(self.m)
      s.g_mean['type'] = m
      return self._typed(result, 'dashed' if m > .5 else 'solid' if m < .5 else 'unknown', self._conf(m), stamp, s)
    short, long = mean(self.short_m), mean(self.long_m)
    if s.state == 'dashed':
      kind, m = ('solid', short) if short < .5 else ('dashed', short)
    else:
      if long > .5:
        kind, m = 'dashed', long
      elif short < .5:
        kind, m = 'solid', short
      elif s.state == 'solid':
        kind, m = 'solid', long
      else:
        kind, m = None, .5
    if kind == 'solid' and s.state != 'solid':
      # empty the long window (keep the short one): the dashed frames that made it dashed must not reverse it
      while s.items and odometer - s.items[0][1] > self.short_m:
        s.items.popleft()
    s.state = kind
    s.g_mean['type'] = m
    return self._typed(result, kind or 'unknown', self._conf(m), stamp, s)

  def _conf(self, m):
    """type window confidence: |m - 0.5| * 2 (default; 0.90 needs a mean >= 0.95), or max(m, 1 - m) as the colour window (F1)."""
    return max(m, 1. - m) if self.type_conf == 'max' else abs(m - .5)*2


VARIANTS = {
  'none': {'kind': 'none'},
  'M3': {'kind': 'median', 'n': 3}, 'M5': {'kind': 'median', 'n': 5}, 'M7': {'kind': 'median', 'n': 7},
  'D8': {'kind': 'distance', 'm': 8.}, 'D12': {'kind': 'distance', 'm': 12.}, 'D15': {'kind': 'distance', 'm': 15.},
  'A12/4': {'kind': 'asym', 'long_m': 12., 'short_m': 4.}, 'A15/4': {'kind': 'asym', 'long_m': 15., 'short_m': 4.},
  'A8/4': {'kind': 'asym', 'long_m': LONG_M, 'short_m': SHORT_M},  # no latch, type only: comparison
  'A8/4L': {'kind': 'asym', 'long_m': LONG_M, 'short_m': SHORT_M, 'color_m': COLOR_M, 'latch': LATCH, 'type_conf': TYPE_CONF,
            'type_latch_m': TYPE_LATCH_M},  # the StripNet default (= 'A8/4L-legacy'; F1d1 on 10-26 only, then returned)
  'A8/4L-legacy': {'kind': 'asym', 'long_m': LONG_M, 'short_m': SHORT_M, 'color_m': COLOR_M, 'latch': LATCH},  # = the default
}
# filterab1026 A/B variants of A8/4L-legacy (replays only; 'A8/4L-F1+d1' was the default on 10-26 only): F1 type confidence max(m, 1-m), F2 colour latch 3 frames, F2b colour latch
# 2 m, F3b latched heads shown while the model abstains, F5 colour window 4 m. F3 / F4 are strip_model / marking_daemon switches.
_AB = {'F1': {'type_conf': 'max'}, 'F2': {'color_latch_frames': 3}, 'F2b': {'color_latch_m': 2.}, 'F3b': {'hold_latch': True},
       'F5': {'color_m': 4.}, 'h2': {'type_latch_frames': 2}, 'h3': {'type_latch_frames': 3}, 'd1': {'type_latch_m': 1.}}
for _names in (('F1', 'h2'), ('F1', 'h3'), ('F1', 'd1'), ('F1',), ('F2',), ('F2b',), ('F3b',), ('F5',), ('F1', 'F2'), ('F1', 'F2', 'F5'), ('F1', 'F2', 'F3b'), ('F1', 'F2', 'F3b', 'F5')):
  VARIANTS['A8/4L-' + '+'.join(_names)] = dict(VARIANTS['A8/4L-legacy'], **{k: v for n in _names for k, v in _AB[n].items()})
# latchdecay1105 graded latch (replays only): base decay lengths, x2 slower (Gs), x0.5 faster (Gf); thresholds unchanged
for _name, _k in (('G', 1.), ('Gs', 2.), ('Gf', .5)):
  VARIANTS['A8/4L-' + _name] = dict(VARIANTS['A8/4L'], graded=True, **dict(GRADED, **{g: GRADED[g]*_k for g in ('g_same', 'g_neutral', 'g_opp')}))
VARIANTS['A8/4L-GsC'] = dict(VARIANTS['A8/4L-Gs'], g_type_expire=False)  # latchcombo1105: Gs colour relaxation / expiry only


def make(name):
  return StripFilter(**VARIANTS[name])
