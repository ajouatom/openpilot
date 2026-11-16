"""CCNC lane marking recogniser process (display only): StripNet with the lane_strip_v4 model.

The model (strip_model.load: assets/lane_strip_v4.*, the only one; DaBin 10-26 removed the model choice setting) is loaded at
start. When it is missing or invalid a warning is logged and nothing is published, so the existing lane display stays; the
process waits until CcncLaneTypeDetection is turned off. CcncLaneTypeDetection is polled every PARAM_CHECK_S. The active model
(manifest version, sha256 prefix) is logged and carried in every packet's 'model' entry. The profile Scanner (Recogniser mode 1)
is kept for offline replays only and never runs here.

Runs outside modeld/controlsd/card. Every new full-resolution road camera NV12 frame goes into a small buffer
of the newest FRAME_BUFFER frames (older ones are dropped; buffers are camerad's shared memory, referenced not
copied). For each new modelV2 the frame the model actually used (same frameId and EOF) is taken first; if it
is not buffered, the frame whose EOF is nearest the model's timestampEof (the model's input frame time, not its
receive time) within MAX_FRAME_OFFSET_NS, with the model's lane geometry shifted by the travel in between. The selected recogniser's
per-frame result goes through the existing display filter (display.DisplayPolicy: two identical results
within 0.4 s, 0.5 s expiry); the confirmed left/right type/colour, their age and timing are published on
customReservedRawData2 for card to read without waiting. No CAN, steering, longitudinal or lane-change
decision is computed here.
"""
import json
import math
import os
import time

from openpilot.selfdrive.carrot.lane_type.marking import MODES, PACKET_TYPE, PACKET_VERSION, mode_of

PARAM_CHECK_S = 1.
# Recorded timing (190 segments): modelV2 arrives 37-53 ms (p5-p99) after its frame, so with 50 ms frames the
# newest frame is sometimes already the next one. 3 frames cover that with margin; camerad recycles its 18
# VisionIPC buffers only after ~0.9 s, so referencing 3 recent buffers during processing is safe.
FRAME_BUFFER = 3
MAX_FRAME_OFFSET_NS = 50_000_000  # one frame period; a two-frame (100 ms) offset is never accepted
# A held VisionIPC buffer is camerad's shared slot, not a copy: the ISP rewrites the slot of frame r while reading out
# frame r+18, before camerad republishes it, so the oldest buffered frame has ~0.7 s from recv() until its pixels can
# change (.analysis/lane-marking-vipc-buffer-20261019). Frames held this long are not processed or published.
MAX_HOLD_NS = 500_000_000
# StripNet: a lane line prob below LANE_PROB_MIN is not inferred. A short dip (LANE_PROB_LOST <= prob, for at most
# LANE_DIP_MAX_NS since the side first fell below LANE_PROB_MIN) is 'lane_prob_dip': the frame is skipped (unknown, not
# entered in the strip filter's windows) and neither those windows nor DisplayPolicy are emptied. Below LANE_PROB_LOST,
# or a dip lasting longer, is 'not_detected', which empties both.
# DaBin 10-26 gate G1 (the product default GATE_RELAX = 'G1'): the floors are half the earlier .5 / .3 (LANE_PROB_MIN_OLD /
# LANE_PROB_LOST_OLD), so a fading line (yellow solid widening into a white dashed one, 425 bookmark 21/6.3) keeps being inferred
# when its strip geometry is valid. rollback1026 follow-up 3 (D1 s2): 425 bookmark windows misclassified -8 s, five sets not
# shown -7 s, new errors +10 s, style changes +6 %.
LANE_PROB_MIN, LANE_PROB_LOST, LANE_DIP_MAX_NS = .25, .15, 1_000_000_000
LANE_PROB_MIN_OLD, LANE_PROB_LOST_OLD = .5, .3
# Gate switch (replays): 'G1' = the floors above (default); None = the pre-10-26 floors (LANE_PROB_*_OLD); 'G2' ignores the lane
# prob, so a side is inferred whenever its strip geometry is valid (strip_input succeeds; otherwise 'invalid_lane_geometry' as
# before). The lane change and lead_close gates are the same in every variant.
GATE_RELAX = 'G1'
# Replay variant A, blinker alone (laneChangeState off): at vEgo >= BLINKER_INFER_SPEED (30 km/h) a side
# whose line is confident (prob >= BLINKER_PROB) and beside the car (|y at the first model point| >= BLINKER_Y m, not
# the line being crossed) is inferred as usual; otherwise the blinker resets it ('lane_change'), as does any
# laneChangeState other than off (10-20 segment 9: a blinker held through two lane changes kept the old white dashed
# line for 10 s while the model saw the yellow solid one). Below the speed (turn lanes, intersections) the blinker
# resets as before (10-20 segment 3 L 14.85 s: a turn-lane block marking taken for a white solid, DaBin 10-20).
BLINKER_PROB, BLINKER_Y, BLINKER_INFER_SPEED = .7, 1., 8.3
# Lane-change inference (10-24, product C2): at >= LANECHANGE_SPEED with the blinker or laneChangeState pre/starting/
# finishing, a side is inferred whatever its lane prob (blinker with laneChangeState off: prob >= .3) and only heads
# with model confidence >= .9 enter the windows; crossing a line resets that side, and a side confirmed during the
# blinker clears its recognition evidence below the speed, retaining card's last confirmed paint. Replay on 11 sets with the 10-24
# human labels: lane-change correct 82 -> 156 s, total errors 346 -> 254 s, 19 s new (codex/lanechange1024/REPORT3_KO.md).
# 'A' is the earlier BLINKER_* rule above; it and the other variants are kept for replays (CCNC_LANECHANGE_INFER).
LANECHANGE_VARIANTS = ('A', 'B', 'B-cross', 'B-cross-0.95', 'B-p0.2', 'B-cross-p0.2', 'C1', 'C2', 'C1-0.5')
LANECHANGE_SPEED = 24. / 3.6
# StripNet, lead car close (radarState leadOne with status set, dRel < LEAD_GATE_DREL m, the radarState within
# LEAD_MAX_AGE_NS of the frame's camera stamp): a side that would be inferred is 'lead_close' instead, which pauses like
# 'lane_prob_dip' (no inference; strip_filter windows / latch and DisplayPolicy are kept, no dip timer). At this range the
# lead car's body fills the strip and can be read as a white line (10-19 16-seg 3fd/5 36.9-42.2 s: 5.3 s with three 407-round seeds); the
# A/B on the strip cache is .analysis/ccnc-field-20261021/REPORT_gate_KO.md (L4; 6 m cost rain white bookmarks 9 -> 4).
LEAD_GATE_DREL, LEAD_MAX_AGE_NS = 4., 200_000_000
# StripNet v4 (strip_model.OUTPUTS_V4): a side whose type argmax is 'unknown' comes back with reason 'unknown' and no
# probabilities; it pauses like 'lane_prob_dip' / 'lead_close' (strip_filter windows / latch and DisplayPolicy kept).
# Its extra fields (strip_model.decide4) are carried in the raw result and in the packet's raw entry (card reads only
# the confirmed heads, marking.parse), and only when present, so a v1-v3 packet is unchanged.
V4_EXTRAS = ('typeClass', 'colorClass', 'double', 'blue', 'pUnknown')


def lanechange_of():
  """The product variant is C2; the environment selects another one for tests and replays."""
  return os.environ.get('CCNC_LANECHANGE_INFER', 'C2')


def requested(started, params, CP=None):
  return started and mode_of(params) in MODES


def lead_d_rel(radar_state, log_mono_ns, stamp_ns, max_age_ns=LEAD_MAX_AGE_NS):
  """leadOne dRel (m) of a radarState received at ``log_mono_ns`` (0: never) for the frame at camera stamp ``stamp_ns``;
  None when the lead's status is not set or the radarState is more than ``max_age_ns`` away from the frame."""
  if not log_mono_ns or abs(stamp_ns - log_mono_ns) > max_age_ns:
    return None
  lead = radar_state.leadOne
  return float(lead.dRel) if lead.status else None


def line_dict(line, shift_m=0.):
  """Model lane line; ``shift_m`` moves it towards the car by the distance driven between the model's frame and
  the processed frame (straight-motion approximation, used only for a neighbouring frame)."""
  return {'x': [x - shift_m for x in line.x], 'y': list(line.y), 'z': list(line.z)}


class FrameBuffer:
  """Newest few camera frames with their own frameId/EOF (captured at recv time, not from the client later)
  and the monotonic time they were received."""
  def __init__(self, size=FRAME_BUFFER):
    from collections import deque
    self.frames = deque(maxlen=size)

  def drain(self, client):
    """Receive everything queued without waiting; only the newest ``size`` frames are kept."""
    received = 0
    while True:
      buf = client.recv(timeout_ms=0)
      if buf is None:
        return received
      self.frames.append((int(client.frame_id), int(client.timestamp_eof), buf, time.monotonic_ns()))
      received += 1

  def select(self, frame_id, eof, max_offset_ns=MAX_FRAME_OFFSET_NS):
    """(frame_id, eof, buf, kind, recv_ns) for the model's frame: exact first, else nearest EOF within the offset."""
    for fid, feof, buf, recv_ns in self.frames:
      if fid == frame_id and feof == eof:
        return fid, feof, buf, 'exact', recv_ns
    if not self.frames:
      return None
    fid, feof, buf, recv_ns = min(self.frames, key=lambda f: abs(f[1] - eof))
    return (fid, feof, buf, 'nearest', recv_ns) if abs(feof - eof) <= max_offset_ns else None

  def clear(self):
    self.frames.clear()


def buffer_drop_reason(frame_id, buf, recv_ns, now_ns, max_hold_ns=MAX_HOLD_NS):
  """None while the held buffer still holds frame ``frame_id``, else 'hold' or 'frame_id'.

  The hold time is the guarantee: the ISP overwrites a slot's pixels before camerad updates the frame id stored
  in it, so a matching ``buf.frame_id`` alone does not prove the pixels are untouched. A different id means the
  slot has already been republished (or camerad restarted) and is always a drop."""
  if now_ns - recv_ns >= max_hold_ns:
    return 'hold'
  if int(buf.frame_id) != frame_id:
    return 'frame_id'
  return None


class Recogniser:
  """Only the selected engine is constructed (the other is never imported or loaded). The daemon builds only StripNet
  (mode 2); the Scanner (mode 1) remains for offline replays (tools/lane_profile/replay_marking.py) and is not offered on
  the car any more. ``model`` (mode 2): the StripModel to use; None loads strip_model.StripModel() (lane_strip_v4).

  StripNet (mode 2) always runs its type and colour results through a strip_filter.StripFilter, by default the
  asymmetric A8/4 type filter (strip_filter.LONG_M / SHORT_M) with the strip_filter.COLOR_M colour window and the
  latch (strip_filter.LATCH: the last confident type / colour is kept until the other one is confident; type confidence
  strip_filter.TYPE_CONF, type change confirmation strip_filter.TYPE_LATCH_M: none since the 10-26 return to it), applied
  to each side's filter_input result before it leaves process(). ``strip_filter`` is for tests and replays only:
  None builds that default, another StripFilter is used instead of it, and False turns the filter off (the raw
  filter_input results, both heads). The Scanner (mode 1) has no filter unless one is passed, and ignores it."""
  def __init__(self, mode, strip_filter=None, model=None, *, lanechange=None):
    self.mode = mode
    self.lanechange = lanechange_of() if lanechange is None else lanechange
    if self.lanechange not in LANECHANGE_VARIANTS:
      raise ValueError(f'unknown lane-change inference variant: {self.lanechange}')
    self.crossing_y = {}
    self.crossing_min = {}
    self.crossing_stamp = None
    self.lc_seen = {side: set() for side in ('left', 'right')}
    self.lc_frame = None
    self.lc_fresh = {}
    if strip_filter is None and mode == 2:
      from openpilot.selfdrive.carrot.lane_type.strip_filter import COLOR_M, LATCH, LONG_M, SHORT_M, TYPE_CONF, TYPE_LATCH_M, StripFilter
      strip_filter = StripFilter('asym', long_m=LONG_M, short_m=SHORT_M, color_m=COLOR_M, latch=LATCH, type_conf=TYPE_CONF,
                                 type_latch_m=TYPE_LATCH_M)
    self.strip_filter = strip_filter or None
    self.strip_extractor = None
    self.timing = {}
    self.dip_since = {}  # side -> camera stamp of the first frame below LANE_PROB_MIN (StripNet)
    if mode == 1:
      from openpilot.selfdrive.carrot.lane_type.profile_scanner import ProfileScanner
      self.scanners = {side: ProfileScanner(side) for side in ('left', 'right')}
      self.model = None
    elif mode == 2:
      if model is None:
        from openpilot.selfdrive.carrot.lane_type.strip_model import StripModel
        model = StripModel()
      self.model = model
      self.scanners = None
    else:
      raise ValueError('lane marking mode must be 1 (Scanner) or 2 (StripNet)')

  def reset(self):
    """Forget tracking that may hold samples from a reused buffer (StripNet itself keeps no per-frame state; its
    strip filter's windows are emptied)."""
    if self.scanners:
      for scanner in self.scanners.values():
        scanner.reset()
    if self.strip_filter is not None:
      self.strip_filter.reset()
    self.dip_since.clear()
    self.crossing_y.clear()
    self.crossing_min.clear()
    self.crossing_stamp = None
    for heads in self.lc_seen.values():
      heads.clear()
    self.lc_frame = None
    self.lc_fresh.clear()

  def _lifecycle_resets(self, frame):
    """C2 clears fresh blinker recognition evidence on slowdown, independently of card's paint memory."""
    if self.lanechange != 'C2':
      return set()
    stamp = frame['stamp']
    if self.lc_frame is not None and stamp <= self.lc_frame['stamp']:
      for heads in self.lc_seen.values():
        heads.clear()
    self.lc_frame = frame
    resets = {side for side, heads in self.lc_seen.items() if heads and frame.get('v_ego', 0.) < LANECHANGE_SPEED}
    for side in resets:
      self.lc_seen[side].clear()
    return resets

  def observe_confirmation(self, confirmed, frame):
    """Use the existing DisplayPolicy's confirmations; a latched old value is not new model evidence."""
    if self.lanechange != 'C2':
      return
    for side in ('left', 'right'):
      for head, h in confirmed.get(side, {}).items():
        if h['stamp'] != frame['stamp'] or not self.lc_fresh.get((side, head)):
          continue
        if frame.get('blinker') and frame.get('v_ego', 0.) >= LANECHANGE_SPEED:
          self.lc_seen[side].add(head)
        elif not frame.get('blinker'):
          self.lc_seen[side].discard(head)

  def _lanechange_infer(self, frame):
    return (self.lanechange != 'A' and frame.get('v_ego', 0.) >= LANECHANGE_SPEED
            and (frame.get('blinker', False) or frame.get('lcs_change', False)
                 or frame.get('lcs') in ('preLaneChange', 'laneChangeStarting', 'laneChangeFinishing')))

  def _crossing(self, frame):
    """Sign crossing or the same near-then-outward renumber used by desire_lib.LaneCrossing.

    A renumber changes both inner-line identities; reset each side before admitting its new evidence.
    The minimum is restarted after a crossing, so a held blinker can contain another change.
    """
    stamp = frame['stamp']
    if (not self.lanechange.startswith(('B-cross', 'C')) or not self._lanechange_infer(frame)
        or (self.crossing_stamp is not None and not 0 < stamp - self.crossing_stamp <= 300_000_000)):
      self.crossing_y.clear()
      self.crossing_min.clear()
    self.crossing_stamp = stamp
    if not self.lanechange.startswith(('B-cross', 'C')) or not self._lanechange_infer(frame):
      return False
    crossed = False
    current = {}
    for side in ('left', 'right'):
      ys = frame['lines'][side]['y']
      if not len(ys) or not math.isfinite(ys[0]):
        self.crossing_y.pop(side, None)
        self.crossing_min.pop(side, None)
        continue
      y = current[side] = float(ys[0])
      old = self.crossing_y.get(side)
      near = self.crossing_min.get(side, abs(y))
      crossed |= (old is not None and old * y < 0) or (near < 1. and abs(y) - near > 1.5)
      self.crossing_min[side] = min(near, abs(y))
      if y != 0:
        self.crossing_y[side] = y
    if crossed:
      self.crossing_min = {side: abs(y) for side, y in current.items()}
      self.crossing_y = current
    return crossed

  def _skip_reason(self, side, frame):
    """StripNet: why this side is not inferred ('lane_change', 'not_detected', 'lane_prob_dip', 'lead_close'), or None."""
    prob, stamp = frame['probs'][side], frame['stamp']
    if self._lanechange_infer(frame):
      self.dip_since.pop(side, None)
      if self.lanechange.endswith('-p0.2') and not prob >= .2:
        return 'lane_prob_dip'  # Pause this side below the experimental lane-probability floor.
      if self.lanechange.startswith('C') and not frame.get('lcs_change') and frame.get('lcs', 'off') == 'off':
        if not prob >= (.5 if self.lanechange == 'C1-0.5' else .3):
          return 'lane_prob_dip'
      lead = frame.get('lead_d_rel')
      return 'lead_close' if lead is not None and lead < LEAD_GATE_DREL else None
    if self.lanechange != 'A' and (frame.get('blinker') or frame.get('lcs_change')
                                 or frame.get('lcs') in ('preLaneChange', 'laneChangeStarting', 'laneChangeFinishing')):
      return 'lane_change'  # Below 24 km/h still resets.
    floor, lost = {'G1': (LANE_PROB_MIN, LANE_PROB_LOST), 'G2': (-math.inf, -math.inf)}.get(GATE_RELAX, (LANE_PROB_MIN_OLD, LANE_PROB_LOST_OLD))
    if prob >= floor:
      self.dip_since.pop(side, None)
    else:
      since = self.dip_since.setdefault(side, stamp)
      if stamp < since:
        since = self.dip_since[side] = stamp
    if 'blinker' in frame:
      blinker = frame['blinker'] and not (frame.get('v_ego', 0.) >= BLINKER_INFER_SPEED and prob >= BLINKER_PROB
                                          and abs(frame['lines'][side]['y'][0]) >= BLINKER_Y)
      if frame['lcs_change'] or blinker:
        return 'lane_change'
    elif frame['lane_change']:  # a frame without the split flags (older callers): any lane change resets
      return 'lane_change'
    if prob >= floor or (GATE_RELAX == 'G2' and math.isfinite(prob)):
      lead = frame.get('lead_d_rel')
      return 'lead_close' if lead is not None and lead < LEAD_GATE_DREL else None
    return 'lane_prob_dip' if prob >= lost and stamp - since <= LANE_DIP_MAX_NS else 'not_detected'

  def process(self, y_plane, uv_plane, frame):
    """frame: lines/probs per side, rpy, k, stamp (camera EOF ns), odometer, leads, lane_change (blinker or
    laneChangeState), optionally blinker, lcs_change (laneChangeState != off), v_ego and lead_d_rel (radarState leadOne dRel m,
    None: no valid lead) separately for StripNet."""
    out = {}
    self.timing = {}
    if self.mode == 1:
      for side in ('left', 'right'):
        r = self.scanners[side].update(y_plane, uv_plane, frame['lines'][side], frame['probs'][side], frame['rpy'], frame['k'],
                                       frame['stamp'], frame['odometer'], leads=frame['leads'], lane_change=frame['lane_change'])
        out[side] = filter_input(r['type'], r['color'], .99, .99, frame['stamp'], dashed_observed=r['type'] == 'dashed',
                                 reason='lane_change' if frame['lane_change'] else r['reason'], reset=r['reset'])
      return out
    from openpilot.selfdrive.carrot.lane_type.strip_model import batch_strip_inputs
    lifecycle_resets = self._lifecycle_resets(frame)
    extract_start = time.monotonic_ns()
    strips, sides = [], []
    strip = getattr(self.model, 'strip', {})  # the manifest's strip geometry (StripModel.strip); a replay wrapper without it: default
    # v4 mirror_right: the right strip's columns are flipped so its ego side is on the high columns, as on the left strip
    # (strip_model.MIRROR_AXIS); a replay wrapper without the attribute (or a v1-v3 model): no flip.
    mirror = getattr(self.model, 'mirror_right', getattr(getattr(self.model, 'model', None), 'mirror_right', False))
    eligible = []
    for side in ('left', 'right'):
      skip = self._skip_reason(side, frame)
      if skip:
        out[side] = filter_input('unknown', 'unknown', 0., 0., frame['stamp'], reason=skip)
      else:
        eligible.append(side)
    if eligible:
      if self.strip_extractor is None:
        from openpilot.selfdrive.carrot.lane_type.strip_extractor import Extractor
        self.strip_extractor = Extractor()
      extracted = batch_strip_inputs(y_plane, uv_plane, [frame['lines'][s] for s in eligible], frame['rpy'], frame['k'],
                                     [strip], self.strip_extractor)
      for side, s in zip(eligible, extracted, strict=True):
        if s is None:
          out[side] = filter_input('unknown', 'unknown', 0., 0., frame['stamp'], reason='invalid_lane_geometry')
          continue
        strips.append(s[..., ::-1] if mirror is True and side == 'right' else s)
        sides.append(side)
    infer_start = time.monotonic_ns()
    predictions = self.model.infer(strips)
    self.timing = {'extractMs': (infer_start - extract_start)/1e6, 'inferMs': (time.monotonic_ns() - infer_start)/1e6}
    models = {}
    for side, r in zip(sides, predictions, strict=True):
      # The strip model's own confident dashed decision is its pattern evidence.
      out[side] = filter_input(r['type'], r['color'], r['typeConfidence'], r['colorConfidence'], frame['stamp'],
                               dashed_observed=r['type'] == 'dashed', reason=r['reason'])
      out[side].update({k: r[k] for k in V4_EXTRAS if k in r})
      models[side] = r
    changing = self._lanechange_infer(frame)
    crossed = self._crossing(frame)
    if changing:
      for side, r in models.items():
        threshold = .95 if self.lanechange == 'B-cross-0.95' and frame['probs'][side] < .3 else .9
        for head in ('type', 'color'):
          if not math.isfinite(r[head + 'Confidence']) or r[head + 'Confidence'] < threshold:
            out[side][head], out[side][head + 'Confidence'] = 'unknown', 0.
        if out[side]['type'] == out[side]['color'] == 'unknown':
          out[side]['reason'] = 'lane_prob_dip'  # pause without clearing the windows or display
    if crossed or lifecycle_resets:
      for side in ('left', 'right') if crossed else lifecycle_resets:
        out[side]['trackingReset'] = True
        self.dip_since.pop(side, None)
    if self.strip_filter is not None:
      from openpilot.selfdrive.carrot.lane_type.strip_filter import p_dashed, p_yellow
      for side in ('left', 'right'):
        # filter_input drops the model's probabilities, so P(dashed) / P(yellow) come from the model output;
        # reset-reason sides (no inference) are passed too and empty the windows; a lane_prob_dip / lead_close side is passed
        # without probabilities, so it is skipped and keeps them. Unlearned yellow-dashed frames stay out of both, and so does a
        # v4 'unknown' (type argmax unknown) side: it is passed like lane_prob_dip.
        r = models.get(side)
        learned = r is not None and r['reason'] == 'strip_model'
        p, py = (p_dashed(r), p_yellow(r)) if learned else (None, None)
        if not learned and r is not None and r['reason'] == 'unknown' and 'pYellowHeld' in r:
          py = r['pYellowHeld']  # A/B F3 (strip_model.HOLD_COLOR): the type abstains, the colour still enters its window
        if changing:
          if out[side]['type'] == 'unknown':
            p = None
          if out[side]['color'] == 'unknown':
            py = None
        out[side] = dict(self.strip_filter.update(side, out[side], p, frame['stamp'], frame['odometer'], py),
                         typePDashed=p, colorPYellow=py)
    if self.lanechange == 'C2':
      self.lc_fresh = {(side, head): r.get('reason') == 'strip_model' and r.get(head) == out[side][head]
                       and r.get(head) != 'unknown' and r[head + 'Confidence'] >= .9
                       for side, r in models.items() for head in ('type', 'color')}
    return out


def filter_input(kind, color, type_conf, color_conf, stamp, *, dashed_observed=False, reason='', reset=False):
  """Result dict in the form display.DisplayPolicy expects."""
  return {'type': kind, 'color': color, 'typeConfidence': float(type_conf if kind in ('dashed', 'solid') else 0.),
          'colorConfidence': float(color_conf if color in ('white', 'yellow') else 0.), 'typeMonoTimeNanos': stamp,
          'colorMonoTimeNanos': stamp, 'dashedPatternObserved': bool(dashed_observed), 'dashedPatternMonoTimeNanos': stamp,
          'reason': reason, 'trackingReset': bool(reset)}


def minor_faults():
  """Linux process minor faults, or None off Linux/when procfs is unavailable. comm may contain spaces and ')'."""
  try:
    with open('/proc/self/stat', encoding='ascii') as stat:
      return int(stat.read().rsplit(')', 1)[1].split()[7])  # field 10; suffix starts at field 3 (state)
  except (OSError, ValueError, IndexError, UnicodeError):
    return None


def packet(mode, frame_id, camera_ns, raw, confirmed, now_ns, timing, counters, model=None):
  """``model``: the active StripModel.info() (name, manifest version, sha256 prefix), carried as 'model' when given
  (marking.parse ignores it)."""
  data = {'type': PACKET_TYPE, 'version': PACKET_VERSION, 'mode': MODES[mode], 'frameId': frame_id, 'cameraMonoTimeNanos': camera_ns,
          'publishMonoTimeNanos': now_ns, 'timing': timing, 'counters': counters}
  if model is not None:
    data['model'] = model
  for side in ('left', 'right'):
    heads = confirmed.get(side, {})
    entry = {'raw': {k: raw.get(side, {}).get(k) for k in ('type', 'color', 'typeConfidence', 'colorConfidence', 'reason')}}
    entry['raw'].update({k: raw[side][k] for k in V4_EXTRAS if k in raw.get(side, {})})  # v4 only (marking.parse ignores raw)
    if 'displayResetNs' in raw.get(side, {}):
      entry['displayResetNs'] = raw[side]['displayResetNs']
    for head in ('type', 'color'):
      h = heads.get(head)
      entry[head] = h['value'] if h else 'unknown'
      entry[head + 'AgeNs'] = int(max(0, now_ns - h['stamp'])) if h else None
    data[side] = entry
  return data


def planes(buffer):
  """Y and interleaved UV views of the shared NV12 buffer (no full-frame copy)."""
  import numpy as np
  w, h, stride, uv_offset = buffer.width, buffer.height, buffer.stride, buffer.uv_offset
  raw = np.frombuffer(buffer.data, dtype=np.uint8)
  if raw.size < uv_offset + stride*(h//2) or w % 2 or h % 2:
    raise ValueError('invalid NV12 layout')
  y = raw[:stride*h].reshape(h, stride)[:, :w]
  uv = raw[uv_offset:uv_offset + stride*(h//2)].reshape(h//2, stride//2, 2)[:, :w//2]
  return y, uv


def main():
  from openpilot.common.params import Params
  params = Params()
  mode = mode_of(params)
  if mode not in MODES:
    return  # OFF: no OpenCV import, no model load, no IPC
  import cv2
  # 2 OpenCV threads (10-23 on-device benchmark, little cores, offroad): StripNet remap (KleidiCV) runs in parallel;
  # new v4 28.6 -> 22.0 ms/frame p50, p95 31.2 -> 24.8, outputs bit-identical. 3 threads had a worse p95.
  cv2.setNumThreads(2)
  from openpilot.selfdrive.carrot.lane_type.strip_extractor import tune_allocator
  tune_allocator()
  from msgq.visionipc import VisionIpcClient, VisionStreamType
  import openpilot.cereal.messaging as messaging
  from openpilot.common.swaglog import cloudlog
  from openpilot.common.transformations.camera import DEVICE_CAMERAS
  from openpilot.selfdrive.carrot.lane_type.display import DisplayPolicy
  from openpilot.selfdrive.carrot.lane_type.strip_model import load

  strip_net = load(warn=lambda message: cloudlog.warning('CCNC lane marking: %s', message))
  if strip_net is None:  # no model: publish nothing (the existing lane display stays) until the option is turned off
    while mode_of(params) in MODES:
      time.sleep(PARAM_CHECK_S)
    return
  model_info = strip_net.info()
  cloudlog.info('CCNC lane marking: StripNet model %s active (manifest v%s, sha256 %s)', model_info['name'], model_info['version'],
                model_info['sha256'])

  if hasattr(os, 'sched_setaffinity'):
    background = set(os.sched_getaffinity(0)) & {0, 1, 2, 3}
    if background:
      os.sched_setaffinity(0, background)
  if hasattr(os, 'nice'):
    os.nice(10)
  sm = messaging.SubMaster(['modelV2', 'liveCalibration', 'carState', 'deviceState', 'roadCameraState', 'radarState'], poll='modelV2')
  pm = messaging.PubMaster(['customReservedRawData2'])
  client = VisionIpcClient('camerad', VisionStreamType.VISION_STREAM_ROAD, False)  # every frame; FrameBuffer keeps 3
  frames = FrameBuffer()
  recogniser, policy = Recogniser(mode, model=strip_net, lanechange=lanechange_of()), DisplayPolicy()
  counters = {'processed': 0, 'exact': 0, 'nearest': 0, 'frame_mismatch': 0, 'stale_inputs': 0, 'errors': 0,
              'drop_hold_before': 0, 'drop_frame_id_before': 0, 'drop_hold_after': 0, 'drop_frame_id_after': 0}
  odometer, last_stamp, next_param = 0., None, time.monotonic() + PARAM_CHECK_S
  while True:
    if time.monotonic() >= next_param:
      next_param = time.monotonic() + PARAM_CHECK_S
      if mode_of(params) not in MODES:
        break
    sm.update(100)
    if not client.is_connected():
      if not client.connect(False):
        continue
      frames.clear()
    frames.drain(client)
    if not sm.updated['modelV2']:
      continue
    model = sm['modelV2']
    picked = frames.select(int(model.frameId), int(model.timestampEof))
    if picked is None:
      counters['frame_mismatch'] += 1
      continue
    frame_id, frame_eof, buffer, kind, recv_ns = picked
    counters[kind] += 1
    calibration, car, device = sm['liveCalibration'], sm['carState'], sm['deviceState']
    if (not sm.valid['liveCalibration'] or str(calibration.calStatus) != 'calibrated' or len(calibration.rpyCalib) != 3 or
        len(model.laneLines) < 3 or not sm.alive['carState'] or not device.started):
      counters['stale_inputs'] += 1
      policy.reset()
      continue
    drop = buffer_drop_reason(frame_id, buffer, recv_ns, time.monotonic_ns())
    if drop:  # nothing read from the buffer yet: skip the frame, keep all state
      counters[f'drop_{drop}_before'] += 1
      continue
    start_ns, cpu_start = time.monotonic_ns(), time.thread_time_ns()
    faults_before = minor_faults()
    stamp = frame_eof  # the processed pixels' time
    offset_s = (frame_eof - int(model.timestampEof))/1e9
    if last_stamp is not None and 0 < stamp - last_stamp < 2_000_000_000:
      odometer += max(0., float(car.vEgo))*(stamp - last_stamp)/1e9
    last_stamp = stamp
    try:
      camera = DEVICE_CAMERAS[(str(device.deviceType), str(sm['roadCameraState'].sensor))].fcam
      k = camera.intrinsics.copy()
      k[0] *= buffer.width/camera.width
      k[1] *= buffer.height/camera.height
      y_plane, uv_plane = planes(buffer)
      shift = max(0., float(car.vEgo))*offset_s
      frame = {'lines': {'left': line_dict(model.laneLines[1], shift), 'right': line_dict(model.laneLines[2], shift)},
               'probs': {'left': float(model.laneLineProbs[1]), 'right': float(model.laneLineProbs[2])},
               'rpy': list(calibration.rpyCalib), 'k': k, 'stamp': stamp, 'odometer': odometer,
               'leads': [{'prob': lead.prob, 'x': [x - shift for x in lead.x], 'y': list(lead.y)} for lead in model.leadsV3],
               'lane_change': bool(car.leftBlinker or car.rightBlinker or str(model.meta.laneChangeState) != 'off'),
               'blinker': bool(car.leftBlinker or car.rightBlinker),
               'lcs': str(model.meta.laneChangeState),
               'lcs_change': str(model.meta.laneChangeState) != 'off', 'v_ego': float(car.vEgo),
               'lead_d_rel': lead_d_rel(sm['radarState'], sm.logMonoTime['radarState'] if sm.seen['radarState'] else 0, stamp)}
      raw = recogniser.process(y_plane, uv_plane, frame)
    except (ValueError, KeyError, AttributeError, cv2.error) as error:
      counters['errors'] += 1
      cloudlog.warning('CCNC lane marking: %s', error)
      policy.reset()
      continue
    drop = buffer_drop_reason(frame_id, buffer, recv_ns, time.monotonic_ns())
    if drop:  # the pixels may have changed while they were read: do not publish, forget what the scanner took in
      counters[f'drop_{drop}_after'] += 1
      recogniser.reset()
      continue
    faults_after = minor_faults()
    now_ns = time.monotonic_ns()
    confirmed = policy.confirm({'cameraMonoTimeNanos': stamp, **raw}, now_ns)
    recogniser.observe_confirmation(confirmed, frame)
    counters['processed'] += 1
    timing = {'processingMs': (now_ns - start_ns)/1e6, 'threadCpuMs': (time.thread_time_ns() - cpu_start)/1e6,
              'cameraToPublishMs': (now_ns - stamp)/1e6, 'frameOffsetMs': offset_s*1e3, 'frameMatch': kind}
    timing.update(recogniser.timing)
    if faults_before is not None and faults_after is not None:
      timing['minfltDelta'] = max(0, faults_after - faults_before)
    payload = json.dumps(packet(mode, int(model.frameId), stamp, raw, confirmed, now_ns, timing, counters, model_info),
                         separators=(',', ':'), allow_nan=False).encode()
    msg = messaging.new_message('customReservedRawData2', size=len(payload), valid=True)
    msg.customReservedRawData2 = payload
    pm.send('customReservedRawData2', msg)


if __name__ == '__main__':
  main()
