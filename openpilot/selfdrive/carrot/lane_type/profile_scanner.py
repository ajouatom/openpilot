"""1-D lane boundary profile scanner: row-aligned strip + odometry-indexed paint/road evidence.

Runtime copy of the validated prototype (tools/lane_profile/scanner.py now re-exports this module);
the recognition rules are unchanged. No SE(2)/3D attitude, optical flow or image registration:
* each frame samples the CURRENT modelV2 boundary, one output row per integer source row
  (4.5-20m, no vertical interpolation), on a metric lateral grid;
* each row is paint / road / unknown. Occlusion, broad (transverse) paint, ambiguous
  stripes, bad exposure and missing paths are UNKNOWN, never a gap;
* rows are indexed by s = odometer + row distance into 0.25m bins, with a small 1-D
  correlation correction for wheel-speed scale drift;
* dashed = a road gap bounded by paint on both sides whose road bins were seen at
  different times after the car moved; solid = a paint run longer than any measured dash.
"""
import math

import cv2
import numpy as np

from openpilot.common.transformations.camera import view_frame_from_device_frame
from openpilot.common.transformations.orientation import rot_from_euler

NEAR_M, FAR_M = 4.5, 20.
HALF_M, COLUMNS, SUB = .75, 96, 4
COL_M = 2*HALF_M/COLUMNS
HOOD_FRACTION = .86
BIN_M = .25
REF_M = 12.

# Row evidence.
PAINT_CONTRAST, PAINT_PEAK, YELLOW_CB = 12., 20., -8.
MIN_WIDTH_M, MAX_WIDTH_M, BROAD_M = .06, .32, .40
PATH_TOL_M, AMBIGUOUS_M, ROAD_HALF_M = .06, .15, .10
SHADOW_RATIO = .75
WEAK_RATIO, TEXTURE_RIDGES = .45, 3
GAP_CONTRAST_RATIO, MIN_PAINT_CONTRAST, MIN_ANCHOR_ROWS, ANCHOR_M = .25, 25., 5, 8.
# Decisions. SOLID_M must exceed the longest measured dash (see measure_dashes); set by caller.
GAP_MIN_M, CONFIRM_TRAVEL_M, GAP_ROAD_FRACTION, MIN_DASH_M = 1., 1., .5, 1.5
SHORT_UNKNOWN_M, SOLID_PAINT_FRACTION = .75, .85
COLOR_NEAR_M, COLOR_MIN_BINS, COLOR_DOMINANCE = 12., 4, .9
YELLOW_SCORE, WHITE_SCORE = 4.5, 2.
BEHIND_M, RESET_GAP_NS, FRESH_NS, LINE_JUMP_M = 30., 500_000_000, 500_000_000, .6


def projection(rpy, k):
  return np.asarray(k, float) @ view_frame_from_device_frame @ rot_from_euler(np.asarray(rpy, float))


def strip_geometry(line, rpy, k, width, height, hood=HOOD_FRACTION, half_m=HALF_M, cols=COLUMNS):
  """Row-aligned sampling maps. Rows are integer source rows (map_y exact); columns are metric.

  ``half_m`` / ``cols``: lateral half width (m) and column count of the strip. The defaults (+-0.75 m, 96 columns) are the
  scanner's and StripNet v1-v3's; a StripNet manifest may ask for another width (strip_model.strip_of). The scanner's row
  evidence (row_evidence) always uses the defaults."""
  x, y, z = (np.asarray(line[c], float) for c in 'xyz')
  if len(x) < 3 or not np.isfinite(x).all() or np.any(np.diff(x) <= 0) or x[0] > NEAR_M or x[-1] < FAR_M + 1:
    return None
  m = projection(rpy, k)
  ds = np.arange(NEAR_M, FAR_M + .001, .02)
  lat, zz = np.interp(ds, x, y), np.interp(ds, x, z)
  p = np.column_stack((ds, lat, zz)) @ m.T
  if (p[:, 2] <= .1).any():
    return None
  v = p[:, 1]/p[:, 2]
  if np.any(np.diff(v) >= 0):
    return None
  top, bottom = max(0, math.ceil(v[-1])), min(math.floor(v[0]), int(height*hood) - 1, height - 1)
  if bottom - top < 8:
    return None
  ys = np.arange(top, bottom + 1)
  d = np.interp(ys, v[::-1], ds[::-1])
  lat, zz = np.interp(d, x, y), np.interp(d, x, z)
  edges = []
  for off in (-half_m, half_m):
    q = np.column_stack((d, lat + off, zz)) @ m.T
    edges.append(q[:, 0]/q[:, 2])
  # Depth is constant across a row's lateral span, so u is linear in the metric offset.
  t = (np.arange(cols*SUB) + .5)/(cols*SUB)
  mx = (edges[0][:, None] + t[None]*(edges[1] - edges[0])[:, None]).astype(np.float32)
  my = np.repeat(ys[:, None].astype(np.float32), cols*SUB, 1)
  return {'ys': ys, 'd': d, 'mx': mx, 'my': my, 'u0': edges[0], 'u1': edges[1], 'matrix': m, 'half_m': half_m, 'cols': cols}


def sample(y_plane, uv_plane, geo):
  """Luma and chroma strips (R x geo['cols'], default COLUMNS). Horizontal box prefilter of SUB samples per column."""
  r, cols = len(geo['ys']), geo.get('cols', COLUMNS)
  if geo['mx'].shape != (r, cols*SUB) or geo['my'].shape != (r, cols*SUB):  # INTER_AREA below would silently resample
    raise ValueError(f"strip sample maps {geo['mx'].shape} / {geo['my'].shape}, expected {(r, cols*SUB)}")
  # The box mean over each column's SUB samples is cv2.resize INTER_AREA on an exact integer factor: the same values
  # as reshape(..).mean(dtype=float32) but ~4x faster (the numpy mean over the interleaved UV axis dominated the daemon).
  lum = cv2.remap(y_plane, geo['mx'], geo['my'], cv2.INTER_LINEAR, borderMode=cv2.BORDER_REPLICATE)
  lum = cv2.resize(lum.astype(np.float32), (cols, r), interpolation=cv2.INTER_AREA)
  uv = cv2.remap(uv_plane, geo['mx']*.5, geo['my']*.5, cv2.INTER_LINEAR, borderMode=cv2.BORDER_REPLICATE)
  uv = cv2.resize(uv.astype(np.float32), (cols, r), interpolation=cv2.INTER_AREA)
  return lum, uv[..., 0], uv[..., 1]


def nv12_planes(nv12, width, height):
  return nv12[:height, :width], nv12[height:height + height//2, :width].reshape(height//2, width//2, 2)


def _runs(mask):
  r, c = mask.shape
  padded = np.zeros((r, c + 2), np.int8)
  padded[:, 1:-1] = mask
  delta = np.diff(padded, axis=1)
  rs, c0 = np.nonzero(delta == 1)
  _, c1 = np.nonzero(delta == -1)
  return rs, c0, c1


def _segment(values, rs, c0, c1, op):
  r, c = values.shape
  flat = np.r_[values.ravel(), 0]
  idx = np.empty(2*len(rs), np.int64)
  idx[0::2], idx[1::2] = rs*c + c0, rs*c + c1
  return op.reduceat(flat, idx)[0::2]


def row_evidence(lum, cb, cr, geo, leads=(), prior=None, paint_ref=40.):
  """Per-row paint/road/unknown along the selected boundary path."""
  r = lum.shape[0]
  d = geo['d']
  road = np.percentile(lum, 30, axis=1).astype(np.float32)
  contrast = lum - road[:, None]
  dcb, dcr = cb - np.median(cb, 1)[:, None], cr - np.median(cr, 1)[:, None]
  mask = (contrast >= np.minimum(PAINT_CONTRAST, np.maximum(6., .12*road))[:, None]) | ((contrast >= 6) & (dcb <= YELLOW_CB))
  rs, c0, c1 = _runs(mask)
  width = (c1 - c0)*COL_M
  center = -HALF_M + (c0 + c1)*.5*COL_M
  n = np.maximum(c1 - c0, 1)
  peak = _segment(contrast, rs, c0, c1, np.maximum)
  mcb = _segment(dcb, rs, c0, c1, np.add)/n
  mcr = _segment(dcr, rs, c0, c1, np.add)/n
  yellow = mcb <= YELLOW_CB
  strong = (peak >= np.minimum(PAINT_PEAK, np.maximum(10., .2*road[rs]))) | yellow
  stripe = strong & (width >= MIN_WIDTH_M) & (width <= MAX_WIDTH_M) & (c1 - c0 >= 2)
  broad = strong & (width > BROAD_M)
  path = fit_path(d[rs[stripe]], center[stripe], width[stripe], yellow[stripe], prior)
  out = {'state': np.full(r, -1, np.int8), 'reason': np.full(r, 'no_path', object), 'path': path,
         'cb': np.full(r, np.nan, np.float32), 'cr': np.full(r, np.nan, np.float32), 'peak': np.full(r, np.nan, np.float32)}
  if path is None:
    return out
  expected = path['offset'] + path['slope']*(d - REF_M)
  e_run = expected[rs]
  match = stripe & (np.abs(center - e_run) <= PATH_TOL_M) & (np.abs(width - path['width']) <= .10)
  near = strong & ~match & (np.abs(center - e_run) <= AMBIGUOUS_M)
  over = broad & (np.abs(center - e_run) <= width/2 + .30)
  # A matched stripe much dimmer than this line's own paint (illumination-scaled) is pavement
  # texture (longitudinal grooves, seams), not paint. Several strong ridges across the strip
  # in one row are texture too. Both leave the row UNKNOWN: neither paint nor gap.
  lit_road = float(np.median(road))
  weak = match & (peak < WEAK_RATIO*paint_ref*road[rs]/max(lit_road, 1.))
  texture = np.bincount(rs[strong & ~match & (width <= MAX_WIDTH_M)], minlength=r) >= TEXTURE_RIDGES
  weak_row = np.bincount(rs[weak], minlength=r) > 0
  match &= ~weak
  paint_row = np.zeros(r, bool)
  paint_row[rs[match]] = True
  paint_row &= ~texture
  weak_row &= ~paint_row
  # Strongest matched run per row supplies its colour and contrast.
  order = np.argsort(peak[match])
  mr = rs[match][order]
  out['cb'][mr], out['cr'][mr], out['peak'][mr] = mcb[match][order], mcr[match][order], peak[match][order]
  ambiguous = np.bincount(rs[near], minlength=r) > 0
  transverse = np.bincount(rs[over], minlength=r) > 0
  # Road check: the path window must look like the surrounding local road.
  col = np.clip(np.rint((expected + HALF_M)/COL_M - .5).astype(int), 0, COLUMNS - 1)
  k = int(round(ROAD_HALF_M/COL_M))
  rows = np.arange(r)[:, None]
  window = lum[rows, np.clip(col[:, None] + np.arange(-k, k + 1), 0, COLUMNS - 1)]
  side = np.r_[-np.arange(10, 26), np.arange(10, 26)]
  ref = np.median(lum[rows, np.clip(col[:, None] + side, 0, COLUMNS - 1)], 1)
  inside = (col - k >= 0) & (col + k < COLUMNS)
  wcb = dcb[rows, np.clip(col[:, None] + np.arange(-k, k + 1), 0, COLUMNS - 1)].min(1)
  # Expected paint contrast scales with the row's illumination: a shadowed stripe is
  # dimmer in absolute terms. Rows darker than the lit road (shadow bands) or with a
  # sharp transverse illumination change never certify a gap; they stay UNKNOWN.
  lit = float(np.median(ref))
  # Absence is only meaningful next to strong paint OF THIS FRAME on this path: a gap must be
  # far fainter than the line's own paint (a worn line is UNKNOWN, not a gap), and must lie
  # within ANCHOR_M of a paint row seen in the same frame.
  painted = np.flatnonzero(paint_row)
  frame_paint = float(np.median(peak[match])) if len(painted) >= MIN_ANCHOR_ROWS else 0.
  faint = (window.max(1) - ref) < np.maximum(4., GAP_CONTRAST_RATIO*frame_paint*ref/max(lit, 1.))
  faint &= frame_paint >= MIN_PAINT_CONTRAST
  if len(painted):
    faint &= np.abs(d[:, None] - d[painted][None]).min(1) <= ANCHOR_M
  step = np.abs(np.diff(road, prepend=road[0])) + np.abs(np.diff(road, append=road[-1]))
  exposed = (ref >= 25) & (ref <= 215) & (ref >= SHADOW_RATIO*lit) & (step <= .15*lit)
  occluded = _occluded(geo, expected, leads)
  state, reason = out['state'], out['reason']
  reason[:] = 'unresolved'
  state[paint_row] = 1
  reason[paint_row] = 'paint'
  road_ok = ~paint_row & ~ambiguous & ~transverse & ~occluded & inside & exposed & faint & (wcb > YELLOW_CB) & ~texture & ~weak_row
  state[road_ok] = 0
  reason[road_ok] = 'road'
  for flag, name in ((texture, 'texture'), (weak_row, 'weak_paint'), (occluded, 'occluded'), (transverse, 'transverse_paint'), (ambiguous, 'ambiguous_stripe'),
                     (~exposed, 'exposure'), (~inside, 'outside_strip')):
    pick = ~paint_row & ~road_ok & flag & (reason == 'unresolved')
    reason[pick] = name
  # Paint occluded by a lead box is still paint, but an occluded "road" never is.
  state[paint_row & occluded] = -1
  reason[paint_row & occluded] = 'occluded'
  return out


def _occluded(geo, expected, leads):
  r = len(geo['ys'])
  hit = np.zeros(r, bool)
  boxes = []
  for lead in leads or ():
    if lead['prob'] < .5 or not len(lead['x']) or not len(lead['y']):
      continue
    x, y = float(lead['x'][0]), float(lead['y'][0])
    if not math.isfinite(x + y) or not 2 < x < 80:
      continue
    corners = np.array([[x, y + dy, dz] for dy in (-1.3, 1.3) for dz in (-2.0, 1.4)]) @ geo['matrix'].T
    if (corners[:, 2] <= 0).any():
      continue
    p = corners[:, :2]/corners[:, 2, None]
    boxes.append((*p.min(0), *p.max(0)))
  if not boxes:
    return hit
  u = geo['u0'] + (expected + HALF_M)/(2*HALF_M)*(geo['u1'] - geo['u0'])
  for x0, y0, x1, y1 in boxes:
    hit |= (u >= x0 - 4) & (u <= x1 + 4) & (geo['ys'] >= y0) & (geo['ys'] <= y1 + 4)
  return hit


OFFSETS = np.arange(-.46, .461, .02)
SLOPES = np.arange(-.03, .0301, .01)


def fit_path(d, center, width, yellow, prior=None):
  """Hough vote over (offset at 12m, slope); support = distinct 0.5m distance bins."""
  if len(d) < 8:
    return None
  if prior is not None:
    offsets = prior['offset'] + np.arange(-.08, .081, .02)
    slopes = prior['slope'] + np.arange(-.01, .0101, .005)
  else:
    offsets, slopes = OFFSETS, SLOPES
  oh, sh = np.meshgrid(offsets, slopes, indexing='ij')
  oh, sh = oh.ravel(), sh.ravel()
  resid = np.abs(center[None] - oh[:, None] - sh[:, None]*(d[None] - REF_M))
  hit = resid <= .04
  bins = np.floor((d - NEAR_M)/.5).astype(int)
  onehot = np.zeros((len(d), bins.max() + 1), np.float32)
  onehot[np.arange(len(d)), bins] = 1
  support = ((hit.astype(np.float32) @ onehot) > 0).sum(1)*.5
  score = support - 2.*np.abs(oh) - (0 if prior is None else 4.*np.abs(oh - prior['offset']))
  best = int(np.argmax(score))
  if support[best] < 2.:
    return None
  paths = []
  # Parallel second stripe (double line). The ego-side member is primary.
  for i in np.argsort(-score):
    if support[i] < max(2., .5*support[best]):
      break
    if all(abs(oh[i] - p['offset']) >= .12 for p in paths) and abs(sh[i] - sh[best]) <= .01 and (
        not paths or abs(oh[i] - paths[0]['offset']) <= .45):
      sel = hit[i]
      paths.append({'offset': float(oh[i]), 'slope': float(sh[i]), 'support_m': float(support[i]),
                    'width': float(np.median(width[sel])), 'yellow_fraction': float(yellow[sel].mean())})
      if len(paths) == 2:
        break
  return {**paths[0], 'members': paths}


def choose_primary(path, side):
  """For a double boundary, the stripe nearer ego (left: larger y, right: smaller y)."""
  if path is None or len(path['members']) < 2:
    return path
  member = max(path['members'], key=lambda p: p['offset'] if side == 'left' else -p['offset'])
  return {**member, 'members': path['members']}


class ProfileScanner:
  def __init__(self, side, solid_m=12., confirm_travel_m=CONFIRM_TRAVEL_M):
    self.side, self.solid_m, self.confirm_travel_m = side, solid_m, confirm_travel_m
    self.generation = 0
    self.reset()

  def reset(self):
    self.bins, self.correction, self.prior, self.last_stamp, self.last_line = {}, 0., None, None, None
    self.last_path_stamp, self.paint_ref = None, 40.
    self.generation += 1

  def update(self, y_plane, uv_plane, line, probability, rpy, k, stamp, odometer, *, leads=(), lane_change=False):
    result = {'type': 'unknown', 'color': 'unknown', 'reason': '', 'generation': self.generation, 'reset': False,
              'gaps': [], 'solid_run_m': 0., 'rows': 0, 'paint_rows': 0, 'road_rows': 0}
    line_y = float(np.interp(10., line['x'], line['y'])) if len(line['x']) else math.nan
    if (lane_change or self.last_stamp is not None and not 0 < stamp - self.last_stamp <= RESET_GAP_NS or
        self.last_line is not None and abs(line_y - self.last_line) > LINE_JUMP_M):
      self.reset()
      result.update(reset=True, generation=self.generation)
    self.last_stamp, self.last_line = stamp, line_y
    if lane_change:
      return dict(result, reason='lane_change')
    geo = strip_geometry(line, rpy, k, y_plane.shape[1], y_plane.shape[0]) if probability >= .5 else None
    if geo is None:
      self.prior = None
      return self._decide(dict(result, reason='no_geometry'), stamp, odometer, observed=False)
    lum, cb, cr = sample(y_plane, uv_plane, geo)
    ev = row_evidence(lum, cb, cr, geo, leads, self.prior, self.paint_ref)
    path = choose_primary(ev['path'], self.side)
    if path is not ev['path'] and path is not None:
      ev = row_evidence(lum, cb, cr, geo, leads, path, self.paint_ref)
      ev['path'] = choose_primary(ev['path'], self.side)
    self.prior = ev['path']
    state = ev['state']
    result.update(rows=len(state), paint_rows=int((state == 1).sum()), road_rows=int((state == 0).sum()),
                  path=ev['path'], evidence=ev, geometry=geo)
    if ev['path'] is None:
      return self._decide(dict(result, reason='no_path'), stamp, odometer, observed=False)
    if (state == 1).sum() >= 3:
      # Rise fast to the line's real paint; decay slowly so texture cannot redefine "paint".
      m = float(np.nanmedian(ev['peak'][state == 1]))
      self.paint_ref = .8*self.paint_ref + .2*m if m > self.paint_ref else .99*self.paint_ref + .01*m
    self.last_path_stamp = stamp
    self._accumulate(geo['d'], state, ev['cb'], ev['cr'], stamp, odometer)
    return self._decide(dict(result, reason='observed'), stamp, odometer, observed=True)

  def _accumulate(self, d, state, cb, cr, stamp, odometer):
    known = state >= 0
    if not known.any():
      return
    s = odometer + self.correction + d[known]
    keys = np.floor(s/BIN_M).astype(int)
    st, dd = state[known], d[known]
    frame = {}
    for key, value, dist, b, c in zip(keys.tolist(), st.tolist(), dd.tolist(), cb[known].tolist(), cr[known].tolist(), strict=True):
      e = frame.setdefault(key, [0, 0, [], dist])
      e[value] += 1  # e[0] road rows, e[1] paint rows
      if value == 1 and dist <= COLOR_NEAR_M and math.isfinite(b):
        e[2].append((b, c))
    # 1-D drift correction against established bins (wheel-speed scale, model distance bias).
    labels = {key: 1 if e[1] and not e[0] else 0 if e[0] and not e[1] else None for key, e in frame.items()}
    edges = sum(labels.get(key) is not None and labels.get(key + 1) is not None and labels[key] != labels[key + 1]
                for key in labels)
    if edges >= 1 and self.bins:
      def score(shift):
        total = 0
        for key, value in labels.items():
          old = self.bins.get(key + shift)
          if value is None or old is None:
            continue
          ref = 1 if old['p'] >= 2 and old['r'] == 0 else 0 if old['r'] >= 2 and old['p'] == 0 else None
          if ref is not None:
            total += 1 if ref == value else -1
        return total
      scores = {shift: score(shift) for shift in range(-4, 5)}
      best = max(scores, key=lambda x: (scores[x], -abs(x)))
      if best and scores[best] - scores[0] >= 3:
        self.correction += best*BIN_M
        frame = {key + best: e for key, e in frame.items()}
    for key, (road, paint, colors, _dist) in frame.items():
      b = self.bins.setdefault(key, {'p': 0, 'r': 0, 'road_odo': [math.inf, -math.inf], 'stamp': stamp, 'colors': [],
                                     'color_frames': 0})
      if paint and not road:
        b['p'] += 1
      elif road and not paint:
        b['r'] += 1
        b['road_odo'] = [min(b['road_odo'][0], odometer), max(b['road_odo'][1], odometer)]
      b['stamp'] = stamp
      if colors:
        b['colors'].append(tuple(np.median(np.asarray(colors), 0)))
        b['color_frames'] += 1
    low = math.floor((odometer + self.correction - BEHIND_M)/BIN_M)
    for key in [key for key in self.bins if key < low]:
      del self.bins[key]

  def bin_state(self, b):
    if b is None:
      return None
    if b['p'] and b['p'] >= 3*b['r']:
      return 1
    if b['r'] >= 2 and b['r'] >= 3*b['p'] and b['road_odo'][1] - b['road_odo'][0] >= self.confirm_travel_m:
      return 0
    if b['r'] and not b['p']:
      return 'r'  # road seen, not yet confirmed from a different time/position
    return None

  def _decide(self, result, stamp, odometer, observed):
    fresh = self.last_path_stamp is not None and 0 <= stamp - self.last_path_stamp <= FRESH_NS
    if not self.bins or not fresh:
      return dict(result, reason=result['reason'] or 'no_evidence')
    lo, hi = min(self.bins), max(self.bins)
    states = [self.bin_state(self.bins.get(key)) for key in range(lo, hi + 1)]
    # Runs of (state, start, end) over contiguous bins.
    runs = []
    for i, value in enumerate(states):
      if runs and runs[-1][0] == value:
        runs[-1][2] = i
      else:
        runs.append([value, i, i])
    gaps = find_gaps(states, lo)
    last_gap_end = max((g['start_m'] + g['length_m'] for g in gaps), default=-math.inf)
    solid_runs = paint_runs(states, lo)
    # The latest continuous paint run (after every verified gap) decides solid; a long run
    # older than a verified gap belongs to a previous marking.
    latest = max((r['length_m'] for r in solid_runs if r['end_m'] > last_gap_end), default=0.)
    dashes = [r for r in solid_runs if r['bounded']]
    result.update(gaps=gaps, solid_run_m=latest, complete_dashes=dashes)
    # Stretch observed after the last verified gap with no confirmed road at all. If it is
    # longer than any dash, a dashed verdict is stale (the marking changed); without a single
    # continuous run it is UNKNOWN rather than dashed.
    tail = 0.
    if gaps:
      first = int(round(last_gap_end/BIN_M)) - lo
      after = states[max(first, 0):]
      seen = [i for i, v in enumerate(after) if v == 1]
      tail = (seen[-1] + 1)*BIN_M if seen and 0 not in after[:seen[-1] + 1] else 0.
    result['paint_since_gap_m'] = tail
    kind = 'unknown'
    if latest >= self.solid_m:
      kind = 'solid'
    elif gaps and tail < self.solid_m:
      kind = 'dashed'
    colors = []
    for b in self.bins.values():
      if b['color_frames'] and self.bin_state(b) == 1:
        cb_, cr_ = np.median(np.asarray(b['colors']), 0)
        colors.append(classify_color(cb_, cr_))
    color = 'unknown'
    known = [c for c in colors if c != 'unknown']
    if len(known) >= COLOR_MIN_BINS:
      top = max(('white', 'yellow'), key=known.count)
      if known.count(top) >= COLOR_DOMINANCE*len(known):
        color = top
    result.update(type=kind, color=color, color_bins=len(known), observed=observed)
    return result


def paint_runs(states, lo=0):
  """Continuous paint: paint bins joined only across short unknown runs, never across road.

  'bounded' runs have confirmed road directly (within one unknown run) on both ends: these
  are complete dashes, whose length distribution sets the solid threshold.
  """
  runs, i, n = [], 0, len(states)
  while i < n:
    if states[i] != 1:
      i += 1
      continue
    j, end, paint = i, i, 0
    while j < n:
      if states[j] == 1:
        paint, end, j = paint + 1, j, j + 1
        continue
      k = j
      while k < n and states[k] is None:
        k += 1
      if (k - j)*BIN_M <= SHORT_UNKNOWN_M and k < n and states[k] == 1:
        j = k
        continue
      break
    length = (end - i + 1)*BIN_M
    if paint*BIN_M >= SOLID_PAINT_FRACTION*length:
      runs.append({'start_m': (lo + i)*BIN_M, 'end_m': (lo + end + 1)*BIN_M, 'length_m': length,
                   'bounded': _bounded(states, i, -1) and _bounded(states, end, 1)})
    i = end + 1
  return runs


def _bounded(states, index, step):
  k = index + step
  skipped = 0
  while 0 <= k < len(states) and states[k] in (None, 'r') and skipped < 4:
    k, skipped = k + step, skipped + 1
  if not 0 <= k < len(states) or states[k] != 0:
    return False
  run = 0
  while 0 <= k < len(states) and states[k] == 0:
    k, run = k + step, run + 1
  return run*BIN_M >= GAP_MIN_M


def _paint_extent(states, index, step):
  """Paint length of the group containing ``index``, walking away from the gap."""
  paint, unknown, k = 0, 0, index
  while 0 <= k < len(states):
    if states[k] == 1:
      paint, unknown = paint + 1, 0
    elif states[k] is None and unknown < 3:
      unknown += 1
    else:
      break
    k += step
  return paint*BIN_M


def find_gaps(states, lo=0):
  """Verified gaps: between two paint groups, a confirmed-road run of at least GAP_MIN_M.

  Paint groups merge across unknown bins (blurred dash ends, a passing shadow) but never
  across confirmed road. Unknown bins inside a gap add nothing to its measured length.
  """
  paint = [i for i, v in enumerate(states) if v == 1]
  gaps = []
  for a, b in zip(paint[:-1], paint[1:], strict=True):
    if b - a < 2:
      continue
    between = states[a + 1:b]
    if 0 not in between:
      continue
    longest = run = road = 0
    for v in between:
      run = run + 1 if v == 0 else 0
      longest = max(longest, run)
      road += v == 0
    # Both bounding paint groups must be dash-sized: crosswalk bars and other short transverse
    # marks repeat at ~1m and must not make a dash pattern (min measured dash p5 = 2.5m).
    if (longest*BIN_M >= GAP_MIN_M and road >= GAP_ROAD_FRACTION*len(between) and
        _paint_extent(states, a, -1) >= MIN_DASH_M and _paint_extent(states, b, 1) >= MIN_DASH_M):
      gaps.append({'start_m': (lo + a + 1)*BIN_M, 'length_m': len(between)*BIN_M, 'road_m': road*BIN_M})
  return gaps


def classify_color(dcb, dcr):
  """Paint chroma relative to the same rows' road chroma (removes white-balance cast).

  Yellow paint lowers Cb and raises Cr; score = dCr - dCb. Thresholds come from TRAIN-route
  reviewed frames only (white: 99th pct <= 3.1, yellow 25th pct 4.8 on qcamera; fcamera yellow > 16).
  The band between is UNKNOWN.
  """
  score = dcr - dcb
  if score >= YELLOW_SCORE:
    return 'yellow'
  if score <= WHITE_SCORE:
    return 'white'
  return 'unknown'
