"""Offline RGB traffic-signal observer. No messaging, Params or control outputs.

Known horizontal lamp layout only. Frame state describes visually agreeing tracks,
not lane association or permission to move. Feed frames in timestamp order.
"""
from dataclasses import dataclass, field
import math
import cv2
import numpy as np


def overlap(a, b):
  inter = max(0, min(a[2], b[2]) - max(a[0], b[0])) * max(0, min(a[3], b[3]) - max(a[1], b[1]))
  return inter / max(1, (a[2]-a[0])*(a[3]-a[1]) + (b[2]-b[0])*(b[3]-b[1]) - inter)


def small_percentile(values, percentile):
  """Linear percentile for nonempty finite image samples, without generic setup."""
  a = values.ravel()
  rank = (a.size - 1) * (percentile / 100.)
  lo, hi = math.floor(rank), math.ceil(rank)
  work = np.partition(a, (lo, hi))
  low, high = float(work[lo]), float(work[hi])
  fraction = rank - lo
  return high - (high - low) * (1. - fraction) if fraction >= .5 else low + (high - low) * fraction


def dark_scene(rgb):
  gray = cv2.cvtColor(rgb[:int(rgb.shape[0] * .55)], cv2.COLOR_RGB2GRAY)
  cumulative = np.cumsum(cv2.calcHist([gray], [0], None, [256], [0, 256]).ravel())
  rank = .7 * (gray.size - 1)
  lower, upper = np.searchsorted(cumulative, [math.floor(rank), math.ceil(rank)], side='right')
  return lower + (upper - lower) * (rank - math.floor(rank)) <= 45


def lamp_evidence(rgb, box):
  x1, y1, x2, y2 = map(int, box)
  a = rgb[y1:y2, x1:x2]
  if a.size == 0:
    return 'unknown', 0., {}
  h, w = a.shape[:2]
  hsv = cv2.cvtColor(a, cv2.COLOR_RGB2HSV)
  hue, sat, val = cv2.split(hsv)
  red = ((hue <= 12) | (hue >= 170)) & (sat >= 90) & (val >= 95)
  green = (hue >= 38) & (hue <= 105) & (sat >= 65) & (val >= 95)
  yy, xx = np.mgrid[:h, :w]
  # Use interior lamps, excluding housing borders and bright adjacent sky.
  scores = []
  peaks = []
  gray = cv2.cvtColor(a, cv2.COLOR_RGB2GRAY)/255.
  median = float(np.median(gray))
  for mask, xc in [(red, .125), (green, .875)]:
    valid = mask & (xx >= (xc-.13)*w) & (xx <= (xc+.13)*w) & (yy >= .15*h) & (yy <= .85*h)
    count, ids, stats, centroids = cv2.connectedComponentsWithStats(valid.astype(np.uint8), 8)
    best = 0.
    for k in range(1, count):
      bx, by, bw, bh, area = stats[k]
      if area < 2 or not .4 <= bw/max(bh, 1) <= 2.5:
        continue
      cx, cy = centroids[k]
      position = max(0., 1-abs(cx/w-xc)/.22) * max(0., 1-abs(cy/h-.5)/.6)
      strength = small_percentile(val[ids == k], 80)/255
      best = max(best, position*strength*min(1., area/max(2., h*h*.12)))
    scores.append(best)
    disk = (xx+.5-xc*w)**2 + (yy+.5-.5*h)**2 <= max(1., .32*h)**2
    peaks.append(max(0., small_percentile(gray[disk], 85)-median))
  r, g = scores
  state = 'red' if r >= .18 and g < .12 else 'green' if g >= .18 and r < .12 else 'unknown'
  return state, max(r, g), dict(red_score=r, green_score=g, left_brightness=peaks[0], right_brightness=peaks[1])


def housing_proposals(rgb):
  """Current-image housing proposals. Never accepts annotations or future frames."""
  height, width = rgb.shape[:2]
  y_end = int(height*.60)
  gray = cv2.cvtColor(rgb[:y_end], cv2.COLOR_RGB2GRAY)
  candidates = []
  for threshold in (45, 70, 100, 130):
    dark = cv2.compare(gray, threshold, cv2.CMP_LT)
    dark = cv2.morphologyEx(dark, cv2.MORPH_OPEN, np.ones((2, 3), np.uint8))
    contours, _ = cv2.findContours(dark, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    for contour in contours:
      x, y, w, h = cv2.boundingRect(contour)
      if not (16 <= w <= 220 and 4 <= h <= 65 and 2.6 <= w/h <= 6.8):
        continue
      if x < width*.04 or x+w > width*.96 or y < 3 or y+h >= y_end:
        continue
      fill = cv2.contourArea(contour)/(w*h)
      if fill < .5:
        continue
      box = [x, y, x+w, y+h]
      state, strength, evidence = lamp_evidence(rgb, box)
      if state == 'unknown':
        continue
      # Dark horizontal housing with lit lamp at its expected end.
      interior = float(np.median(gray[y:y+h, x:x+w]))
      surround = np.concatenate([gray[max(0,y-3):y, x:x+w].ravel(), gray[y+h:y+h+3, x:x+w].ravel()])
      contrast = float(np.median(surround))-interior
      if contrast < 8:
        continue
      quality = strength * min(1., fill/.75)
      if quality >= .25:
        candidates.append(dict(box=box, raw=state, quality=float(quality), source='detected', **evidence))
  kept = []
  for c in sorted(candidates, key=lambda v: -v['quality']):
    if all(overlap(c['box'], k['box']) < .35 for k in kept):
      kept.append(c)
  return kept[:20]


def night_proposals(rgb):
  """Recover saturated lamps from a compact white core and its colored halo.

  Inferred horizontal boxes are proposals, not measured housings or lane labels.
  Red halos must surround the core: one-sided reflections cannot seed a track.
  """
  h, w = rgb.shape[:2]
  end = int(h * 0.55)
  gray = cv2.cvtColor(rgb[:end], cv2.COLOR_RGB2GRAY)
  # This extension is used only when the upper scene is dark.
  # Exact linear 70th percentile for uint8, avoiding a full-frame partition.
  cumulative = np.cumsum(cv2.calcHist([gray], [0], None, [256], [0, 256]).ravel())
  rank = .7 * (gray.size - 1)
  lower, upper = np.searchsorted(cumulative, [math.floor(rank), math.ceil(rank)], side='right')
  if lower + (upper - lower) * (rank - math.floor(rank)) > 45:
    return []
  # Keep a full maximum-core margin around eligible centers. Color conversion
  # is restricted to each small halo; full-scene HSV was too slow on C4 little CPUs.
  ox, oy = max(0, int(.12 * w) - 38), max(0, int(.08 * h) - 22)
  ex, ey = min(w, math.ceil(.88 * w) + 38), min(end, math.ceil(.52 * h) + 22)
  mask = cv2.compare(gray[oy:ey, ox:ex], 180, cv2.CMP_GT)
  _, _, stats, cents = cv2.connectedComponentsWithStats(mask, 8)
  seeds = []
  for stat, center in zip(stats[1:], cents[1:], strict=True):
    x, y, cw, ch, area = map(int, stat)
    cx, cy = map(float, center)
    x, y, cx, cy = x + ox, y + oy, cx + ox, cy + oy
    diam = max(cw, ch)
    if not (3 <= cw <= 38 and 3 <= ch <= 22 and 7 <= area <= 350 and 0.4 <= cw / ch <= 3.5 and area / (cw * ch) >= 0.35):
      continue
    if not 0.12 * w < cx < 0.88 * w or not 0.08 * h < cy < 0.52 * h:
      continue
    ax = max(0, int(cx - 1.4 * diam))
    bx = min(w, int(cx + 1.4 * diam) + 1)
    ay = max(0, int(cy - 1.4 * diam))
    by = min(end, int(cy + 1.4 * diam) + 1)
    hue, sat, val = cv2.split(cv2.cvtColor(rgb[ay:by, ax:bx], cv2.COLOR_RGB2HSV))
    color = ((hue <= 12) | (hue >= 170)) & (sat >= 90) & (val >= 110)
    yy, xx = np.mgrid[ay:by, ax:bx]
    ring = ((xx - cx) ** 2 + (yy - cy) ** 2 >= 0.35**2 * diam**2) & ((xx - cx) ** 2 + (yy - cy) ** 2 <= 1.3**2 * diam**2)
    is_red = np.count_nonzero(color & ring) >= max(8, np.count_nonzero(ring) * 0.22)
    if is_red:
      if not (0.65 <= cw / ch <= 1.55 and area / (cw * ch) >= 0.55):
        continue
      quadrants = [(xx < cx) & (yy < cy), (xx >= cx) & (yy < cy), (xx < cx) & (yy >= cy), (xx >= cx) & (yy >= cy)]
      coverage = [float(np.mean(color[ring & q])) for q in quadrants]
      if min(coverage) < 0.4 or max(coverage) - min(coverage) > 0.45:
        continue
      state = 'red'
      xc = 0.125
    else:
      # Split a connected arrow+round green core by its largest inscribed disk.
      distance = cv2.distanceTransform(np.pad((gray[y : y + ch, x : x + cw] > 180).astype(np.uint8), 1), cv2.DIST_L2, 5)
      _, radius, _, location = cv2.minMaxLoc(distance)
      if radius < 1.5:
        continue
      cx = x + location[0] - 1
      cy = y + location[1] - 1
      diam = max(3, int(round(2 * radius)))
      ax = max(0, int(cx - 1.4 * diam))
      bx = min(w, int(cx + 1.4 * diam) + 1)
      ay = max(0, int(cy - 1.4 * diam))
      by = min(end, int(cy + 1.4 * diam) + 1)
      yy, xx = np.mgrid[ay:by, ax:bx]
      ring = ((xx - cx) ** 2 + (yy - cy) ** 2 >= 0.35**2 * diam**2) & ((xx - cx) ** 2 + (yy - cy) ** 2 <= 1.3**2 * diam**2)
      hue, sat, val = cv2.split(cv2.cvtColor(rgb[ay:by, ax:bx], cv2.COLOR_RGB2HSV))
      green = (hue >= 38) & (hue <= 105) & (sat >= 65) & (val >= 110)
      if np.count_nonzero(green & ring) < max(8, np.count_nonzero(ring) * 0.22):
        continue
      state = 'green'
      xc = 0.875
    # Nearby white characters of a sign must not form a compact lamp core.
    if state == 'red' and np.count_nonzero((gray[ay:by, ax:bx] > 180) & (((xx - cx) ** 2 + (yy - cy) ** 2) > diam**2)) > area * 0.3:
      continue
    bw = 4 * diam
    bh = max(6, 2 * diam)
    box = [int(cx - bw * xc), int(cy - bh / 2), int(cx + bw * (1 - xc)), int(cy + bh / 2)]
    if box[0] < 0 or box[1] < 0 or box[2] >= w or box[3] >= end:
      continue
    raw, strength, evidence = lamp_evidence(rgb, box)
    if raw != state:
      continue
    seeds.append(dict(box=box, raw=state, quality=max(0.25, strength), source='night_lamp_core', core_diameter=diam, **evidence))
  return seeds

def detect(rgb, *, daytime_cores=False):
  """Daytime core proposals are replay-only pending failed-transition correction."""
  if daytime_cores and not dark_scene(rgb):
    return day_proposals(rgb)[:20]
  candidates = housing_proposals(rgb)
  for candidate in night_proposals(rgb):
    if all(overlap(candidate['box'], other['box']) < .35 for other in candidates):
      candidates.append(candidate)
  return candidates[:20]


def day_proposals(rgb):
  """Forward compact lamps with adjoining dark bar evidence.

  A dark housing can merge into trees/background, so its connected outer contour
  cannot supply the box. Seed a locally bright colored core instead. These are
  inferred boxes, not object/lane labels. Saturation alone is insufficient.
  """
  h, w = rgb.shape[:2]
  ox, oy, ex, ey = int(.25*w), int(.08*h), int(.80*w), int(.52*h)
  hsv = cv2.cvtColor(rgb[oy:ey, ox:ex], cv2.COLOR_RGB2HSV)
  val = hsv[:, :, 2]
  bright = cv2.compare(cv2.morphologyEx(val, cv2.MORPH_TOPHAT, np.ones((15, 15), np.uint8)), 70, cv2.CMP_GE)
  red = cv2.bitwise_and(cv2.bitwise_or(cv2.inRange(hsv, (0, 90, 140), (12, 255, 255)),
                                     cv2.inRange(hsv, (170, 90, 140), (179, 255, 255))), bright)
  green = cv2.bitwise_and(cv2.inRange(hsv, (38, 65, 140), (105, 255, 255)), bright)
  _, ids, stats, centers = cv2.connectedComponentsWithStats(cv2.bitwise_or(red, green), 8)
  gray = cv2.cvtColor(rgb, cv2.COLOR_RGB2GRAY)
  out = []
  for k, stat in enumerate(stats[1:], 1):
    x, y, cw, ch, area = map(int, stat)
    if not (3 <= cw <= 30 and 3 <= ch <= 25 and 5 <= area <= 400 and .55 <= cw/ch <= 1.7 and area/(cw*ch) >= .35):
      continue
    red_count = np.count_nonzero(red[y:y+ch, x:x+cw][ids[y:y+ch, x:x+cw] == k])
    if red_count >= area*.9:
      color = 'red'
    elif red_count <= area*.1:
      color = 'green'
    else:
      continue
    x += ox
    y += oy
    cx, cy = centers[k] + [ox, oy]
    v = val[y-oy:y-oy+ch, x-ox:x-ox+cw]
    # Include the saturated white center even when the colored halo is a ring.
    core = (v >= max(95, float(v.max())*.85)).astype(np.uint8)
    _, _, core_stats, core_centers = cv2.connectedComponentsWithStats(core, 8)
    index = 1 + int(np.argmax(core_stats[1:, cv2.CC_STAT_AREA]))
    _, _, bw, bh, core_area = core_stats[index]
    ccx, ccy = core_centers[index]
    if not (3 <= bw <= 18 and 3 <= bh <= 18 and .6 <= bw/bh <= 1.6 and core_area/(bw*bh) >= .55):
      continue
    if abs(ccx+x-cx) > cw*.25 or abs(ccy+y-cy) > ch*.25:
      continue
    cx, cy = ccx+x, ccy+y
    diameter = max(bw, bh)
    width, height = 4*diameter, 2*diameter
    xc = .125 if color == 'red' else .875
    box = [int(cx-width*xc), int(cy-height/2), int(cx+width*(1-xc)), int(cy+height/2)]
    x1, y1, x2, y2 = box
    if x1 < 0 or y1 < 3 or x2 > w or y2+3 > h:
      continue
    ux1, ux2 = (int(cx+diameter*.75), x2) if color == 'red' else (x1, int(cx-diameter*.75))
    uy1, uy2 = int(cy-diameter*.35), int(cy+diameter*.35)+1
    dark = gray[uy1:uy2, ux1:ux2]
    surround = np.r_[gray[y1-3:y1, ux1:ux2].ravel(), gray[y2:y2+3, ux1:ux2].ravel()]
    if not dark.size or not surround.size:
      continue
    contrast = float(np.median(surround)) - float(np.median(dark))
    if contrast < 8 or float(np.median(dark)) > 80:
      continue
    state, quality, evidence = lamp_evidence(rgb, box)
    if state == color and quality >= .25:
      out.append(dict(box=box, raw=state, quality=float(quality), source='day_color_core', contrast=contrast, **evidence))
  return out


@dataclass
class Track:
  ident: int
  box: list
  last_seen: float
  born: float
  state: str = 'unknown'
  pending: str = 'unknown'
  pending_since: float = 0.
  observations: int = 0
  pending_count: int = 0
  seen_red: bool = False
  last_support: float = 0.
  evidence: dict = field(default_factory=dict)


class SignalTracker:
  """Bounded confirmation and expiry per tracked housing; never carries state across IDs."""
  # Object continuity and output freshness are separate contracts. The worker's
  # CPU duty limit produces 150-200 ms observations on the internal-model C4.
  MAX_OBSERVATION_GAP = .25
  MAX_VISIBLE_AGE = .125

  def __init__(self, *, daytime_cores=False):
    # Keep the unsuccessful daytime experiment out of the live worker. Callers
    # must explicitly select it for offline/isolated validation.
    self.daytime_cores = daytime_cores
    self.tracks = []
    self.next_id = 1
    self.last_timestamp = None
    self.previous_gray = None

  def update(self, timestamp, detections):
    if self.last_timestamp is not None and (timestamp <= self.last_timestamp or timestamp-self.last_timestamp > self.MAX_OBSERVATION_GAP):
      self.tracks = []
    self.last_timestamp = timestamp
    self.tracks = [t for t in self.tracks if timestamp-t.last_seen <= self.MAX_OBSERVATION_GAP]
    used = set()
    for d in detections:
      candidates = []
      for t in self.tracks:
        if t.ident in used:
          continue
        a, b = t.box, d['box'];aw, ah = a[2]-a[0], a[3]-a[1];bw, bh = b[2]-b[0], b[3]-b[1]
        distance = math.hypot((a[0]+a[2]-b[0]-b[2])/2, (a[1]+a[3]-b[1]-b[3])/2)
        if .65 <= bw/aw <= 1.55 and .5 <= bh/ah <= 2 and distance < max(6, aw*.3) and overlap(a,b) > .12:
          candidates.append((distance, t))
      if candidates:
        t = min(candidates, key=lambda v:v[0])[1]
      else:
        t = Track(self.next_id, d['box'], timestamp, timestamp)
        self.next_id += 1
        self.tracks.append(t)
      used.add(t.ident)
      gap = timestamp-t.last_seen
      t.box = d['box'];t.last_seen = timestamp;t.observations += 1;t.evidence = d
      raw = d['raw']
      if raw == 'unknown':
        t.pending = 'unknown'
        t.pending_count = 0
        if timestamp-t.last_support > .15:
          t.state = 'unknown'
      elif raw != t.pending or gap > self.MAX_OBSERVATION_GAP:
        t.pending = raw;t.pending_since = timestamp
        t.pending_count = 1
        # A contradictory observation removes an earlier green immediately.
        if raw != t.state:
          t.state = 'unknown'
      else:
        t.pending_count += 1
        if timestamp-t.pending_since >= (.10 if raw == 'red' else .15)-1e-6 and t.pending_count >= 3:
          if raw == 'red':
            t.seen_red = True
          t.state = raw if raw == 'red' or t.seen_red else 'unknown'
      if raw == t.state and raw != 'unknown':
        t.last_support = timestamp
    visible = []
    for t in self.tracks:
      age = timestamp-t.last_seen
      state = t.state if age <= self.MAX_VISIBLE_AGE else 'unknown'
      visible.append(dict(id=t.ident, box=t.box, state=state, age=age, observations=t.observations, seen_red=t.seen_red,
                          support_state=t.pending, support_since=t.pending_since, support_count=t.pending_count,
                          evidence=t.evidence))
    # No ego-lane claim. Conflicting visible signals cannot become a green vote.
    reliable = [t for t in visible if t['state'] != 'unknown' and t['observations'] >= 3]
    states = {t['state'] for t in reliable}
    state = next(iter(states)) if len(states) == 1 else 'unknown'
    return dict(state=state, reason='agreeing_visible_tracks' if len(states)==1 else 'conflict_or_unconfirmed', tracks=visible)

  def process(self, rgb, timestamp):
    detections = detect(rgb, daytime_cores=self.daytime_cores)
    gray = cv2.cvtColor(rgb, cv2.COLOR_RGB2GRAY)
    if self.previous_gray is not None and self.last_timestamp is not None and 0 < timestamp-self.last_timestamp <= self.MAX_OBSERVATION_GAP:
      for track in self.tracks:
        # The template must belong to the immediately preceding image. An old
        # box alone cannot identify the object in that intervening image.
        if not track.seen_red or track.last_seen != self.last_timestamp:
          continue
        if any(overlap(track.box,d['box']) > .25 for d in detections):
          continue
        x1,y1,x2,y2 = map(int,track.box);w=x2-x1;h=y2-y1;pad=max(6,h//2)
        ax=max(0,x1-pad);ay=max(0,y1-pad);bx=min(gray.shape[1],x2+pad);by=min(gray.shape[0],y2+pad)
        template=self.previous_gray[ay:by,ax:bx]
        margin=max(10,w//5);sx=max(0,ax-margin);sy=max(0,ay-margin);ex=min(gray.shape[1],bx+margin);ey=min(gray.shape[0],by+margin)
        if template.size == 0 or ey-sy < template.shape[0] or ex-sx < template.shape[1]:
          continue
        scores=cv2.matchTemplate(gray[sy:ey,sx:ex],template,cv2.TM_CCOEFF_NORMED)
        _,score,_,loc=cv2.minMaxLoc(scores)
        if score < .75:
          continue
        dx=sx+loc[0]-ax;dy=sy+loc[1]-ay;box=[x1+dx,y1+dy,x2+dx,y2+dy]
        raw,strength,evidence=lamp_evidence(rgb,box)
        detections.append(dict(box=box,raw=raw,quality=strength,source='causal_template',match_score=score,**evidence))
    self.previous_gray=gray
    return self.update(timestamp,detections)
