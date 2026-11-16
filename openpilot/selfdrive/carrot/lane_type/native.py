"""Observe original camera pixels along a projected boundary, without stretching.

Offline experimental helper. No call from the production daemon or CAN path.
Each returned profile sample has a distinct original image row and a physical
distance. Occlusion/low-resolution/ambiguous paint is UNKNOWN, never a gap.
"""
import numpy as np

from openpilot.selfdrive.carrot.lane_type.dual_roi import project, visible_rows


def native_observation(rgb, line, probability, rpy, k, leads=()):
  import cv2
  empty = {'heads': {}, 'profile': [], 'reason': 'unobservable', 'paddingUsed': False}
  if probability < .5 or not np.isfinite(probability):
    return empty
  distances = np.arange(4.5, 25.001, .025)
  try:
    maps = project(line, rpy, k, distances)
    clear = visible_rows(maps, rgb.shape, leads, line, rpy, k)
  except ValueError:
    return empty
  # Monotonic road projection is necessary for pixel-row/depth inversion.
  center_y = maps[1][:, 32]
  if np.any(np.diff(center_y) >= 0):
    return dict(empty, reason='nonmonotonic_road_projection')
  ys = np.arange(int(np.ceil(center_y[-1])), int(np.floor(center_y[0]))+1)
  ds = np.interp(ys, center_y[::-1], distances[::-1])
  left = np.interp(ds, distances, maps[0][:, 0])
  right = np.interp(ds, distances, maps[0][:, -1])
  valid = np.interp(ds, distances, clear.astype(float)) > .999
  hsv = cv2.cvtColor(rgb, cv2.COLOR_RGB2HSV)
  gray = cv2.cvtColor(rgb, cv2.COLOR_RGB2GRAY)
  observations, patches = [], []
  previous_position = None
  for y, d, lo, hi, ok in zip(ys, ds, left, right, valid, strict=True):
    a, b = int(np.ceil(lo)), int(np.floor(hi))
    record = {'distance': float(d), 'imageRow': int(y), 'ink': None, 'color': None, 'confidence': 0.,
              'centerOffsetM': None, 'sourceWidthPx': max(0, b-a+1), 'reason': 'occluded_or_FOV'}
    observations.append(record)
    patches.append(None)
    if not ok or a < 0 or b >= rgb.shape[1] or b-a+1 < 16:
      continue
    road = float(np.median(np.r_[gray[y, a:a+max(2, (b-a)//8)], gray[y, b-max(2, (b-a)//8):b+1]]))
    values = hsv[y, a:b+1]
    contrast = gray[y, a:b+1].astype(float)-road
    yellow = (values[:, 0] >= 16) & (values[:, 0] <= 42) & (values[:, 1] >= 75) & (values[:, 2] >= 65) & (contrast >= 8)
    white = (values[:, 1] <= 65) & (values[:, 2] >= 100) & (contrast >= 14)
    mask = yellow | white
    bounds = np.flatnonzero(np.diff(np.r_[False, mask, False])).reshape(-1, 2)
    options, broad = [], False
    for start, end in bounds:
      width = (end-start)*.9/(b-a+1)
      offset = ((start+end-1)/2-(b-a)/2)*.9/(b-a+1)
      broad |= width > .38
      if end-start >= 2 and .035 <= width <= .30 and abs(offset) <= .30:
        continuity = abs(offset-previous_position) if previous_position is not None else abs(offset)
        options.append((continuity+.25*abs(offset), start, end, offset))
    if options and not broad:
      _, start, end, offset = min(options)
      yc, wc = int(yellow[start:end].sum()), int(white[start:end].sum())
      color = 'yellow' if yc > wc else 'white'
      dominance = max(yc, wc)/(yc+wc)
      record.update(ink=1, color=color if dominance >= .95 else None, confidence=float(dominance),
                    centerOffsetM=float(offset), reason='original_pixel_paint_ridge')
      previous_position = offset
    elif not broad and road >= 45 and road <= 175 and np.std(gray[y, a:b+1]) <= 22 and not mask.any():
      # Absence is usable only on clearly exposed road; this remains a heuristic
      # requiring image audit, not a segmentation ground truth certificate.
      record.update(ink=0, confidence=.85, reason='exposed_road_candidate')
    else:
      record['reason'] = 'transverse_shadow_texture_or_unresolved_paint'
    record['roadLuma'] = road
    # Raw source pixels: all selected x coordinates are distinct, no remap.
    center = (a+b)/2+(record['centerOffsetM'] or 0.)*(b-a+1)/.9
    half = min((b-a)/2, (rgb.shape[1]-1-center), center)
    xs = np.rint(np.linspace(center-half, center+half, 16)).astype(int)
    if len(np.unique(xs)) == 16:
      patches[-1] = rgb[y, xs].copy()
  profile = list(reversed(observations))  # near -> far
  patches = list(reversed(patches))
  # A single physical contiguous interval. An unresolved paint row may remain
  # in the image, but its profile state is UNKNOWN and contributes no gap vote.
  valid_geometry = np.array([p is not None for p in patches])
  usable = np.flatnonzero(valid_geometry)
  if not len(usable):
    return dict(empty, profile=profile)
  start = usable[0]
  stop = start
  while stop+1 < len(profile) and valid_geometry[stop+1]:
    stop += 1
  heads = {}
  for head, end, count in [('color', 12., 32), ('type', 20., 64)]:
    indices = [i for i in range(start, stop+1) if profile[i]['distance'] <= end]
    extensions = (15.,) if head == 'color' else (25.,)
    for extension in extensions:
      painted = sum(profile[i]['ink'] == 1 for i in indices)
      # Seek only when native row count/paint/repeated-gap evidence is lacking.
      # Never extend colour past 15m or type past 25m.
      if len(indices) < count or painted < 5 or (head == 'type' and _runs([profile[i] for i in indices])['paint_runs'] < 3):
        candidate = [i for i in range(start, stop+1) if profile[i]['distance'] <= extension]
        if len(candidate) >= count:
          indices = candidate
    if len(indices) < count:
      continue
    selected = np.rint(np.linspace(indices[0], indices[-1], count)).astype(int)
    assert len(np.unique(selected)) == count
    image = np.stack([patches[i] for i in selected])
    # Bottom row is near, matching existing strip orientation; no interpolation.
    image = image[::-1].copy()
    ps = [profile[i] for i in indices]
    evidence = _runs(ps)
    colors = [r['color'] for r in ps if r['ink'] == 1 and r['color']]
    dominance = max(colors.count('white'), colors.count('yellow'))/len(colors) if colors else 0.
    evidence.update(color=('yellow' if colors.count('yellow') > colors.count('white') else 'white') if dominance >= .97 and len(colors) >= 5 else 'unknown',
                    colorDominance=dominance, nativePaintRows=len(colors))
    heads[head] = {'image': image, 'near': ps[0]['distance'], 'far': ps[-1]['distance'], 'evidence': evidence,
      'nativeRowsAvailable': len(indices), 'uniqueSourceRows': count, 'uniqueSourceColumns': 16,
      'interpolation': 'none; distinct original RGB pixels', 'paddingUsed': False,
      'visibilityValidated': False, 'observableFraction': sum(r['ink'] is not None for r in ps)/len(ps),
      'centerCorrectionM': float(np.median([r['centerOffsetM'] for r in ps if r['centerOffsetM'] is not None])) if colors else None}
  return {'heads': heads, 'profile': profile, 'reason': 'native_observation', 'paddingUsed': False}


def _runs(profile):
  """Distance-based paint/gap evidence; UNKNOWN breaks the pattern."""
  runs = []
  for row in profile:
    value = row['ink']
    if runs and runs[-1][0] == value:
      runs[-1][2] = row['distance']
    else:
      runs.append([value, row['distance'], row['distance']])
  paint = [b-a for ink, a, b in runs if ink == 1 and b-a >= .6]
  gaps = [b-a for j, (ink, a, b) in enumerate(runs) if ink == 0 and b-a >= 1.2 and
          j > 0 and j+1 < len(runs) and runs[j-1][0] == runs[j+1][0] == 1]
  span = profile[-1]['distance']-profile[0]['distance'] if len(profile) > 1 else 0.
  known = sum(r['ink'] is not None for r in profile)/max(1, len(profile))
  coverage = sum(r['ink'] == 1 for r in profile)/max(1, len(profile))
  # Frame-level solid evidence uses physical span. No gap in a short crop is
  # never enough. Temporal fusion can supply the missing observed distance.
  kind = 'dashed' if len(paint) >= 3 and len(gaps) >= 2 and known >= .9 else 'unknown'
  if span >= 14 and known >= .97 and coverage >= .97:
    kind = 'solid'
  return {'type': kind, 'repeatingPatternObserved': kind == 'dashed', 'paint_runs': len(paint),
          'paintLengthsM': paint, 'internalGapLengthsM': gaps, 'observedSpanM': span,
          'knownFraction': known, 'coverage': coverage, 'confidenceKind': 'uncalibrated_native_pixel_evidence'}
