"""Experimental separate colour/type observations. Never substitutes missing pixels.

Existing 6--46m runtime and bundled model are unchanged. These helpers are used
by the offline comparison until visibility and device costs are qualified.
"""
import numpy as np

from openpilot.common.transformations.camera import view_frame_from_device_frame
from openpilot.common.transformations.orientation import rot_from_euler
from openpilot.selfdrive.carrot.lane_type.roi import rgb_strip

START_M = 4.5
COLOR_END_M = 15.
TYPE_END_M = 30.
SEARCH_END_M = 35.


def project(line, rpy, k, distances, offset=0., half_width=.45, columns=64):
  xyz = np.asarray([line['x'], line['y'], line['z']], dtype=float)
  if (xyz.ndim != 2 or xyz.shape[0] != 3 or xyz.shape[1] < 3 or not np.isfinite(xyz).all() or
      np.any(np.diff(xyz[0]) <= 0) or xyz[0, 0] > distances.min() or xyz[0, -1] < distances.max()):
    raise ValueError('unavailable or invalid lane geometry')
  points = np.stack((np.broadcast_to(distances[:, None], (len(distances), columns)),
    np.interp(distances, xyz[0], xyz[1])[:, None]+offset+np.linspace(-half_width, half_width, columns),
    np.broadcast_to(np.interp(distances, xyz[0], xyz[2])[:, None], (len(distances), columns))), -1)
  pixels = points @ (np.asarray(k) @ view_frame_from_device_frame @ rot_from_euler(rpy)).T
  if (pixels[..., 2] <= 0).any():
    raise ValueError('lane behind camera')
  return (pixels[..., 0]/pixels[..., 2]).astype(np.float32), (pixels[..., 1]/pixels[..., 2]).astype(np.float32)


def visible_rows(maps, image_shape, leads, line, rpy, k, hood_fraction=.86):
  """Conservative, explicitly heuristic hood/lead exclusions; no object detector."""
  h, w = image_shape[:2]
  mx, my = maps
  clear = np.isfinite(mx+my) & (mx >= 1) & (mx < w-2) & (my >= 1) & (my < h*hood_fraction)
  projection = np.asarray(k) @ view_frame_from_device_frame @ rot_from_euler(rpy)
  for lead in leads:
    if lead['prob'] < .5 or not lead['x'] or not lead['y']:
      continue
    x, y = lead['x'][0], lead['y'][0]
    if not np.isfinite(x+y) or not 2 < x < 80:
      continue
    z = np.interp(x, line['x'], line['z'])
    box = np.array([[x, y+dy, z+dz] for dy in (-1.3, 1.3) for dz in (-2., .2)]) @ projection.T
    if (box[:, 2] <= 0).any():
      continue
    p = box[:, :2]/box[:, 2, None]
    lo, hi = p.min(0), p.max(0)
    clear &= ~((mx >= lo[0]) & (mx <= hi[0]) & (my >= lo[1]) & (my <= hi[1]))
  return clear.all(1)


def interval(distances, clear, end):
  """First contiguous observable interval after 4.5m; never concatenate gaps."""
  usable = np.flatnonzero(clear & (distances <= end))
  if not len(usable):
    return None
  start = usable[0]
  stop = start
  while stop+1 < len(distances) and clear[stop+1] and distances[stop+1] <= end:
    stop += 1
  if distances[stop]-distances[start] < 2.:
    return None
  return float(distances[start]), float(distances[stop])


def paint_evidence(rgb, near, far):
  """Independent pixel/ridge evidence, not the legacy OpenCV classifier's label.

  Narrow coloured ridges are followed near the model centre. Broad transverse
  paint is an ambiguity, and dark road intervals cannot establish dashed gaps.
  Scores are uncalibrated evidence. Temporal/audit gates live in PC tooling.
  """
  import cv2
  hsv = cv2.cvtColor(rgb, cv2.COLOR_RGB2HSV)
  gray = cv2.cvtColor(rgb, cv2.COLOR_RGB2GRAY).astype(float)
  h, w = gray.shape
  road = np.median(np.concatenate((gray[:, :8], gray[:, -8:]), 1), 1)
  contrast = gray-road[:, None]
  yellow = (hsv[..., 0] >= 16) & (hsv[..., 0] <= 42) & (hsv[..., 1] >= 75) & (hsv[..., 2] >= 65) & (contrast >= 8)
  white = (hsv[..., 1] <= 65) & (hsv[..., 2] >= 100) & (contrast >= 14)
  paint = yellow | white
  positions = np.full(h, np.nan)
  selected = np.zeros_like(paint)
  broad = np.zeros(h, bool)
  for i, mask in enumerate(paint):
    boundaries = np.flatnonzero(np.diff(np.r_[False, mask, False]))
    options = []
    for a, b in boundaries.reshape(-1, 2):
      width = (b-a)*.9/w
      if width > .38:
        broad[i] = True
      if .035 <= width <= .30 and abs((a+b-1)/2-(w-1)/2)*.9/w <= .30:
        options.append((abs((a+b-1)/2-(w-1)/2), a, b))
    if options:
      _, a, b = min(options)
      positions[i] = (a+b-1)/2
      selected[i, a:b] = True
  present = np.isfinite(positions)
  # Fill tiny sampling holes only, never a metre-scale paint gap.
  filled = cv2.morphologyEx(present.astype(np.uint8)[:, None], cv2.MORPH_CLOSE, np.ones((3, 1), np.uint8)).ravel().astype(bool)
  cuts = np.r_[0, np.flatnonzero(filled[1:] != filled[:-1])+1, h]
  runs = [(bool(filled[a]), float((b-a)*(far-near)/h), int(a), int(b)) for a, b in zip(cuts[:-1], cuts[1:], strict=True)]
  segments = [length for ink, length, _, _ in runs if ink and length >= .8]
  gaps = [(length, a, b) for j, (ink, length, a, b) in enumerate(runs)
          if not ink and 0 < j < len(runs)-1 and length >= 1.3]
  yc, wc = int((selected & yellow).sum()), int((selected & white).sum())
  total = yc+wc
  dominance = max(yc, wc)/total if total else 0.
  color = ('yellow' if yc > wc else 'white') if total >= 80 and dominance >= .97 else 'unknown'
  offset = float(np.nanmedian(positions)-(w-1)/2)*.9/w if present.any() else None
  jitter = float(np.nanmedian(np.abs(positions-np.nanmedian(positions))))*.9/w if present.any() else None
  stable_ridge = present.sum() >= 12 and jitter is not None and jitter <= .07 and abs(offset) <= .28
  road_cv = float(np.std(road)/max(np.mean(road), 1))
  shadow_safe = road_cv <= .22 and all(np.median(road[a:b]) >= .75*np.median(road[filled]) for _, a, b in gaps)
  # Broad crosswalk/guide paint invalidates auto type, not every complex scene.
  clean = float(broad.mean()) < .035 and stable_ridge
  kind = 'unknown'
  repeat = clean and shadow_safe and len(segments) >= 3 and len(gaps) >= 2 and all(.8 <= x <= 8 for x in segments)
  if repeat and all(1.3 <= g[0] <= 10 for g in gaps) and .12 <= filled.mean() <= .78:
    kind = 'dashed'
  elif clean and far-near >= 20 and filled.mean() >= .94 and max((x[1] for x in runs if not x[0]), default=0) <= .6:
    kind = 'solid'
  return {'type': kind, 'color': color if stable_ridge else 'unknown', 'colorDominance': dominance,
    'paintPixels': total, 'paintRows': int(present.sum()), 'coverage': float(filled.mean()),
    'centerOffsetM': offset, 'centerJitterM': jitter, 'broadPaintFraction': float(broad.mean()),
    'roadLumaCV': road_cv, 'shadowSafe': bool(shadow_safe), 'repeatingPatternObserved': bool(repeat),
    'paintLengthsM': segments, 'internalGapLengthsM': [g[0] for g in gaps],
    'scoreKind': 'uncalibrated_independent_pixel_evidence'}


def extract_pair(rgb, line, probability, rpy, k, leads=()):
  if not np.isfinite(probability) or probability < .5:
    return {'reason': 'low_model_probability', 'heads': {}}
  distances = np.arange(START_M, SEARCH_END_M+.01, .1)
  try:
    probe = project(line, rpy, k, distances)
    clear = visible_rows(probe, rgb.shape, leads, line, rpy, k)
  except ValueError:
    return {'reason': 'invalid_lane_geometry', 'heads': {}}
  heads = {}
  for head, end in [('color', COLOR_END_M), ('type', TYPE_END_M)]:
    bounds = interval(distances, clear, end)
    if bounds is None:
      continue
    near, far = bounds
    ds = np.linspace(far, near, 192)
    maps = project(line, rpy, k, ds)
    strip = rgb_strip(rgb, maps)
    evidence = paint_evidence(strip, near, far)
    # Robust constant correction only. No arbitrary per-row warp follows shadows.
    offset = evidence['centerOffsetM']
    if offset is not None and abs(offset) <= .28 and evidence['centerJitterM'] <= .07:
      corrected = project(line, rpy, k, ds, offset=offset)
      if visible_rows(corrected, rgb.shape, leads, line, rpy, k).all():
        maps = corrected
        strip = rgb_strip(rgb, maps)
        evidence = paint_evidence(strip, near, far)
      else:
        offset = 0.
    else:
      offset = 0.
    if head == 'color' and evidence['color'] == 'unknown':
      # A near dashed gap can contain no paint. Seek the next observable paint
      # only as far as 25m; colour never inherits a label from the type head.
      for limit in (20., 25.):
        extended = interval(distances, clear, limit)
        if not extended or extended[1] <= far+1:
          continue
        ds2 = np.linspace(extended[1], near, 192)
        m2 = project(line, rpy, k, ds2, offset=offset)
        if visible_rows(m2, rgb.shape, leads, line, rpy, k).all():
          s2 = rgb_strip(rgb, m2)
          e2 = paint_evidence(s2, near, extended[1])
          if e2['color'] != 'unknown':
            far, ds, maps, strip, evidence = extended[1], ds2, m2, s2, e2
            break
    # Explore only the type head if no repeat/solid evidence at 30m. Resolution
    # must remain measurable; 35m is a hard cap, not an automatic extra crop.
    if head == 'type' and evidence['type'] == 'unknown' and far >= end-.11:
      extended = interval(distances, clear, SEARCH_END_M)
      if extended and extended[1] > far+2:
        ds2 = np.linspace(extended[1], near, 192)
        m2 = project(line, rpy, k, ds2, offset=offset)
        native_step = np.median(np.hypot(np.diff(m2[0][:, 32]), np.diff(m2[1][:, 32]))[:64])
        if native_step >= .30 and visible_rows(m2, rgb.shape, leads, line, rpy, k).all():
          s2 = rgb_strip(rgb, m2)
          e2 = paint_evidence(s2, near, extended[1])
          if e2['type'] != 'unknown':
            far, ds, maps, strip, evidence = extended[1], ds2, m2, s2, e2
    heads[head] = {'image': strip, 'maps': maps, 'near': near, 'far': far, 'centerCorrectionM': offset,
      'evidence': evidence, 'observedLengthM': far-near, 'visibilityHeuristic': 'FOV_bottom14percent_model_leads_v1',
      'visibilityValidated': False, 'paddingUsed': False}
  return {'reason': 'observable_heads' if heads else 'unobservable', 'heads': heads}
