"""Offline reference-point ranking. Not signal classification or control permission.

Project an assumed overhead point and rank image candidates relative to it.
Offsets in metres live on the ASSUMED reference plane, not at a measured signal
range. Low-confidence road predictions remain explicitly diagnostic only.
"""
from dataclasses import dataclass
import math

import numpy as np

from openpilot.common.transformations.orientation import rot_from_euler


@dataclass(frozen=True)
class ReferencePoint:
  xyz: tuple
  pixel: tuple
  source: str
  reliable: bool


def project_reference(model, rpy, *, distance=30., height=5., source='lanes', focal=1141.5):
  """No extrapolation; Z points down, so overhead height is SUBTRACTED."""
  try:
    if source not in ('lanes', 'edges') or len(rpy) != 3:
      return None
    if not all(math.isfinite(v) for v in (*rpy, distance, height, focal)) or min(distance, height, focal) <= 0:
      return None
    curves = list(model.laneLines)[1:3] if source == 'lanes' else list(model.roadEdges)
    if len(curves) != 2:
      return None
    points = []
    for c in curves:
      x, y, z = [np.asarray(getattr(c, axis), dtype=float) for axis in ('x', 'y', 'z')]
      if (x.ndim != 1 or len(x) < 2 or y.shape != x.shape or z.shape != x.shape
          or not all(np.isfinite(a).all() for a in (x, y, z)) or np.any(np.diff(x) <= 0)
          or not x[0] <= distance <= x[-1]):
        return None
      points.append((float(np.interp(distance, x, y)), float(np.interp(distance, x, z))))
    width = points[1][0]-points[0][0]
    if not 2 <= width <= (5 if source == 'lanes' else 20):
      return None
    y, z = np.mean(points, axis=0)
    xyz = (distance, float(y), float(z-height))
    camera = rot_from_euler(rpy) @ xyz
    if camera[0] <= 0:
      return None
    pixel = tuple(float(v) for v in np.array([672., 380.])+focal*camera[1:]/camera[0])
    if source == 'lanes':
      stds = list(model.laneLineStds)[1:3]
      probs = list(model.laneLineProbs)[1:3]
      reliable = len(probs) == 2 and all(math.isfinite(p) and .8 <= p <= 1 for p in probs)
    else:
      stds = list(model.roadEdgeStds)
      reliable = True
    reliable = bool(reliable and len(stds) == 2 and all(math.isfinite(s) and 0 <= s <= .5 for s in stds))
    return ReferencePoint(xyz, pixel, source, reliable)
  except (AttributeError, TypeError, ValueError, IndexError):
    return None


def rank_candidates(reference, tracks, rpy, *, focal=1141.5, max_offset=3.):
  """Use current boxes; never choose a stale retained display box.

  `near_reference` is only geometric plausibility. Neither it nor ranking grants
  a red/green vote, red history, signal identity, or lane association.
  """
  if reference is None:
    return []
  if (len(rpy) != 3 or not all(math.isfinite(v) for v in (*rpy, focal, max_offset))
      or focal <= 0 or max_offset <= 0):
    return []
  rotation = rot_from_euler(rpy).T
  ranked = []
  for t in tracks:
    try:
      x1, y1, x2, y2 = t['box']
      if (t['age'] != 0 or not all(math.isfinite(v) for v in (x1,y1,x2,y2))
          or not 0 <= x1 < x2 <= 1344 or not 0 <= y1 < y2 <= 760):
        continue
      center = np.array([(x1+x2)/2, (y1+y2)/2])
      ray = rotation @ np.r_[1., (center-np.array([672.,380.]))/focal]
      if ray[0] <= 0:
        continue
      projected = ray[1:] * (reference.xyz[0]/ray[0])
      offset = float(np.linalg.norm(projected-np.asarray(reference.xyz[1:])))
      ranked.append(dict(id=t['id'], box=list(t['box']), pixel_distance=float(np.linalg.norm(center-reference.pixel)),
                         reference_plane_offset=offset, near_reference=offset <= max_offset,
                         geometry_reliable=reference.reliable))
    except (KeyError, TypeError, ValueError):
      continue
  return sorted(ranked, key=lambda t: t['reference_plane_offset'])
