#!/usr/bin/env python3
"""Per model frame in a replay window: the position mapsd would use (carrotMan's fused
fix, as openpilot_navd::position) and the navRoute geometry active then, for
`cargo run --example replay_maps` of openpilot-mapsd.
usage: positions.py 'DIR/{seg}/rlog.zst' OUT.json START_SEG:SEC END_SEG:SEC [--full]
(--full: frames indexed by roadEncodeIdx, as replay.py --fcam; default qRoadEncodeIdx)"""
import os, sys, json, bisect, capnp, zstandard
pattern, out = sys.argv[1], sys.argv[2]
s0, t0 = sys.argv[3].split(':'); s1, t1 = sys.argv[4].split(':')
s0, t0, s1, t1 = int(s0), float(t0), int(s1), float(t1)
index = 'roadEncodeIdx' if '--full' in sys.argv else 'qRoadEncodeIdx'
REPO = os.path.abspath(os.path.join(os.path.dirname(__file__), '../../..'))
capnp.remove_import_hook()
log = capnp.load(REPO + '/openpilot/cereal/log.capnp', imports=[REPO + '/openpilot/cereal', REPO + '/opendbc_repo/opendbc/car', REPO])
T0 = None; routes = []; route_t = []; pos = []; frames = []; idx = set()
for s in range(0, s1 + 1):
  raw = zstandard.ZstdDecompressor().stream_reader(open(pattern.format(seg=s), 'rb')).read()
  try:
    for e in log.Event.read_multiple_bytes(raw):
      t = e.logMonoTime / 1e9
      if T0 is None: T0 = t
      tt = t - T0; w = e.which()
      if w == 'navRoute' and e.valid:
        c = [[p.longitude, p.latitude] for p in e.navRoute.coordinates]
        if c and (not routes or c != routes[-1]): routes.append(c); route_t.append(tt)
      elif w == 'carrotMan':
        c = e.carrotMan
        if c.xPosLat != 0: pos.append((tt, c.xPosLat, c.xPosLon, c.xPosAngle, c.xPosSpeed / 3.6))
      elif w == index: idx.add(getattr(e, index).frameId)
      elif w == 'modelV2' and s0 * 60 + t0 <= tt <= s1 * 60 + t1: frames.append((tt, e.modelV2.frameId))
  except Exception: pass
pt = [p[0] for p in pos]
rows = []
for tt, f in frames:
  if f not in idx: continue
  i = bisect.bisect_right(pt, tt) - 1; r = bisect.bisect_right(route_t, tt) - 1
  if i < 0 or r < 0: continue
  _, lat, lon, ang, v = pos[i]
  rows.append(dict(frame=f, t=tt, lat=lat, lon=lon, bearing=ang, advance=v * (tt - pt[i]), route=r))
json.dump(dict(routes=routes, frames=rows), open(out, 'w'))
print('routes', len(routes), 'frames', len(rows))
