#!/usr/bin/env python3
"""Replay part of a drive through N16 on the PC with the inputs the comma sent, and
keep the tablet's logged output next to it for comparison (compare.py).

ONNX Runtime CPU on N16's fp16 graph takes minutes per frame; convert it once with
to_fp32.py (about 0.25 s/frame on 12 threads).

Video, either
  --qcam 'DIR/{seg}/qcamera.ts'          road camera only, 526x330. The wide input is the
                                         road camera warped into the wide frame (outer part
                                         black): fine for straight road, not for turns.
  --fcam '.../{seg}/fcamera.hevc' --ecam '.../{seg}/ecamera.hevc'   full road + wide video.
Route map: black (default, what every drive got before mapsd ef6c0f1f3) or --maps from
mapsd's replay_maps example. {seg} in patterns is the segment number.

usage: replay.py --rlog 'DIR/{seg}/rlog.zst' --start SEG:SEC --end SEG:SEC --model n16_fp32.onnx
                 (--qcam P | --fcam P --ecam P) [--maps maps.bin] OUT.npz
"""
import argparse, bisect, math, os, time
import numpy as np, av, capnp, zstandard, onnxruntime as ort

REPO = os.path.abspath(os.path.join(os.path.dirname(__file__), '../../..'))
ap = argparse.ArgumentParser()
ap.add_argument('out')
ap.add_argument('--rlog', required=True)
ap.add_argument('--start', required=True); ap.add_argument('--end', required=True)
ap.add_argument('--model', required=True)
ap.add_argument('--qcam'); ap.add_argument('--fcam'); ap.add_argument('--ecam')
ap.add_argument('--maps')
ap.add_argument('--lat-smooth', type=float, default=0.06, help='LatSmoothSec / 100')
ap.add_argument('--long-delay', type=float, default=0.30, help='LongActuatorDelay / 100')
ap.add_argument('--threads', type=int, default=12)
a = ap.parse_args()
assert a.qcam or (a.fcam and a.ecam), 'need --qcam or --fcam and --ecam'
s0, t0 = (float(x) for x in a.start.split(':')); s1, t1 = (float(x) for x in a.end.split(':'))
s0, s1 = int(s0), int(s1)

capnp.remove_import_hook()
C = REPO + '/openpilot/cereal'
log = capnp.load(C + '/log.capnp', imports=[C, REPO + '/opendbc_repo/opendbc/car', REPO])

# ---------- geometry: openpilot common/transformations ----------
def rot_from_euler(rpy):
  r, p, y = rpy
  Rx = np.array([[1, 0, 0], [0, math.cos(r), -math.sin(r)], [0, math.sin(r), math.cos(r)]])
  Ry = np.array([[math.cos(p), 0, math.sin(p)], [0, 1, 0], [-math.sin(p), 0, math.cos(p)]])
  Rz = np.array([[math.cos(y), -math.sin(y), 0], [math.sin(y), math.cos(y), 0], [0, 0, 1]])
  return Rz @ Ry @ Rx
VIEW_FROM_DEVICE = np.array([[0., 0., 1.], [1., 0., 0.], [0., 1., 0.]]).T
CY = 47.6
MED = np.array([[910., 0, 256.], [0, 910., CY], [0, 0, 1]])
SBIG = np.array([[455., 0, 256.], [0, 455., 0.5 * (256 + CY)], [0, 0, 1]])
CAMERAS = {  # sensor: (width, height, road focal, wide focal)
  'ar0231': (1928, 1208, 2648., 567.), 'ox03c10': (1928, 1208, 2648., 567.), 'unknown': (1928, 1208, 2648., 567.),
  'os04c10': (1344, 760, 1522. * 3 / 4, 567. / 4 * 3)}
def intrinsics(w, h, f): return np.array([[f, 0, w / 2], [0, f, h / 2], [0, 0, 1]])
def warp_matrix(rpy, K_cam, K_model):
  return K_cam @ VIEW_FROM_DEVICE @ rot_from_euler(rpy) @ np.linalg.inv(K_model @ VIEW_FROM_DEVICE)

class Sampler:
  """Bilinear lookup of model pixels (w x h) in a source plane (sw x sh); black outside."""
  def __init__(self, M, w, h, sw, sh):
    u, v = np.meshgrid(np.arange(w, dtype=np.float64), np.arange(h, dtype=np.float64))
    p = M @ np.stack([u.ravel(), v.ravel(), np.ones(u.size)])
    x, y = p[0] / p[2], p[1] / p[2]
    self.inside = (x >= 0) & (x <= sw - 1) & (y >= 0) & (y <= sh - 1)
    self.x0 = np.clip(np.floor(x), 0, sw - 2).astype(np.int64); self.y0 = np.clip(np.floor(y), 0, sh - 2).astype(np.int64)
    self.fx = np.clip(x - self.x0, 0, 1); self.fy = np.clip(y - self.y0, 0, 1); self.shape = (h, w)
  def __call__(self, img):
    i = img.astype(np.float32); x0, y0, fx, fy = self.x0, self.y0, self.fx, self.fy
    v = i[y0, x0] * (1 - fx) * (1 - fy) + i[y0, x0 + 1] * fx * (1 - fy) + i[y0 + 1, x0] * (1 - fx) * fy + i[y0 + 1, x0 + 1] * fx * fy
    return np.round(np.where(self.inside, v, 0.)).reshape(self.shape).astype(np.uint8)
def half(M): return np.diag([.5, .5, 1.]) @ M @ np.diag([2., 2., 1.])
def model_input(yuv, samplers):
  Y, U, V = (s(p) for s, p in zip(samplers, yuv))
  return np.stack([Y[0::2, 0::2], Y[0::2, 1::2], Y[1::2, 0::2], Y[1::2, 1::2], U, V])

class Video:
  """Sequential decoder: frames are asked for in increasing order per segment."""
  def __init__(self, pattern): self.pattern, self.seg, self.it, self.i, self.frame = pattern, None, None, -1, None
  def get(self, seg, idx):
    if seg != self.seg or idx < self.i:
      self.seg, self.it, self.i = seg, av.open(self.pattern.format(seg=seg)).decode(video=0), -1
    while self.i < idx:
      self.frame = next(self.it); self.i += 1
    f = self.frame.reformat(format='yuv420p'); out = []
    for pl in f.planes:
      out.append(np.frombuffer(pl, np.uint8).reshape(-1, pl.line_size)[:pl.height, :pl.width])
    return out

# ---------- navigation input: carrot selfdrive/navigation/helpers.py ----------
SIDED = {'depart': (1, 2, 3), 'arrive': (4, 5, 6), 'on ramp': (17, 18, 19), 'off ramp': (20, 20, 21), 'fork': (22, 23, 24),
         'roundabout': (26, 26, 26), 'rotary': (26, 26, 26), 'roundabout turn': (26, 26, 26),
         'exit roundabout': (27, 27, 27), 'exit rotary': (27, 27, 27), 'notification': (28, 28, 28)}
TURN = {'straight': 8, 'slight right': 9, 'right': 10, 'sharp right': 11, 'uturn': 13, 'sharp left': 14, 'left': 15, 'slight left': 16}
def valhalla(t, m):
  if t not in {*SIDED, 'turn', 'continue', 'new name', 'end of road'}: return None
  if t in SIDED: s, r, l = SIDED[t]; return r if 'right' in m else l if 'left' in m else s
  return TURN.get(m, 8)
def navigation_input(maneuvers, traveled):
  n = np.zeros(47, np.float32)
  for dist, t, m in maneuvers:
    c, d = valhalla(t, m), dist - traveled
    if c is not None and c > 6 and math.isfinite(d) and d >= 0:
      n[c], n[-1] = 1, d / 100
      break
  return n

# ---------- rlog ----------
def events(seg):
  raw = zstandard.ZstdDecompressor().stream_reader(open(a.rlog.format(seg=seg), 'rb')).read()
  try:
    yield from log.Event.read_multiple_bytes(raw)
  except Exception:
    pass  # a truncated last segment
full = bool(a.fcam)
road_idx, wide_idx = {}, {}
nav, car, desire_log, cal, md = [], [], [], [], []
sensor, lat_delay, T0 = 'unknown', 0.2, None
for s in range(0, s1 + 1):
  for e in events(s):
    t = e.logMonoTime / 1e9
    T0 = t if T0 is None else T0
    if s < s0: break  # only the route start time from earlier segments
    w, tt = e.which(), t - T0
    if w == ('roadEncodeIdx' if full else 'qRoadEncodeIdx'): x = getattr(e, w); road_idx[x.frameId] = (x.segmentNum, x.segmentId)
    elif w == 'wideRoadEncodeIdx': x = e.wideRoadEncodeIdx; wide_idx[x.frameId] = (x.segmentNum, x.segmentId)
    elif w == 'navInstructionCarrot':
      nav.append((tt, e.valid, [(m.distance, str(m.type), str(m.modifier)) for m in e.navInstructionCarrot.allManeuvers]))
    elif w == 'carState': car.append((tt, e.carState.vEgo))
    elif w == 'lateralPlan': desire_log.append((tt, e.lateralPlan.desire.raw))
    elif w == 'liveCalibration': cal.append((tt, list(e.liveCalibration.rpyCalib)))
    elif w == 'liveDelay': lat_delay = e.liveDelay.lateralDelay
    elif w == 'roadCameraState': sensor = str(e.roadCameraState.sensor)
    elif w == 'modelV2':
      m = e.modelV2
      md.append(dict(t=tt, frame=m.frameId, curv=m.action.desiredCurvature, y=list(m.position.y), x=list(m.position.x),
                     ds=list(m.meta.desireState), lane=list(m.laneLines[1].y), vx=list(m.velocity.x)))
md = [r for r in md if s0 * 60 + t0 <= r['t'] <= s1 * 60 + t1 and r['frame'] in road_idx]
def latest(series, t):
  i = bisect.bisect_right([x[0] for x in series], t) - 1
  return series[i] if i >= 0 else None
print(f'{len(md)} frames, t {md[0]["t"]:.1f}-{md[-1]["t"]:.1f}, sensor {sensor}, lateral delay {lat_delay:.3f}', flush=True)

# ---------- warps (calibration taken at the window start) ----------
rpy = latest(cal, md[0]['t'])[1]
W, H, f_road, f_wide = CAMERAS.get(sensor, CAMERAS['unknown'])
if full:
  K_road, K_wide, size = intrinsics(W, H, f_road), intrinsics(W, H, f_wide), (W, H)
  road_video, wide_video = Video(a.fcam), Video(a.ecam)
else:
  q = 526 / W
  K_road = np.diag([q, q, 1.]) @ intrinsics(W, H, f_road); K_wide, size = K_road, (526, 330)
  road_video = wide_video = Video(a.qcam)
def samplers(M):
  sw, sh = size
  return (Sampler(M, 512, 256, sw, sh), Sampler(half(M), 256, 128, sw // 2, sh // 2), Sampler(half(M), 256, 128, sw // 2, sh // 2))
road_s, wide_s = samplers(warp_matrix(rpy, K_road, MED)), samplers(warp_matrix(rpy, K_wide, SBIG))

maps = {}
if a.maps:
  raw = np.fromfile(a.maps, np.uint8).reshape(-1, 5 + 65536)
  ids = raw[:, :4].copy().view('<u4')[:, 0]
  maps = {int(f): raw[i, 5:] for i, f in enumerate(ids) if raw[i, 4] == 1}
so = ort.SessionOptions(); so.intra_op_num_threads = a.threads
sess = ort.InferenceSession(a.model, so, providers=['CPUExecutionProvider'])
ins = {i.name: i for i in sess.get_inputs()}; out_names = [o.name for o in sess.get_outputs()]
def dtype(name): return np.float16 if 'float16' in ins[name].type else np.float32
state = {n: np.zeros(ins[n].shape, dtype(n)) for n in ins if n.startswith('state_')}
prev_desire = np.zeros(8, np.float32)
action_t = np.array([[lat_delay + a.lat_smooth + 0.075, a.long_delay + 0.075]], np.float32)
outs, navs, started = [], [], time.time()
for k, r in enumerate(md):
  seg, idx = road_idx[r['frame']]
  wseg, widx = wide_idx.get(r['frame'], (seg, idx)) if full else (seg, idx)
  # Copy the road planes before the shared qcamera decoder moves on.
  road = model_input([p.copy() for p in road_video.get(seg, idx)], road_s)
  wide = model_input(wide_video.get(wseg, widx), wide_s)
  d = latest(desire_log, r['t']); desire = np.zeros(8, np.float32)
  if d and 0 <= d[1] < 8: desire[d[1]] = 1
  desire[0] = 0
  pulse = np.where(desire - prev_desire > 0.99, desire, 0).astype(np.float32); prev_desire = desire
  n, v = latest(nav, r['t']), latest(car, r['t'])[1]
  fresh = n is not None and n[1] and 0 <= r['t'] - n[0] < 2
  navigation = navigation_input(n[2], max(0., v) * (r['t'] - n[0])) if fresh else np.zeros(47, np.float32)
  nav_map = maps[r['frame']] if r['frame'] in maps else np.zeros(65536, np.uint8)
  feed = dict(state)
  feed.update(new_img=np.stack([road, wide]).astype(dtype('new_img')), desire=pulse,
              traffic_convention=np.array([[1, 0]], np.float32), action_t=action_t,
              nav_map=nav_map.reshape(1, 1, 256, 256).astype(np.float32), navigation=navigation[None])
  o = dict(zip(out_names, sess.run(None, feed)))
  for name in state: state[name] = o['next_' + name].astype(state[name].dtype)
  outs.append(o['outputs'][0][:2066].astype(np.float32)); navs.append(navigation)
  if k % 100 == 0: print(f'{k} {(time.time() - started) / (k + 1):.2f}s/frame', flush=True)
np.savez(a.out, out=np.stack(outs), nav=np.stack(navs), t=np.array([r['t'] for r in md]),
         frame=np.array([r['frame'] for r in md]), v=np.array([latest(car, r['t'])[1] for r in md]),
         dev_curv=np.array([r['curv'] for r in md]), dev_y=np.array([r['y'] for r in md]),
         dev_x=np.array([r['x'] for r in md]), dev_lane=np.array([r['lane'] for r in md]),
         dev_vx=np.array([r['vx'] for r in md]), dev_ds=np.array([r['ds'] for r in md]))
print('saved', a.out)
