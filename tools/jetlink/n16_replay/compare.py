#!/usr/bin/env python3
"""Replay runs next to the tablet's logged output (every 0.5 s), plus overall agreement.
Plan y is metres to the right at 20/40 m ahead; curv is the action curvature
(action[0] / max(1, v)^2, as get_action_from_model); turnL the left-turn desire state.
usage: compare.py RUN.npz [RUN2.npz ...] [--from T --to T]"""
import sys, numpy as np

def plan(out):  # (n, 33, 15) means
  return out[:, 917:917 + 495].reshape(-1, 33, 15)
def lanes(out):  # (n, 4, 33, 2) y,z means
  return out[:, 0:264].reshape(-1, 4, 33, 2)
def desire(out):
  d = out[:, 2054:2062]; e = np.exp(d - d.max(1, keepdims=True)); return e / e.sum(1, keepdims=True)
def y_at_x(xs, ys, x):
  return np.array([np.interp(x, a, b) for a, b in zip(xs, ys)])

args = sys.argv[1:]
lo = float(args[args.index('--from') + 1]) if '--from' in args else -1e9
hi = float(args[args.index('--to') + 1]) if '--to' in args else 1e9
runs = [(p, np.load(p)) for p in args if p.endswith('.npz')]
base = runs[0][1]
t, v = base['t'], base['v']
print('device vs first run, whole window:')
p0 = plan(base['out'])
print('  lane1 y@idx10 |dev-pc| mean %.3f m' % np.mean(np.abs(base['dev_lane'][:, 10] - lanes(base['out'])[:, 1, 10, 0])))
print('  plan y@idx16  |dev-pc| mean %.3f m' % np.mean(np.abs(base['dev_y'][:, 16] - p0[:, 16, 1])))
print()
hdr = '  t     v   nav(cmd,dist) | device: y@20m y@40m curv | ' + ' | '.join('%s: y@20m y@40m curv turnL' % p.split('/')[-1][:10] for p, _ in runs)
print(hdr)
for i in range(0, len(t), 10):
  if not lo <= t[i] <= hi: continue
  nav = base['nav'][i]; cmd = int(np.argmax(nav[:46])) if nav[:46].max() > 0 else 0
  row = '%6.1f %4.0f  (%2d,%5.0f)    | %6.1f %6.1f %+.4f |' % (t[i], v[i] * 3.6, cmd, nav[46] * 100,
        y_at_x(base['dev_x'][i:i+1], base['dev_y'][i:i+1], 20)[0], y_at_x(base['dev_x'][i:i+1], base['dev_y'][i:i+1], 40)[0], base['dev_curv'][i])
  for _, r in runs:
    p = plan(r['out'][i:i+1]); a = r['out'][i, 2062:2066]
    curv = a[0] / max(1., r['v'][i]) ** 2
    row += ' %6.1f %6.1f %+.4f %.2f |' % (y_at_x(p[:, :, 0], p[:, :, 1], 20)[0], y_at_x(p[:, :, 0], p[:, :, 1], 40)[0], curv, desire(r['out'][i:i+1])[0, 1])
  print(row)
