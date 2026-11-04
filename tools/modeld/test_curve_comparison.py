from types import SimpleNamespace as NS

import numpy as np

from tools.modeld.compare_curve_policies import compare  # noqa: TID251 - offline tool
from tools.modeld import compare_curve_policies as comparison  # noqa: TID251
from tools.modeld.ssh_curve_speed_reference import CurveSpeed  # noqa: TID251


def test_missing_recorded_std_is_never_reported_as_measured(monkeypatch, tmp_path):
  t = np.linspace(0, 10, 33).tolist()
  x = (np.asarray(t)*20).tolist()
  record = (100., True, 1, t, x, [0.]*33, [0.]*33, [20.]*33, [.1]*33, [], 0)
  data = ([record], [(100., True, 20., 0., 1.)], [(100., True, .2)], [], {},
          {'AutoCurveSpeedFactor': 100, 'AutoCurveSpeedLowerLimit': 30}, [], [])
  monkeypatch.setattr(comparison, 'read_segment', lambda _: data)
  summary, rows = compare(tmp_path/'recorded.zst')
  assert summary['measured_std_frames'] == 0 and len(rows) == 1
  assert not rows[0]['measured_std'] and 'measured_reference_accel' not in rows[0]
  assert rows[0]['reference_accel_0.05'] < rows[0]['reference_accel_0.0']
  # Real stds follow their separate path; scenarios never replace them.
  data[0][0] = (*record[:9], [.01]*33, 0)
  summary, rows = compare(tmp_path/'recorded.zst')
  assert summary['measured_std_frames'] == 1
  assert rows[0]['measured_reference_accel'] == rows[0]['reference_accel_0.01']


def test_reference_rejects_isolated_curvature_spike_and_releases_gradually():
  times = np.linspace(0, 10, 33)
  def msg(yaw):
    return NS(position=NS(t=times, x=times*20), velocity=NS(x=np.full(33, 20.)),
              orientationRate=NS(z=yaw, zStd=[]))
  spike = np.zeros(33)
  spike[8] = 1.
  assert CurveSpeed(.05).update(msg(spike), 20.) > 0
  curve = np.zeros(33)
  curve[8:14] = 1.
  policy = CurveSpeed(.05)
  for _ in range(40):
    previous = policy.update(msg(curve), 20.)
  assert previous < 0
  for _ in range(10):
    value = policy.update(msg(np.zeros(33)), 20.)
    assert value-previous <= .1+1e-9
    previous = value
