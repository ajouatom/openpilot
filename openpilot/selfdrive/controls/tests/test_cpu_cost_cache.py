import ast
import math
import struct
from pathlib import Path
from types import SimpleNamespace as NS

import numpy as np
import pytest



class Solver:
  def __init__(self):
    self.calls = []
    self.values = {}

  def cost_set(self, i, field, value):
    self.calls.append((i, field))
    self.values[i, field] = value.copy()


def cost_method(lateral):
  folder = 'lateral' if lateral else 'longitudinal'
  path = Path(__file__).parents[1] / f'lib/{folder}_mpc_lib/{"lat" if lateral else "long"}_mpc.py'
  cls = next(n for n in ast.parse(path.read_text()).body if isinstance(n, ast.ClassDef))
  name = 'set_weights' if lateral else 'set_cost_weights'
  method = next(n for n in cls.body if isinstance(n, ast.FunctionDef) and n.name == name)
  times = np.array([10 * (i / 12)**2 for i in range(13)])
  ns = {'np': np, 'math': math, 'struct': struct, 'N': 32 if lateral else 12, 'COST_E_DIM': 3 if lateral else 5,
        'A_CHANGE_FACTORS': tuple(np.interp(t, [0., 1., 2.], [1., 1., 0.]) for t in times[:12])}
  exec(compile(ast.Module(body=[method], type_ignores=[]), str(path), 'exec'), ns)
  return ns[name]


@pytest.mark.parametrize('lateral', [False, True])
def test_cost_cache_tracks_exact_values_solver_and_mode(lateral):
  fn = cost_method(lateral)
  obj = NS(solver=Solver(), mode='acc', _cost_key=None, _cost_solver=None)
  w = [1., .1, 0., .04, 700.] if lateral else [5., 0., 0., 0., 200., 20.]
  cw = [1e6, 1e6, 1e6, 100.]
  def call():
    fn(obj, *w) if lateral else fn(obj, w, cw)
  count = 33 if lateral else 25
  call()
  assert len(obj.solver.calls) == count
  matrices = {k: v.tobytes() for k, v in obj.solver.values.items()}
  call()
  assert len(obj.solver.calls) == count
  assert matrices == {k: v.tobytes() for k, v in obj.solver.values.items()}
  # Signed zero and a single representable step must invalidate the cache.
  for value in (-0., np.nextafter(0., 1.), float('nan'), float('nan'), float('inf'), float('inf')):
    w[2] = value
    before = len(obj.solver.calls)
    call()
    assert len(obj.solver.calls) == before + count
  w[2] = 0.
  call()
  obj.solver = Solver()
  call()
  assert len(obj.solver.calls) == count
  if not lateral:
    obj.mode = 'blended'
    call()
    assert len(obj.solver.calls) == 2 * count
    cw[3] += 1.
    call()
    assert len(obj.solver.calls) == 3 * count
  obj._cost_key = None
  call()
  assert len(obj.solver.calls) >= 2 * count


@pytest.mark.parametrize('lateral', [False, True])
def test_partial_cost_write_failure_never_leaves_old_cache_valid(lateral):
  fn = cost_method(lateral)
  solver = Solver()
  obj = NS(solver=solver, mode='acc', _cost_key=None, _cost_solver=None)
  w = [1., .1, 0., .04, 700.] if lateral else [5., 0., 0., 0., 200., 20.]
  cw = [1e6, 1e6, 1e6, 100.]
  def call():
    fn(obj, *w) if lateral else fn(obj, w, cw)
  call()
  original = solver.cost_set
  def fail(i, field, value):
    original(i, field, value)
    if i == 2:
      raise RuntimeError('test partial write')
  solver.cost_set = fail
  w[0] += 1.
  with pytest.raises(RuntimeError):
    call()
  w[0] -= 1.
  solver.cost_set = original
  before = len(solver.calls)
  call()
  assert len(solver.calls) == before + (33 if lateral else 25)
