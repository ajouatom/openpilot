import ast
from pathlib import Path
from types import SimpleNamespace

import pytest

from openpilot.common.runtime_diagnostics import RuntimeDiagnostics
from openpilot.selfdrive.car import cruise
from openpilot.selfdrive.car.tests.test_toyota_pcm_mode0 import full_helper, make_cs, step  # noqa: F401


@pytest.mark.parametrize('phase', ['params', 'bluetooth', 'assistant', 'finish'])
@pytest.mark.parametrize('brand', ['hyundai', 'toyota'])
def test_full_cruise_localizes_wait_and_clears_previous_frame(full_helper, monkeypatch, tmp_path, phase, brand):  # noqa: F811
  helper = full_helper
  helper.assistant_speed.root = tmp_path / 'assistant'
  helper.CP.brand = brand
  helper.CP.pcmCruise = brand == 'toyota'
  elapsed = [0, 0]
  clock = SimpleNamespace(monotonic_ns=lambda: elapsed[0], thread_time_ns=lambda: elapsed[1],
                          monotonic=lambda: elapsed[0] / 1e9)
  monkeypatch.setattr(cruise, 'time', clock)
  owner, method = {
    'params': (helper, 'update_params'),
    'bluetooth': (helper.bluetooth_commands, 'read'),
    'assistant': (helper.assistant_speed, 'read'),
    'finish': (helper.assistant_speed, 'finish'),
  }[phase]
  original = getattr(owner, method)

  def delayed(*args, **kwargs):
    elapsed[0] += 71_000_000
    elapsed[1] += 4_000_000
    return original(*args, **kwargs)

  monkeypatch.setattr(owner, method, delayed)
  assert step(helper, make_cs()) == 83.
  measured = helper.runtime_timing
  assert measured[f'cruise_{phase}_ms'] == 71.
  assert measured[f'cruise_{phase}_cpu_ms'] == 4.
  assert measured['cruise_total_ms'] == 71.
  assert measured['cruise_total_cpu_ms'] == 4.
  assert len(measured) == 12
  assert helper._activate_cruise == 0
  monkeypatch.setattr(owner, method, original)
  assert step(helper, make_cs()) == 83.
  assert all(value == 0 for value in helper.runtime_timing.values())
  assert measured[f'cruise_{phase}_ms'] == 71.  # Earlier frame has its own values.


@pytest.mark.parametrize('frame,refreshed', [(1, True), (2, False), (11, True)])
def test_card_step_publishes_correlated_cruise_measurement(frame, refreshed):
  # Execute the complete production step, isolating its platform-specific imports
  # and IPC. Its call to RuntimeDiagnostics and the logger are exercised together.
  path = Path(cruise.__file__).with_name('card.py')
  tree = ast.parse(path.read_text(encoding='utf8'))
  cls = next(node for node in tree.body if isinstance(node, ast.ClassDef) and node.name == 'Car')
  method = next(node for node in cls.body if isinstance(node, ast.FunctionDef) and node.name == 'step')
  namespace = {'time': SimpleNamespace(thread_time=lambda: 0., monotonic_ns=lambda: 1_000_000_000),
               'CAN_BACKEND': 'test'}
  exec(compile(ast.Module(body=[method], type_ignores=[]), str(path), 'exec'), namespace)
  events = []
  timing = RuntimeDiagnostics('card', lambda *a, **kw: events.append(kw), interval=0, peak_metric='state_tail_ms')
  state = SimpleNamespace(buttonEvents=[])

  class Inputs(dict):
    seen = {'onroadEvents': True}

  car = SimpleNamespace(
    state_update=lambda: state, state_publish=lambda cs: None,
    cruise_main_toggle=SimpleNamespace(update=lambda *a: False), CP=SimpleNamespace(passive=True),
    sm=Inputs(carControl=SimpleNamespace(enabled=True), onroadEvents=[]),
    card_diag_recv_ns=923_000_000, card_diag_stage_current={'state_tail': 71409},
    v_cruise_helper=SimpleNamespace(frame=frame, runtime_timing={'cruise_params_ms': 70., 'cruise_params_cpu_ms': 1.}),
    runtime_diagnostics=timing,
  )
  namespace['step'](car)
  peak = events[0]['peak_sample']
  assert peak['context'] == {'can_backend': 'test', 'cruise_frame': frame,
                             'cruise_params_refreshed': refreshed, 'state_receive_mono_ns': 923_000_000}
  assert peak['metrics']['state_tail_ms'] == 71.409
  assert peak['metrics']['cruise_params_ms'] == 70.
  assert peak['metrics']['cruise_params_cpu_ms'] == 1.
