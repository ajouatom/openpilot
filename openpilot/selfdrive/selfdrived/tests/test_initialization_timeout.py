"""Exercise production startup gating without device-only process imports."""
import ast
from pathlib import Path
from types import SimpleNamespace

import pytest


def sample_method(*, simulation=False, replay=False):
  path = Path(__file__).resolve().parents[1] / 'selfdrived.py'
  tree = ast.parse(path.read_text(encoding='utf-8'))
  cls = next(n for n in tree.body if isinstance(n, ast.ClassDef) and n.name == 'SelfdriveD')
  method = next(n for n in cls.body if isinstance(n, ast.FunctionDef) and n.name == 'data_sample')
  records = []
  namespace = {
    'messaging': SimpleNamespace(recv_one=lambda sock: None),
    'DT_CTRL': 0.01, 'SIMULATION': simulation, 'REPLAY': replay,
    'VisionIpcClient': SimpleNamespace(available_streams=lambda *a, **kw: ['road', 'wide']),
    'VisionStreamType': SimpleNamespace(VISION_STREAM_ROAD='road', VISION_STREAM_WIDE_ROAD='wide'),
    'State': SimpleNamespace(enabled='enabled'), 'IGNORED_SAFETY_MODES': (),
    'cloudlog': SimpleNamespace(event=lambda name, **kw: records.append((name, kw))),
  }
  exec(compile(ast.Module(body=[method], type_ignores=[]), str(path), 'exec'), namespace)
  return namespace['data_sample'], records


def startup(frame, *, healthy=False, can_valid=True):
  class Messages(dict):
    def update(self, timeout):
      assert timeout == 0

    def all_checks(self):
      return healthy

  sm = Messages(pandaStates=[])
  sm.frame = frame
  sm.valid = sm.alive = sm.freq_ok = {'modelV2': healthy}
  sm.ignore_alive, sm.ignore_valid = [], []
  return SimpleNamespace(sm=sm, initialized=False, enabled=False, use_wide_camera=True,
                         CS_prev=SimpleNamespace(canValid=can_valid), car_state_sock=None,
                         state_machine=SimpleNamespace(state='disabled'), mismatch_counter=0)


@pytest.mark.parametrize('frame', [601, 870, 999, 1000])
def test_unready_services_wait_through_ten_seconds(frame):
  sample, records = sample_method()
  state = startup(frame)
  sample(state)
  assert not state.initialized
  assert not records


@pytest.mark.parametrize('frame', [200, 870, 950])
def test_healthy_services_finish_immediately(frame):
  sample, records = sample_method()
  state = startup(frame, healthy=True)
  sample(state)
  assert state.initialized
  assert len(records) == 1 and not records[0][1]['timeout']
  assert not state.enabled


def test_model_readiness_alone_does_not_bypass_invalid_can():
  sample, records = sample_method()
  state = startup(950, healthy=True, can_valid=False)
  sample(state)
  assert not state.initialized and not records


def test_timeout_still_enters_diagnostics_without_engaging():
  sample, records = sample_method()
  state = startup(1001, can_valid=False)
  sample(state)
  assert state.initialized and not state.enabled
  assert state.state_machine.state == 'disabled'
  assert records[0][1]['timeout'] and not records[0][1]['canValid']
  assert records[0][1]['not_alive'] == ['modelV2']
  sample(state)
  assert len(records) == 1


@pytest.mark.parametrize('simulation,replay,initialized', [(True, False, True), (True, True, False), (False, True, False)])
def test_existing_simulation_and_replay_policy(simulation, replay, initialized):
  sample, _ = sample_method(simulation=simulation, replay=replay)
  state = startup(601)
  sample(state)
  assert state.initialized == initialized
