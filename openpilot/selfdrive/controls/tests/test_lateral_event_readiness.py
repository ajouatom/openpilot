import ast
from pathlib import Path
from typing import Optional

import pytest

from openpilot.cereal import log
from openpilot.common.utils import MovingAverage
from openpilot.selfdrive.controls.lib.lateral_readiness import LateralStartupGate, lateral_inputs_ready
from openpilot.selfdrive.controls.tests.test_lateral_readiness import Messages, production_guard


@pytest.mark.parametrize('poll', [False, True])
def test_event_change_bursts_do_not_block_ready_lateral_inputs(poll):
  # Run the real frequency tracker without requiring native IPC. controlsd
  # polls selfdriveState; card polls all of its subscribed services.
  path = Path(__file__).parents[3] / 'cereal/messaging/__init__.py'
  ns = {'Optional': Optional, 'MovingAverage': MovingAverage}
  exec(production_guard(path, lambda n: isinstance(n, ast.ClassDef) and n.name == 'FrequencyTracker'), ns)
  tracker = ns['FrequencyTracker'](1.0, 100.0, poll)
  sm = Messages()
  for t in range(1, 12):
    tracker.record_recv_time(float(t))
  assert tracker.valid
  # Brake/gas/steer changes publish between the regular heartbeat messages.
  rejected_burst = False
  for t in (11.02, 11.06, 11.12, 11.5, 12.0, 12.2):
    tracker.record_recv_time(t)
    rejected_burst |= not tracker.valid
    sm.freq_ok['onroadEvents'] = tracker.valid
    assert lateral_inputs_ready(sm, sm['carState'])
  assert rejected_burst


@pytest.mark.parametrize('failure', ['seen', 'valid', 'alive'])
def test_event_cadence_exception_preserves_required_receipt_and_health(failure):
  sm = Messages()
  sm.freq_ok['onroadEvents'] = False
  getattr(sm, failure)['onroadEvents'] = False
  assert not lateral_inputs_ready(sm, sm['carState'])
  getattr(sm, failure)['onroadEvents'] = True
  assert lateral_inputs_ready(sm, sm['carState'])


def test_event_bursts_do_not_bypass_initialization_or_model_health():
  sm = Messages()
  sm.freq_ok['onroadEvents'] = False
  sm['onroadEvents'] = [log.OnroadEvent.new_message(name='selfdriveInitializing')]
  assert not lateral_inputs_ready(sm, sm['carState'])
  sm['onroadEvents'] = []
  for failure in ('seen', 'valid', 'alive', 'freq_ok'):
    getattr(sm, failure)['modelV2'] = False
    assert not lateral_inputs_ready(sm, sm['carState'])
    getattr(sm, failure)['modelV2'] = True
  assert lateral_inputs_ready(sm, sm['carState'])


@pytest.mark.parametrize('boundary', ['carState', 'carControl'])
def test_first_readiness_accepts_healthy_event_bursts(boundary):
  sm = Messages()
  gate = LateralStartupGate()
  sm.freq_ok['onroadEvents'] = False
  sm.valid['onroadEvents'] = False
  assert not gate.update(sm, sm['carState'], boundary)
  sm.valid['onroadEvents'] = True
  assert gate.update(sm, sm['carState'], boundary)
  # The independently requested startup-only policy remains one-way.
  assert gate.update(None, None, boundary)
