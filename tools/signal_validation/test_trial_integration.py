import ast
from pathlib import Path
from types import SimpleNamespace as NS

from openpilot.cereal import log

ROOT = Path(__file__).resolve().parents[2]


def test_existing_ioniq_and_new_tesla_fields_keep_independent_wire_ids():
  msg = log.ModelDataV2.new_message()
  values = {'autoTurnSignal': 'left', 'autoTurnSignalLaneChange': True,
            'autoTurnSignalDirection': 'right', 'autoTurnSignalRequestId': 987654321}
  for name, value in values.items():
    setattr(msg.meta, name, value)
  fields = msg.meta.schema.fields
  for name, ordinal in zip(values, (22, 23, 24, 25), strict=True):
    assert fields[name].proto.ordinal.explicit == ordinal
  with log.ModelDataV2.from_bytes(msg.to_bytes()) as decoded:
    for name, value in values.items():
      assert getattr(decoded.meta, name) == value


def test_non_tesla_skips_signal_can_observation_and_cancellation():
  tree = ast.parse((ROOT / 'openpilot/selfdrive/car/card.py').read_text(encoding='utf-8'))
  method = next(n for n in ast.walk(tree) if isinstance(n, ast.FunctionDef) and n.name == 'state_update')
  guard = next(n for n in method.body if isinstance(n, ast.If) and ast.unparse(n.test) == "self.CP.brand == 'tesla'")
  # No receiver, clock, CAN or CS exists: executing any guarded statement fails.
  exec(compile(ast.Module(body=[guard], type_ignores=[]), '<card-state>', 'exec'),
       {'self': NS(CP=NS(brand='hyundai'))})


def test_non_tesla_desire_never_runs_tesla_intent_machine():
  from openpilot.selfdrive.controls.tests.test_desire_helper_stalk import TestDesireHelperStalkRequest
  fixture = TestDesireHelperStalkRequest()
  fixture.setup_method()
  def forbidden(*args, **kwargs):
    raise AssertionError('Tesla intent ran on non-Tesla')
  fixture.helper._update_auto_signal = forbidden
  fixture.start_and_cross()
  fixture.finish()
  assert fixture.helper.auto_signal_request_id == 0


def test_model_publishes_both_indicators_without_losing_nav_initialization():
  tree = ast.parse((ROOT / 'openpilot/selfdrive/modeld/modeld.py').read_text(encoding='utf-8'))
  assignments = [n for n in ast.walk(tree) if isinstance(n, ast.Assign) and
                 any(ast.unparse(t).startswith('modelv2_send.modelV2.meta.autoTurnSignal') for t in n.targets)]
  msg = log.ModelDataV2.new_message()
  h = NS(auto_turn_signal='left', auto_turn_signal_lane_change=True,
         auto_signal_direction='right', auto_signal_request_id=71)
  exec(compile(ast.Module(body=assignments, type_ignores=[]), '<model-meta>', 'exec'),
       {'modelv2_send': NS(modelV2=msg), 'DH': h})
  assert str(msg.meta.autoTurnSignal) == 'left' and msg.meta.autoTurnSignalLaneChange
  assert str(msg.meta.autoTurnSignalDirection) == 'right' and msg.meta.autoTurnSignalRequestId == 71
  assert any(isinstance(n, ast.Assign) and ast.unparse(n) == 'nav_input = MapInput()' for n in ast.walk(tree))
