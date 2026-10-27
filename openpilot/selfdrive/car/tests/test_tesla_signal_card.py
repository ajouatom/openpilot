import ast
from pathlib import Path
from types import SimpleNamespace as NS

import pytest

from openpilot.cereal import car, log
from openpilot.selfdrive.car.tests.test_tesla_turn_signals import N, feedback, opened, state
from openpilot.selfdrive.car.tesla_turn_signals import TeslaTurnSignals
from openpilot.selfdrive.controls.lib.lateral_readiness import LateralStartupGate
from openpilot.selfdrive.pandad import can_list_to_can_capnp


@pytest.mark.parametrize("confirmed", [False, True])
def test_card_publishes_confirmed_request_id_separately_from_ownership(confirmed):
  path = Path(__file__).parents[1] / "card.py"
  tree = ast.parse(path.read_text())
  method = next(n for n in ast.walk(tree) if isinstance(n, ast.FunctionDef) and n.name == "state_update")
  fields = {"autoTurnSignalDirection", "autoTurnSignalReadyRequestId", "autoTurnSignalStatus"}
  guard = next(n for n in method.body if isinstance(n, ast.If) and ast.unparse(n.test) == "self.CP.brand == 'tesla'")
  assignments = [n for n in guard.body if isinstance(n, ast.Assign) and
                 any(isinstance(t, ast.Attribute) and isinstance(t.value, ast.Name) and
                     t.value.id == "CS" and t.attr in fields for t in n.targets)]
  assert len(assignments) == 3
  signals = opened()
  if not confirmed:
    signals.session.on_seen = False
  cs = car.CarState.new_message()
  namespace = {"self": NS(tesla_signals=signals, CP=NS(brand="tesla")), "CS": cs, "signal_now": N + 3}
  exec(compile(ast.Module(body=assignments, type_ignores=[]), str(path), "exec"), namespace)
  assert cs.autoTurnSignalDirection == 1
  assert cs.autoTurnSignalReadyRequestId == int(confirmed)
  assert cs.autoTurnSignalStatus == "active"


def test_received_driver_input_precedes_independent_cancellation_boundary():
  tree = ast.parse((Path(__file__).parents[1] / "card.py").read_text())
  calls = [node for node in ast.walk(tree) if isinstance(node, ast.Call) and isinstance(node.func, ast.Attribute)
           and isinstance(node.func.value, ast.Attribute) and node.func.value.attr == "tesla_signals"]
  driver = next(node.lineno for node in calls if node.func.attr == "observe_driver")
  observe = next(node.lineno for node in calls if node.func.attr == "observe")
  urgent = min(node.lineno for node in calls if node.func.attr == "take_sends")
  assert driver < observe < urgent


def test_lamp_diagnostics_are_periodic_only_during_sessions_and_immediate_on_status_change():
  path = Path(__file__).parents[1] / "card.py"
  tree = ast.parse(path.read_text())
  method = next(n for n in ast.walk(tree) if isinstance(n, ast.FunctionDef) and n.name == "_log_tesla_signals")
  events = []
  namespace = {"cloudlog": NS(event=lambda name, **values: events.append((name, values)))}
  exec(compile(ast.Module(body=[method], type_ignores=[]), str(path), "exec"), namespace)
  signals = TeslaTurnSignals(True)
  card = NS(tesla_signals=signals, tesla_signal_status=None, tesla_signal_log_ns=0)
  def log_signals(now):
    namespace["_log_tesla_signals"](card, state(), now)

  log_signals(N)
  log_signals(N + 5_000_000_000)
  assert len(events) == 1

  card.tesla_signals = signals = opened()
  log_signals(N + 1)
  for tick in range(1, 100):
    log_signals(N + 1 + tick * 10_000_000)
  assert len(events) == 2
  log_signals(N + 1_000_000_001)
  assert len(events) == 3
  assert events[-1][0] == "tesla_auto_turn_signal"
  assert events[-1][1]["active_submissions"] == events[-1][1]["active_tx_echoes"] == 1
  signals.status = "cancelling:driver_input_unavailable"
  log_signals(N + 1_000_000_002)
  assert len(events) == 4
  assert events[-1][1]["status"] == signals.status
  assert signals.session.sent == 1  # Logging never submits or advances a request.

@pytest.mark.parametrize("gate", ["ready", "startup", "dashcam", "stale_model", "stale_control"])
@pytest.mark.parametrize("suspend_ns", [0, 5_000_000_000])
def test_final_card_boundary_uses_gated_control_and_fresh_intent(gate, suspend_ns):
  path = Path(__file__).parents[1] / "card.py"
  tree = ast.parse(path.read_text())
  method = next(n for n in ast.walk(tree) if isinstance(n, ast.FunctionDef) and n.name == "controls_update")
  ns = {"car": car, "REPLAY": False, "can_list_to_can_capnp": can_list_to_can_capnp,
        "time": NS(monotonic=lambda: (N - suspend_ns) / 1e9, monotonic_ns=lambda: N - suspend_ns,
                   clock_gettime_ns=lambda clock: N if clock == 7 else N - suspend_ns, CLOCK_BOOTTIME=7, CLOCK_MONOTONIC=1)}
  exec(compile(ast.Module(body=[method], type_ignores=[]), str(path), "exec"), ns)
  meta = log.ModelDataV2.new_message()
  meta.meta.autoTurnSignalDirection = "left"
  meta.meta.autoTurnSignalRequestId = 42
  messages = {"modelV2": meta.as_reader(), "radarState": None}

  class Messages(dict):
    def all_alive(self, services):
      return True

  sm = Messages(messages)
  sm.valid = sm.alive = sm.seen = dict.fromkeys(("modelV2", "carControl", "radarState"), True)
  sm.recv_time = dict.fromkeys(sm.valid, (N - suspend_ns) / 1e9)
  if gate.startswith("stale"):
    sm.recv_time["modelV2" if gate == "stale_model" else "carControl"] -= 1
  if gate == "startup":
    # A real, unready gate must run before the lamp update.
    sm.all_checks = lambda services: False
  signals = TeslaTurnSignals(True)
  feedback(signals, N)
  params = NS(get_bool=lambda name: gate == "dashcam")
  applied, sent = [], []

  def apply(*args):
    applied.append(args)
    return car.CarControl.Actuators.new_message().as_reader(), []

  controller = NS(apply=apply)
  stages = ("apply", "sendcan", "total")
  card = NS(spas_signals=NS(enabled=False), initialized_prev=True, params=params, sm=sm, CI=controller,
            pm=NS(send=lambda *args: sent.append(args)), tesla_signals=signals,
            lateral_startup=LateralStartupGate(ready=gate != "startup"), card_diag_recv_ns=N - suspend_ns,
            card_diag_stage_names=stages, card_diag_stage_current=dict.fromkeys(stages, 0),
            card_diag_stage_sum_us=dict.fromkeys(stages, 0), card_diag_stage_max_us=dict.fromkeys(stages, 0),
            card_diag_process_max_us=0, card_diag_slow_process=0, card_diag_frames=0)
  cs = car.CarState.new_message(canValid=True, gearShifter="drive", turnSignalDriverValid=True)
  cc = car.CarControl.new_message(enabled=True, latActive=True)
  ns["controls_update"](card, cs.as_reader(), cc.as_reader())
  with log.Event.from_bytes(sent[-1][1]) as event:
    assert len(event.sendcan) == (1 if gate == "ready" else 0)
  assert applied[-1][0].latActive == (gate not in ("startup", "dashcam"))
  assert cc.latActive  # Do not mutate the subscribed command.
