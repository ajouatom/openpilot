import ast
from pathlib import Path
from types import SimpleNamespace as NS

# card imports messaging (msgq); like test_tesla_signal_card, take the method from the source
_tree = ast.parse((Path(__file__).parents[1] / "card.py").read_text(encoding="utf-8"))
_method = next(n for n in ast.walk(_tree) if isinstance(n, ast.FunctionDef) and n.name == "remote_blinker_update")
_scope: dict = {}
exec(compile(ast.Module(body=[_method], type_ignores=[]), "card.py", "exec"), _scope)
update = _scope["remote_blinker_update"]


def make(actions):
  it, store = iter(actions), {"CanfdHazardTest": 0}
  params = NS(get_int=lambda key: store[key], put_int_nonblocking=lambda key, value: store.__setitem__(key, value))
  return NS(remote_blinker=NS(read=lambda: next(it, None)), params=params, CI=NS(CC=NS(remote_blinker=0))), store


def test_blinker_presses_blink_at_once():
  car, store = make(['blinkerLeft'])
  update(car, 0.0)
  assert car.CI.CC.remote_blinker == 3 and store["CanfdHazardTest"] == 0
  car, store = make(['blinkerRight'])
  update(car, 0.0)
  assert car.CI.CC.remote_blinker == 4


def test_hazard_action_toggles_the_hazard_lights_can_test():
  car, store = make(['hazard', None, 'hazard'])
  update(car, 0.0)
  assert store["CanfdHazardTest"] == 1 and car.CI.CC.remote_blinker == 0
  update(car, 1.0)
  assert store["CanfdHazardTest"] == 1
  update(car, 2.0)
  assert store["CanfdHazardTest"] == 0
