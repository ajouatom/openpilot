import ast
from pathlib import Path
from types import SimpleNamespace as NS

# card imports messaging (msgq); like test_tesla_signal_card, take the method and its constant from the source
_tree = ast.parse((Path(__file__).parents[1] / "card.py").read_text(encoding="utf-8"))
_scope: dict = {}
_window = next(n for n in _tree.body if isinstance(n, ast.Assign) and ast.unparse(n.targets[0]) == "REMOTE_HAZARD_WINDOW")
_method = next(n for n in ast.walk(_tree) if isinstance(n, ast.FunctionDef) and n.name == "remote_blinker_update")
exec(compile(ast.Module(body=[_window, _method], type_ignores=[]), "card.py", "exec"), _scope)
update, REMOTE_HAZARD_WINDOW = _scope["remote_blinker_update"], _scope["REMOTE_HAZARD_WINDOW"]


def make(actions, remote_hazard):
  it, store = iter(actions), {"RemoteHazard": remote_hazard, "CanfdHazardTest": 0}
  params = NS(get_bool=lambda key: bool(store[key]), get_int=lambda key: store[key],
              put_int_nonblocking=lambda key, value: store.__setitem__(key, value))
  return NS(remote_blinker=NS(read=lambda: next(it, None)), remote_blink_pending=None, params=params,
            CI=NS(CC=NS(remote_blinker=0))), store


def test_without_remote_hazard_a_press_blinks_at_once():
  car, store = make(['blinkerLeft'], False)
  update(car, 0.0)
  assert car.CI.CC.remote_blinker == 3 and store["CanfdHazardTest"] == 0


def test_left_and_right_together_toggle_the_hazard_lights_can_test():
  car, store = make(['blinkerLeft', 'blinkerRight', None, 'blinkerRight', 'blinkerLeft'], True)
  update(car, 0.0)
  update(car, REMOTE_HAZARD_WINDOW - 0.1)
  assert store["CanfdHazardTest"] == 1 and car.CI.CC.remote_blinker == 0  # no blink for the pair
  update(car, 2.0)
  update(car, 3.0)
  update(car, 3.2)
  assert store["CanfdHazardTest"] == 0 and car.CI.CC.remote_blinker == 0


def test_a_single_press_blinks_after_the_window():
  car, store = make(['blinkerRight', None, None], True)
  update(car, 0.0)
  update(car, REMOTE_HAZARD_WINDOW - 0.1)
  assert car.CI.CC.remote_blinker == 0
  update(car, REMOTE_HAZARD_WINDOW + 0.1)
  assert car.CI.CC.remote_blinker == 4 and store["CanfdHazardTest"] == 0
