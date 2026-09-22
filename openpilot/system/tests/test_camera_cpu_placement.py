"""Camera/UI placement contract without importing device drivers or the GUI."""
import ast
from pathlib import Path
import re
from types import SimpleNamespace

import pytest

ROOT = Path(__file__).resolve().parents[3]


def load_function(path, name, scope):
  tree = ast.parse((ROOT / path).read_text(encoding="utf-8"))
  function = next(n for n in ast.walk(tree) if isinstance(n, ast.FunctionDef) and n.name == name)
  exec(compile(ast.Module(body=[function], type_ignores=[]), path, "exec"), scope)
  return scope[name]


def test_camera_irqs_follow_camerad_across_power_save():
  source = (ROOT / "openpilot/system/camerad/main.cc").read_text(encoding="utf-8")
  cores = re.findall(r"util::set_core_affinity\(\{(\d+)\}\)", source)
  assert cores == ["6"]
  camera_core = int(cores[0])
  calls, writes = [], []
  set_power_save = load_function("openpilot/system/hardware/tici/hardware.py", "set_power_save", {
    "affine_irq": lambda core, action: calls.append((core, action)),
    "sudo_write": lambda value, path: writes.append((value, path)),
  })
  hardware = SimpleNamespace(amplifier=None)
  # Exercise real power-save code, including offline/online transitions, with
  # sysfs writes mocked. Camera work must stay off non-isolated core5.
  for powersave in (False, True, False):
    calls.clear()
    writes.clear()
    set_power_save(hardware, powersave)
    assert calls == [(7, "kgsl-3d0")] + [
      (camera_core, action) for action in
      ("a5", "cci", "cpas_camnoc", "cpas-cdm", "csid", "ife", "csid-lite", "ife-lite")
    ]
    for core in range(4, 8):
      assert ("0" if powersave else "1", f"/sys/devices/system/cpu/cpu{core}/online") in writes


@pytest.mark.parametrize("big_ui", [False, True])
def test_ui_updates_onroad_policy_even_without_a_render(big_ui):
  calls, events = [], []
  states = iter([True, False, True])
  ui_state = SimpleNamespace(started=False)
  ui_state.update = lambda: setattr(ui_state, "started", next(states))
  def scheduler(core, *, enabled):
    assert core == 6 and enabled
    return SimpleNamespace(update=lambda onroad, **kw: calls.append(onroad))
  main = load_function("openpilot/selfdrive/ui/ui.py", "main", {
    "TICI": True, "BIG_UI": big_ui, "DisplayScheduler": scheduler,
    "gc": SimpleNamespace(disable=lambda: events.append("gc_disabled")),
    "set_core_affinity": lambda cores: events.append(tuple(cores)),
    "ensure_ui_sched_other": lambda: events.append("sched_other"),
    "gui_app": SimpleNamespace(init_window=lambda _: None, render=lambda: iter([True, False, True])),
    "ui_state": ui_state, "MainLayout": lambda: None, "MiciMainLayout": lambda: None,
  })
  main()
  assert events == ["gc_disabled", (0,), "sched_other"]
  assert calls == [False, True, False, True]


def test_camera_isolated_from_card_and_planner_without_priority_changes():
  # Separate card/radard and planner/radarcan; share camera with short controls.
  placements = {
    "openpilot/selfdrive/car/card.py": (5, "Priority.CTRL_HIGH"),
    "openpilot/selfdrive/controls/controlsd.py": (6, "Priority.CTRL_HIGH"),
    "openpilot/selfdrive/selfdrived/selfdrived.py": (6, "Priority.CTRL_HIGH"),
    "openpilot/selfdrive/controls/plannerd.py": (4, "Priority.CTRL_LOW"),
    "openpilot/selfdrive/carrot/radar/radarcan.py": (4, "Priority.CTRL_LOW"),
    "openpilot/selfdrive/carrot/radar/radard_dpath.py": (5, "Priority.CTRL_LOW"),
    "openpilot/selfdrive/modeld/modeld.py": (7, "54"),
  }
  for path, (core, priority) in placements.items():
    tree = ast.parse((ROOT / path).read_text(encoding="utf-8"))
    calls = [n for n in ast.walk(tree) if isinstance(n, ast.Call) and
             isinstance(n.func, ast.Name) and n.func.id == "config_realtime_process"]
    assert len(calls) == 1, path
    assert ast.literal_eval(calls[0].args[0]) == core, path
    assert ast.unparse(calls[0].args[1]) == priority, path
