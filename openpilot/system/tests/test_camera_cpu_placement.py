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
  assert cores == ["5"]
  camera_core = int(cores[0])
  calls, writes = [], []
  set_power_save = load_function("openpilot/system/hardware/tici/hardware.py", "set_power_save", {
    "affine_irq": lambda core, action: calls.append((core, action)),
    "sudo_write": lambda value, path: writes.append((value, path)),
  })
  hardware = SimpleNamespace(amplifier=None)
  # Exercise real power-save code, including offline/online transitions, with
  # sysfs writes mocked. The target must never drift back to card's core6.
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
@pytest.mark.parametrize("fail_once", [False, True])
def test_ui_uses_little_cores_and_retries_affinity_without_rt_promotion(big_ui, fail_once):
  affinity = {0, 1, 2, 3, 4, 5}
  calls, events = [], []

  def set_affinity(cores):
    nonlocal affinity, fail_once
    calls.append(set(cores))
    if len(cores) > 1 and fail_once:
      fail_once = False
      raise OSError("transient affinity failure")
    affinity = set(cores)

  main = load_function("openpilot/selfdrive/ui/ui.py", "main", {
    "TICI": True, "BIG_UI": big_ui,
    "gc": SimpleNamespace(disable=lambda: events.append("gc_disabled")),
    "os": SimpleNamespace(sched_getaffinity=lambda _: affinity),
    "set_core_affinity": set_affinity,
    "ensure_ui_sched_other": lambda: events.append("sched_other"),
    "gui_app": SimpleNamespace(init_window=lambda _: events.append("window"), render=lambda: iter([True] * 3)),
    "ui_state": SimpleNamespace(update=lambda: None),
    "MainLayout": lambda: events.append("big"),
    "MiciMainLayout": lambda: events.append("mici"),
  })
  main()
  assert calls[0] == {0}  # GUI workers inherit the safe bootstrap affinity.
  assert all(cores == {0, 1, 2, 3} for cores in calls[1:])
  assert affinity == {0, 1, 2, 3}
  assert events == ["gc_disabled", "sched_other", "window", "big" if big_ui else "mici"]


def test_camera_move_keeps_control_and_model_placements():
  # This trial must not silently move deadline-sensitive consumers with it.
  placements = {
    "openpilot/selfdrive/car/card.py": (6, "Priority.CTRL_HIGH"),
    "openpilot/selfdrive/controls/controlsd.py": (4, "Priority.CTRL_HIGH"),
    "openpilot/selfdrive/selfdrived/selfdrived.py": (4, "Priority.CTRL_HIGH"),
    "openpilot/selfdrive/controls/plannerd.py": (5, "Priority.CTRL_LOW"),
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
