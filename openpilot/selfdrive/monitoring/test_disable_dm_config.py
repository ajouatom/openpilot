"""Desktop contracts for the shared snapshot; native IPC is not exercised."""
import ast
from pathlib import Path
from types import SimpleNamespace

import pytest

from openpilot.selfdrive.monitoring.config import configure_monitoring, disabled_mode


ROOT = Path(__file__).resolve().parents[2]


class FakeParams:
  def __init__(self, values=None):
    self.values = dict(values or {})

  def get(self, name):
    return self.values.get(name)

  def get_int(self, name):
    return int(self.values.get(name) or 0)

  def get_bool(self, name):
    return self.get_int(name) == 1

  def put(self, name, value):
    expected = bool if name == "CarrotVisionEnabled" else int
    if type(value) is not expected:
      raise TypeError(f"{name} requires {expected.__name__}, got {type(value).__name__}")
    self.values[name] = value

  def put_int(self, name, value):
    self.put(name, value)

  def put_bool(self, name, value):
    self.put(name, value)


def _tree(relative):
  return ast.parse((ROOT / relative).read_text(encoding="utf-8"))


def _manager_gates():
  names = {"enable_dm", "enable_webrtc", "cluster_hud_active"}
  body = [node for node in _tree("system/manager/process_config.py").body
          if isinstance(node, ast.FunctionDef) and node.name in names]
  namespace = {"disabled_mode": disabled_mode, "Params": FakeParams, "car": SimpleNamespace(CarParams=object)}
  exec(compile(ast.Module(body=body, type_ignores=[]), "process_config.py", "exec"), namespace)
  return namespace


def _consumer(relative, class_name, params):
  cls = next(node for node in _tree(relative).body if isinstance(node, ast.ClassDef) and node.name == class_name)
  init = next(node for node in cls.body if isinstance(node, ast.FunctionDef) and node.name == "__init__")
  assignment = next(node for node in init.body if isinstance(node, ast.Assign)
                    and any(ast.unparse(target) == "self.disable_dm" for target in node.targets))
  consumer = SimpleNamespace(params=params, CP=SimpleNamespace(notCar=False))
  exec(compile(ast.Module(body=[assignment], type_ignores=[]), relative, "exec"),
       {"self": consumer, "disabled_mode": disabled_mode})
  return cls, init, consumer


def _evaluate(test, consumer):
  return eval(compile(ast.Expression(test), "dm_gate", "eval"), {"self": consumer, "SIMULATION": False})


@pytest.mark.parametrize("initial", [0, 1, 2])
@pytest.mark.parametrize("saved", [0, 1, 2])
def test_all_consumers_hold_the_applied_mode_until_reboot(initial, saved):
  # Existing DM2 users do not rerun the streaming migration.
  params = FakeParams({"DisableDM": initial, "DriverMonitoringMode": 0, "CarrotVisionEnabled": 0})
  configure_monitoring(params, {})
  gates = _manager_gates()
  sd_class, sd_init, sd = _consumer("selfdrive/selfdrived/selfdrived.py", "SelfdriveD", params)
  controls_class, _, controls = _consumer("selfdrive/controls/controlsd.py", "Controls", params)
  dm_events = next(node for node in ast.walk(sd_class) if isinstance(node, ast.If)
                   and ast.unparse(node.test) == "not self.CP.notCar and self.disable_dm == 0")
  ignored = next(node for node in sd_init.body if isinstance(node, ast.If)
                 and "self.disable_dm" in ast.unparse(node.test))
  force_decel = next(node for node in ast.walk(controls_class) if isinstance(node, ast.If)
                    and ast.unparse(node.test) == "self.disable_dm == 0")

  params.put("DisableDM", saved)
  assert gates["enable_dm"](True, params, sd.CP) == (initial == 0)
  assert _evaluate(dm_events.test, sd) == (initial == 0)
  assert _evaluate(force_decel.test, controls) == (initial == 0)
  assert _evaluate(ignored.test, sd) == (initial != 0)
  assert gates["enable_webrtc"](True, params, sd.CP) == (initial == 2)
  # Restarting just a child must not pick up a pending saved value either.
  assert _consumer("selfdrive/selfdrived/selfdrived.py", "SelfdriveD", params)[2].disable_dm == initial
  assert _consumer("selfdrive/controls/controlsd.py", "Controls", params)[2].disable_dm == initial

  configure_monitoring(params, {})
  assert disabled_mode(params) == saved
  assert gates["enable_dm"](True, params, sd.CP) == (saved == 0)
  assert gates["enable_webrtc"](True, params, sd.CP) == (saved == 2)
  assert _consumer("selfdrive/selfdrived/selfdrived.py", "SelfdriveD", params)[2].disable_dm == saved
  assert _consumer("selfdrive/controls/controlsd.py", "Controls", params)[2].disable_dm == saved


@pytest.mark.parametrize("mode", [0, 1, 2])
@pytest.mark.parametrize("vision", [0, 1])
@pytest.mark.parametrize("cluster", [0, 1])
def test_manager_streaming_matrix(mode, vision, cluster):
  params = FakeParams({"DisableDM": mode, "DriverMonitoringMode": 0, "CarrotVisionEnabled": vision, "ClusterHud": cluster})
  configure_monitoring(params, {})
  assert _manager_gates()["enable_webrtc"](True, params, None) == (cluster == 0 and (vision == 1 or mode == 2))


def test_missing_or_unsupported_modes_keep_monitoring_enabled():
  params = FakeParams({"DisableDM": 2})
  assert disabled_mode(params) == 0  # manager has not applied a setting yet
  params.put("DisableDM", 99)
  configure_monitoring(params, {})
  assert disabled_mode(params) == 0
  params.put("DisableDMActive", -1)
  assert disabled_mode(params) == 0


def test_snapshot_is_internal_and_not_backed_up():
  registry = (ROOT / "common/params_keys.h").read_text(encoding="utf-8")
  assert '{"DisableDMActive", {CLEAR_ON_MANAGER_START, INT}}' in registry
  # No default: Params backup omits keys with no default, and catalog/profile
  # operations never list the internal snapshot as a user setting.
  import json
  catalog = json.loads((ROOT / "selfdrive/carrot_settings.json").read_text(encoding="utf-8"))
  assert "DisableDMActive" not in {entry["name"] for entry in catalog["params"]}
