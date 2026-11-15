"""QMI/PPP routing and ModemManager regressions, with no device or network access."""
import ast
import json
import os
from pathlib import Path
import queue
import subprocess
import sys
from types import SimpleNamespace

import pytest


ROOT = Path(__file__).resolve().parents[3]


def load_members(path, class_name=None):
  tree = ast.parse((ROOT / path).read_text(encoding="utf-8"))
  if class_name:
    tree = next(node for node in tree.body if isinstance(node, ast.ClassDef) and node.name == class_name)
  return {node.name: node for node in tree.body if isinstance(node, ast.FunctionDef)}


def compile_functions(nodes, namespace):
  future = ast.ImportFrom(module="__future__", names=[ast.alias(name="annotations")], level=0)
  exec(compile(ast.fix_missing_locations(ast.Module(body=[future, *nodes], type_ignores=[])), __file__, "exec"), namespace)
  return namespace


@pytest.fixture
def qmi(monkeypatch, mocker):
  service = "org.freedesktop.ModemManager1"
  props = {"State": 11, "AccessTechnologies": 1 << 14, "Sim": "/sim/0", "UnlockRequired": 1,
           "SignalQuality": (72, True), "EquipmentIdentifier": "test-imei", "Revision": "test-version"}
  objects = {"/modem/0": {service + ".Modem": props, service + ".Modem.Modem3gpp": {"OperatorName": "test-carrier"}}}
  manager = mocker.Mock()
  manager.GetManagedObjects.return_value = objects
  sim = mocker.Mock()
  sim.GetAll.return_value = {"SimIdentifier": "test-iccid", "OperatorIdentifier": "45005"}
  bus = mocker.Mock()
  bus.get_object.side_effect = lambda name, path, **kw: sim if path == "/sim/0" else manager
  dbus = SimpleNamespace(SystemBus=lambda: bus, Interface=lambda obj, name: obj, DBusException=RuntimeError)
  monkeypatch.setitem(sys.modules, "dbus", dbus)
  paths = mocker.Mock(side_effect=lambda *parts: SimpleNamespace(read_text=lambda: "1234"))
  if os.getenv("NATIVE_DEVICE_TESTS") == "1":
    from openpilot.system.hardware.tici import hardware
    monkeypatch.setattr(hardware, "Path", paths)
    device = hardware.Tici()
    monkeypatch.setattr(device, "get_device_type", lambda: "tici")
  else:
    members = load_members("openpilot/system/hardware/tici/hardware.py", "Tici")
    ns = compile_functions([members["_get_qmi_modem_state"], members["get_modem_state"]], {"Path": paths, "json": json})
    device = type("Tici", (), {name: ns[name] for name in ("_get_qmi_modem_state", "get_modem_state")})()
    device.get_device_type = lambda: "tici"
  return SimpleNamespace(device=device, props=props, objects=objects, manager=manager, sim=sim)


@pytest.mark.parametrize(("state", "name"), [
  (-1, "FAILED"), (0, "UNKNOWN"), (1, "INITIALIZING"), (2, "LOCKED"), (3, "DISABLED"),
  (4, "DISABLING"), (5, "ENABLING"), (6, "ENABLED"), (7, "SEARCHING"), (8, "REGISTERED"),
  (9, "DISCONNECTING"), (10, "CONNECTING"), (11, "CONNECTED"), (99, "UNKNOWN"),
])
def test_modemmanager_states(qmi, state, name):
  qmi.props["State"] = state
  result = qmi.device.get_modem_state()
  assert result["state"] == name
  assert result["connected"] == (state == 11)
  assert result["tx_bytes"] == result["rx_bytes"] == 1234
  assert result["operator"] == "test-carrier"
  qmi.manager.GetManagedObjects.assert_called_once_with(timeout=0.5)


@pytest.mark.parametrize(("lock", "name"), [(0, "UNKNOWN"), (1, "READY"), (2, "SIM PIN"),
                                             (3, "SIM PIN2"), (4, "SIM PUK"), (5, "SIM PUK2"), (6, "LOCKED")])
def test_sim_lock_is_reported(qmi, lock, name):
  qmi.props["UnlockRequired"] = lock
  result = qmi.device.get_modem_state()
  assert result["sim_state"] == name
  assert result["iccid"] == "test-iccid"
  assert result["mcc_mnc"] == "45005"


def test_modem_and_sim_disappearance(qmi):
  qmi.props["Sim"] = "/"
  assert qmi.device.get_modem_state()["sim_state"] == "ABSENT"
  qmi.sim.GetAll.assert_not_called()
  qmi.props["Sim"] = "/sim/0"
  qmi.sim.GetAll.side_effect = RuntimeError("SIM removed")
  assert qmi.device.get_modem_state()["sim_state"] == "UNKNOWN"
  qmi.objects.clear()
  assert qmi.device.get_modem_state() == {}
  qmi.manager.GetManagedObjects.side_effect = RuntimeError("service unavailable")
  assert qmi.device.get_modem_state() == {}


@pytest.mark.parametrize(("access", "network"), [(1 << 14, "lte"), (1 << 5, "utran"), (1 << 9, "utran"),
                                                 (1 << 1, "gsm"), (1 << 4, "gsm"), (0, "")])
def test_network_type(qmi, access, network):
  qmi.props["AccessTechnologies"] = access
  assert qmi.device.get_modem_state()["network_type"] == network


@pytest.mark.parametrize("device_type", ["tizi", "mici"])
def test_other_devices_keep_ppp_json(qmi, monkeypatch, tmp_path, device_type):
  state_path = tmp_path / "modem.json"
  state = {"state": "CONNECTED", "connected": True, "network_type": "lte"}
  state_path.write_text(json.dumps(state), encoding="utf-8")
  monkeypatch.setattr(qmi.device, "get_device_type", lambda: device_type)
  monkeypatch.setitem(qmi.device.get_modem_state.__func__.__globals__, "MODEM_STATE_PATH", str(state_path))
  assert qmi.device.get_modem_state() == state
  qmi.manager.GetManagedObjects.assert_not_called()


@pytest.mark.parametrize(("is_tici", "device_type", "expected"), [(True, "tici", False), (True, "tizi", True),
                                                                  (True, "mici", True), (False, "pc", False)])
def test_manager_modem_selection(is_tici, device_type, expected):
  tree = ast.parse((ROOT / "openpilot/system/manager/process_config.py").read_text(encoding="utf-8"))
  modem = next(node for node in ast.walk(tree) if isinstance(node, ast.Call) and isinstance(node.func, ast.Name)
               and node.func.id == "PythonProcess" and node.args[0].value == "modem")
  result = eval(compile(ast.Expression(modem), __file__, "eval"),
                {"PythonProcess": lambda *args, **kw: kw, "TICI": is_tici,
                 "HARDWARE": SimpleNamespace(get_device_type=lambda: device_type), "always_run": object()})
  assert result["enabled"] == expected


@pytest.mark.parametrize("device_type", ["tici", "tizi", "mici"])
@pytest.mark.parametrize("version", [None, "test-version"])
@pytest.mark.parametrize("timeout", [False, True])
def test_modem_recovery_is_once_bounded_and_c3_only(mocker, device_type, version, timeout):
  hardware = mocker.Mock()
  hardware.get_modem_temperatures.return_value = []
  hardware.get_modem_data_usage.return_value = (0, 0)
  hardware.get_modem_version.return_value = version
  hardware.get_device_type.return_value = device_type
  command = mocker.Mock()
  if timeout:
    command.run.side_effect = subprocess.TimeoutExpired("systemctl", 5)
  end_event = mocker.Mock()
  end_event.is_set.side_effect = [False] * 8 + [True]
  ns = {"HARDWARE": hardware, "TICI": True, "AGNOS": True, "DT_HW": 10,
        "cloudlog": mocker.Mock(), "subprocess": command, "queue": queue,
        "HardwareState": SimpleNamespace, "time": mocker.Mock(), "os": mocker.Mock()}
  if os.getenv("NATIVE_DEVICE_TESTS") == "1":
    from openpilot.system.hardware import hardwared
    with mocker.patch.dict(hardwared.hw_state_thread.__globals__, ns):
      hardwared.hw_state_thread(end_event, mocker.Mock())
  else:
    fn = load_members("openpilot/system/hardware/hardwared.py")["hw_state_thread"]
    compile_functions([fn], ns)["hw_state_thread"](end_event, mocker.Mock())
  ns["os"].system.assert_not_called()
  if device_type == "tici" and version is None:
    command.run.assert_called_once_with(["sudo", "-n", "systemctl", "restart", "--no-block", "ModemManager"],
                                        timeout=5, check=True)
  else:
    command.run.assert_not_called()


@pytest.mark.parametrize("method", ["run", "stop"])
def test_direct_ppp_cannot_take_c3_modem(mocker, method):
  ns = {"logging": mocker.Mock()}
  device = SimpleNamespace(device_type="tici")
  if os.getenv("NATIVE_DEVICE_TESTS") == "1":
    from openpilot.system.hardware.tici.modem import Modem
    getattr(Modem, method)(device)
  else:
    fn = load_members("openpilot/system/hardware/tici/modem.py", "Modem")[method]
    compile_functions([fn], ns)[method](device)


@pytest.mark.parametrize("device_type", ["tici", "tizi", "mici"])
@pytest.mark.parametrize("version", ["EC20-test", "EG25-test", "other-modem"])
def test_sim_hot_swap_commands_are_tizi_only(mocker, device_type, version):
  device = SimpleNamespace(device_type=device_type, _at=mocker.Mock())
  if os.getenv("NATIVE_DEVICE_TESTS") == "1":
    from openpilot.system.hardware.tici.modem import Modem
    Modem._configure_modem(device, version)
  else:
    fn = load_members("openpilot/system/hardware/tici/modem.py", "Modem")["_configure_modem"]
    compile_functions([fn], {})["_configure_modem"](device, version)
  commands = [call.args[0] for call in device._at.call_args_list]
  expected = device_type == "tizi" and version != "other-modem"
  assert ('AT+QSIMDET=1,0' in commands) == expected
  assert ('AT+QSIMSTAT=1' in commands) == expected
  if version == "other-modem":
    device._at.assert_not_called()
