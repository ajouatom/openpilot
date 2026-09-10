import sys
from types import ModuleType

from openpilot.selfdrive.carrot.server.services import device_info


class _Hardware:
  def get_device_type(self):
    return "tici"

  def get_imei(self, slot):
    assert slot == 0
    return "351234567890123"

  def get_serial(self):
    return "hardware-serial"

  def get_modem_version(self):
    return "RM500Q"

  def get_network_info(self):
    return {"technology": "LTE", "operator": "Carrier", "state": "CONNECTED"}

  def get_sim_info(self):
    return {"sim_state": ["READY"]}


class _SerialUnavailableHardware(_Hardware):
  def get_serial(self):
    raise RuntimeError("serial service is starting")


def test_tools_device_info_keeps_hardware_identity_outside_params(monkeypatch):
  monkeypatch.setattr(device_info, "get_param_values", lambda _names, _defaults: {
    "DongleId": "dongle-id",
    "HardwareSerial": "",
    "DevicePosition": "center",
    "GitBranch": "carrot-wip",
    "GitCommit": "2df34f5d",
    "GitCommitDate": "2026-09-10",
    "GitPullTime": "2026-09-10T10:11:12Z",
  })
  hardware_module = ModuleType("openpilot.system.hardware")
  hardware_module.HARDWARE = _Hardware()
  monkeypatch.setitem(sys.modules, "openpilot.system.hardware", hardware_module)
  monkeypatch.setattr(device_info, "_boot_time", lambda: "2026-09-10T10:11:12Z")

  info = device_info.get_tools_device_info()

  assert info["identity"] == {
    "device_type": "tici",
    "imei": "351234567890123",
    "imei_available": True,
    "dongle_id": "dongle-id",
    "hardware_serial": "hardware-serial",
    "position": "center",
  }
  assert info["connectivity"]["carrier"] == "Carrier"
  assert info["connectivity"]["sim_state"] == "READY"
  assert info["runtime"] == {"boot_time": "2026-09-10T10:11:12Z"}
  assert "iccid" not in str(info).lower()


def test_tools_device_info_keeps_imei_when_another_hardware_field_is_unavailable(monkeypatch):
  monkeypatch.setattr(device_info, "get_param_values", lambda _names, _defaults: {"HardwareSerial": "param-serial"})
  hardware_module = ModuleType("openpilot.system.hardware")
  hardware_module.HARDWARE = _SerialUnavailableHardware()
  monkeypatch.setitem(sys.modules, "openpilot.system.hardware", hardware_module)

  info = device_info.get_tools_device_info()

  assert info["identity"]["imei"] == "351234567890123"
  assert info["identity"]["hardware_serial"] == "param-serial"
