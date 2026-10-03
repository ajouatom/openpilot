import json
from pathlib import Path

import pytest

from opendbc.car import gen_empty_fingerprint
from opendbc.car import interfaces as interfaces_module
from opendbc.car.hyundai import carstate, hyundaicanfd, interface
from opendbc.car.hyundai.values import CAR, CANFD_CAR, HyundaiFlags, HyundaiSafetyFlags


@pytest.fixture
def configuration(monkeypatch):
  fingerprint = gen_empty_fingerprint()
  values = {"FingerPrints": repr(fingerprint)}

  class Params:
    def get_int(self, key):
      return int(values.get(key, 0))

    def get_bool(self, key):
      return bool(values.get(key, False))

    def get(self, key):
      return values.get(key, "")

    def put_bool(self, key, value):
      pass

  for module in (interface, hyundaicanfd, carstate, interfaces_module):
    monkeypatch.setattr(module, "Params", Params)

  def make(candidate):
    return interface.CarInterface.get_params(candidate, fingerprint, [], False, False, False)

  return values, make


@pytest.mark.parametrize("candidate", [c for c in CANFD_CAR if c in interfaces_module.get_torque_params()])
@pytest.mark.parametrize("hda2", [0, 1])
def test_configured_canfd_vehicles_default_off_and_manual_on(configuration, candidate, hda2):
  values, make = configuration
  values["HyundaiCameraSCC"] = 1
  values["CanfdHDA2"] = hda2
  default = make(candidate)
  assert not default.flags & HyundaiFlags.CANFD_CLUSTER_DIRECT_TX
  assert not default.safetyConfigs[-1].safetyParam & HyundaiSafetyFlags.CANFD_CLUSTER_DIRECT_TX
  values["HyundaiCanfdClusterDirectTx"] = True
  enabled = make(candidate)
  assert enabled.flags == default.flags | HyundaiFlags.CANFD_CLUSTER_DIRECT_TX
  assert enabled.safetyConfigs[-1].safetyParam == default.safetyConfigs[-1].safetyParam | HyundaiSafetyFlags.CANFD_CLUSTER_DIRECT_TX
  # A saved setting change is applied when CarParams is rebuilt, never live.
  values["HyundaiCanfdClusterDirectTx"] = False
  assert enabled.flags & HyundaiFlags.CANFD_CLUSTER_DIRECT_TX
  assert make(candidate).flags == default.flags


@pytest.mark.parametrize("candidate", [CAR.KIA_EV6, CAR.HYUNDAI_SONATA])
def test_setting_ignored_outside_camera_canfd(configuration, candidate):
  values, make = configuration
  original = make(candidate)
  values["HyundaiCanfdClusterDirectTx"] = True
  enabled = make(candidate)
  assert enabled.flags == original.flags
  assert [c.to_dict() for c in enabled.safetyConfigs] == [c.to_dict() for c in original.safetyConfigs]


def test_direct_tx_setting_defaults_and_safety_bit():
  # No platform, including entries without complete torque configuration, opts in.
  assert all(not c.config.flags & HyundaiFlags.CANFD_CLUSTER_DIRECT_TX for c in CAR)
  root = next(p for p in Path(__file__).resolve().parents if (p / "panda/board").is_dir())
  catalog = json.loads((root / "openpilot/selfdrive/carrot_settings.json").read_text(encoding="utf-8"))
  setting = next(p for p in catalog["params"] if p["name"] == "HyundaiCanfdClusterDirectTx")
  assert setting["default"] == 0 and setting["control"] == "toggle"
  keys = (root / "openpilot/common/params_keys.h").read_text(encoding="utf-8")
  assert '{"HyundaiCanfdClusterDirectTx", {PERSISTENT, BOOL, "0"}}' in keys
  # Bit 1024 belongs to the separately gated parked blinker diagnostic.
  assert HyundaiSafetyFlags.CANFD_CLUSTER_DIRECT_TX == 2048
  header = (root / "opendbc_repo/opendbc/safety/safety/safety_hyundai_canfd.h").read_bytes()
  assert b"HYUNDAI_PARAM_CANFD_CLUSTER_DIRECT_TX = 2048;" in header
