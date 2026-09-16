from types import SimpleNamespace

import pytest

from opendbc.can import CANParser
from opendbc.car import Bus, structs
from opendbc.car.hyundai import carcontroller
from opendbc.car.hyundai.values import CAR, HyundaiFlags


# Active steering feedback observed before the K7 PE camera/FCA fault.
MDPS12_ACTIVE = bytes.fromhex("472436c100386880")
LKAS11_STOCK = bytes.fromhex("0400002406014011")


@pytest.mark.parametrize(("candidate", "expected_feedback"), (
  (CAR.KIA_K7_PE, True),
  (CAR.KIA_K7, True),
  (CAR.KIA_K7_HEV_PE, False),
  (CAR.HYUNDAI_SONATA, False),
))
@pytest.mark.parametrize("long_control", (False, True))
@pytest.mark.parametrize("lat_active", (False, True))
def test_camera_mdps_feedback(candidate, expected_feedback, long_control, lat_active, monkeypatch):
  params = SimpleNamespace(get_int=lambda key: int(key == "HyundaiCameraSCC"),
                           get_bool=lambda key: False, get_float=lambda key: 0.0)
  monkeypatch.setattr(carcontroller, "Params", lambda: params)
  cp = structs.CarParams(carFingerprint=candidate, wheelbase=2.855, steerRatio=15.5,
                        flags=int(HyundaiFlags.CAMERA_SCC | HyundaiFlags.SEND_LFA | HyundaiFlags.USE_FCA),
                        openpilotLongitudinalControl=long_control)
  controller = carcontroller.CarController({Bus.pt: "hyundai_kia_generic"}, cp)
  controller.frame = 254
  controller.lkas11_active = True

  parser = CANParser("hyundai_kia_generic", [("MDPS12", 100), ("LKAS11", 100)], 0)
  parser.update([1_000_000_000, [(0x251, MDPS12_ACTIVE, 0), (0x340, LKAS11_STOCK, 0)]])
  state = SimpleNamespace(out=structs.CarState(vEgo=10.0, vEgoRaw=10.0), modelV2=None, is_metric=True,
                          lkas11=dict(parser.vl["LKAS11"]), clu11=None, paddle_button_prev=0, softHoldActive=0,
                          scc11=None, scc12=None, scc13=None, scc14=None, fca11=None)
  control = structs.CarControl(enabled=lat_active, latActive=lat_active)
  control.actuators.torque = 0.1
  control = control.as_reader()

  for frame in range(254, 258):
    # CarState refreshes this snapshot from the vehicle on every control cycle.
    state.mdps12 = dict(parser.vl["MDPS12"])
    _, messages = controller.update(control, state, frame * 10_000_000)
    feedback = [msg for msg in messages if msg[0] == 0x251]
    assert len(feedback) == int(expected_feedback)
    if feedback:
      address, data, bus = feedback[0]
      assert address == 0x251 and bus == 2
      raw = int.from_bytes(data, "little")
      assert (raw >> 13) & 1 == 0  # Camera did not request the external steering.
      assert (raw >> 12) & 1 == 1
      assert data[2] == frame % 256
      assert data[3] == (sum(data) - data[3]) % 256
      changed_fields = (3 << 12) | (0xFFFF << 16)
      assert raw & ~changed_fields == int.from_bytes(MDPS12_ACTIVE, "little") & ~changed_fields

    # Keep the real steering command on the vehicle bus; do not synthesize FCA fault suppression.
    lkas = [msg for msg in messages if msg[0] == 0x340]
    assert len(lkas) == 1 and lkas[0][2] == 0
    assert (int.from_bytes(lkas[0][1], "little") >> 27) & 1 == int(lat_active)
    assert not any(msg[0] in (0x38D, 0x483) for msg in messages)

  state.mdps12 = None
  _, messages = controller.update(control, state, 258 * 10_000_000)
  assert not any(msg[0] == 0x251 for msg in messages)
