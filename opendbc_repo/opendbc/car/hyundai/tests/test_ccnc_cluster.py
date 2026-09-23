import copy
from types import SimpleNamespace

import pytest

from opendbc.can import CANPacker
from opendbc.can.parser import get_raw_value
from opendbc.car.hyundai import hyundaicanfd
from opendbc.car.hyundai.hyundaicanfd import _select_cluster_background
from opendbc.car.hyundai.values import HyundaiFlags


@pytest.mark.parametrize(
  ("cruise_enabled", "lat_active", "paddle_pressed", "paddle_mode", "expected"),
  (
    (True, True, True, 0, 1),
    (False, True, True, 0, 3),
    (False, False, True, 0, 7),
    (True, True, True, 1, 6),
    (True, True, False, 1, 1),
  ),
)
def test_paddle_background_requires_enabled_paddle_mode(
  cruise_enabled, lat_active, paddle_pressed, paddle_mode, expected,
):
  assert _select_cluster_background(cruise_enabled, lat_active, paddle_pressed, paddle_mode) == expected


@pytest.mark.parametrize("distance", (0.0, 1.6, 14.0, 14.1, 20.0, 25.5))
@pytest.mark.parametrize(("message", "detect", "expected_detect"), [
  *(('ADRV_0x1ea', detect, detect) for detect in range(8)),
  *(('CCNC_0x162', detect, expected) for detect, expected in (
    (0, 0), (1, 3), (2, 4), (3, 3), (4, 4), (5, 5), (6, 6), (7, 7),
    (8, 8), (9, 9), (10, 10), (11, 11), (12, 12), (13, 13), (14, 14),
  )),
])
def test_cluster_objects_restore_corner_state_without_blinking_or_distance_clamp(monkeypatch, message, distance, detect, expected_detect):
  monkeypatch.setattr(hyundaicanfd, "Params", lambda: SimpleNamespace(get_int=lambda key: 0))
  packer = CANPacker("hyundai_canfd_generated")
  definition = packer.dbc.name_to_msg[message]
  source = {key: 0 for key in definition.sigs}
  for side in ("LF", "RF", "LR", "RR"):
    source[f"{side}_DETECT"] = detect
    source[f"{side}_DETECT_DISTANCE"] = distance
    source[f"{side}_DETECT_LATERAL"] = 2.9
  if message == "CCNC_0x162":
    source.update(FF_DETECT=detect, FF_DISTANCE=81.2, FF_LATERAL=1.3,
                  FF_DETECT_ALT=2, FF_DISTANCE_ALT=42.1, FF_LATERAL_ALT=0.7)
  original = copy.deepcopy(source)
  state = SimpleNamespace(
    out=SimpleNamespace(brakeHoldActive=False, parkingBrake=False, steeringAngleDeg=6.0,
                        leftBlinker=False, rightBlinker=False),
    modelV2=None, lfahda_cluster=None, cruise_buttons_msg=None, adrv_0x161=None, adrv_0x200=None,
    adrv_0x1ea=source if message == "ADRV_0x1ea" else None,
    ccnc_0x162=source if message == "CCNC_0x162" else None,
    radarState=SimpleNamespace(leadOne=SimpleNamespace(status=True, dRel=81.2, yRel=-1.3, vRel=-5.0)),
  )
  cp = SimpleNamespace(flags=HyundaiFlags.CAMERA_SCC.value)
  can = SimpleNamespace(ECAN=0, CAM=2)
  control = SimpleNamespace(latActive=True, enabled=True)
  # FF uses radarState directly, independently of the older HUD distance.
  hud = SimpleNamespace(leadDistance=123.4, leadRadar=1, leadRelSpeed=-5.0)

  for frame in (0, 5, 65, 70, 100, 135, 200):
    messages = hyundaicanfd.create_ccnc_messages(cp, packer, can, frame, control, state, hud,
                                                0, False, False, 0, False, 0, 0)
    assert len(messages) == 1
    address, data, bus = messages[0]
    assert (address, bus) == (definition.address, 0)
    decoded = {key: get_raw_value(data, sig) * sig.factor + sig.offset for key, sig in definition.sigs.items()}
    assert decoded["CHECKSUM"] == hyundaicanfd.hkg_can_fd_checksum(address, None, bytearray(data))
    for side in ("LF", "RF", "LR", "RR"):
      assert decoded[f"{side}_DETECT_DISTANCE"] == pytest.approx(distance)
      assert decoded[f"{side}_DETECT_LATERAL"] == pytest.approx(2.9)
      assert decoded[f"{side}_DETECT"] == (1 if detect >= 4 and distance != 0 else detect)
    if message == "CCNC_0x162":
      assert decoded["FF_DETECT"] == (expected_detect or 4)
      for key in ("FF_DISTANCE", "FF_LATERAL", "FF_DETECT_ALT", "FF_DISTANCE_ALT", "FF_LATERAL_ALT"):
        assert decoded[key] == pytest.approx(original[key])
    else:
      assert decoded["LANELINE_CURVATURE"] == 2
    assert source == original
