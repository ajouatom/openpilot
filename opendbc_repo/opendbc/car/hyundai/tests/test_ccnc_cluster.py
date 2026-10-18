import copy
from types import SimpleNamespace

import pytest

from opendbc.can import CANDefine, CANPacker
from opendbc.can.parser import get_raw_value
from opendbc.car import structs
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


@pytest.mark.parametrize('cruise_enabled', (False, True))
@pytest.mark.parametrize('lat_active', (False, True))
def test_model_turn_background_requires_active_steering(cruise_enabled, lat_active):
  normal = 1 if cruise_enabled else 7
  assert _select_cluster_background(cruise_enabled, lat_active, False, 0, turning=True) == (6 if lat_active else normal)


@pytest.mark.parametrize('desire', range(5))
@pytest.mark.parametrize('lat_active', (False, True))
def test_turn_changes_only_background_in_existing_cluster_frame(monkeypatch, desire, lat_active):
  from openpilot.cereal import log
  monkeypatch.setattr(hyundaicanfd, 'Params', lambda: SimpleNamespace(get_int=lambda key: 0, get=lambda key: '0'))
  packer = CANPacker('hyundai_canfd_generated')
  definition = packer.dbc.name_to_msg['ADRV_0x161']
  stock = {key: 0 for key in definition.sigs}
  stock['COUNTER'] = 17
  original = stock.copy()
  model = log.ModelDataV2.new_message()
  model.meta.desire = desire
  state = SimpleNamespace(
    adrv_0x161=stock, out=structs.CarState(), modelV2=model, lfahda_cluster=None,
    cruise_buttons_msg=None, adrv_0x200=None, adrv_0x1ea=None, ccnc_0x162=None,
    is_metric=True, paddle_button_prev=0, softHoldActive=0, trailer_connected=False,
  )
  control = structs.CarControl(latActive=lat_active, enabled=True)

  def messages():
    return hyundaicanfd.create_ccnc_messages(
      SimpleNamespace(flags=HyundaiFlags.CAMERA_SCC), CANPacker('hyundai_canfd_generated'),
      SimpleNamespace(ECAN=0, CAM=2), 5, control, state, control.hudControl, 0, False, False, 0, False, 0, 0)
  actual = messages()
  monkeypatch.setattr(hyundaicanfd, '_select_cluster_background',
                      lambda enabled, lat, paddle, mode, **kwargs: 6 if mode > 0 and paddle else 1 if enabled else 3 if lat else 7)
  baseline = messages()
  assert len(actual) == len(baseline) == 1
  assert (actual[0][0], actual[0][2]) == (baseline[0][0], baseline[0][2]) == (0x161, 0)

  def decode(message):
    return {key: get_raw_value(message[1], sig) for key, sig in definition.sigs.items()}
  values, old = decode(actual[0]), decode(baseline[0])
  assert values['BACKGROUND'] == (6 if lat_active and desire in (1, 2) else 1)
  assert {k: v for k, v in values.items() if k not in ('BACKGROUND', 'CHECKSUM')} == {
    k: v for k, v in old.items() if k not in ('BACKGROUND', 'CHECKSUM')}
  assert values['CHECKSUM'] == hyundaicanfd.hkg_can_fd_checksum(0x161, None, bytearray(actual[0][1]))
  assert stock == original


@pytest.mark.parametrize("distance", (0.0, 1.6, 14.0, 14.1, 20.0, 25.5))
@pytest.mark.parametrize(("message", "detect", "expected_corner", "expected_front"), [
  *(('ADRV_0x1ea', detect, expected, None) for detect, expected in (
    (0, 0), (1, 1), (2, 2), (3, 3), (4, 1), (5, 1), (6, 1), (7, 1),
  )),
  *(('CCNC_0x162', detect, 3, expected) for detect, expected in (
    (1, 3), (2, 4), (3, 3), (4, 4), (5, 5), (6, 6), (7, 7),
    (8, 8), (9, 9), (10, 10), (11, 11), (12, 12), (13, 13), (14, 14),
  )),
  # EV5 supplies nonzero corner geometry while 0x162 DETECT stays hidden.
  ('CCNC_0x162', 0, 3, 0),
])
def test_cluster_objects_restore_corner_state_without_blinking_or_distance_clamp(monkeypatch, message, distance, detect,
                                                                               expected_corner, expected_front):
  monkeypatch.setattr(hyundaicanfd, "Params", lambda: SimpleNamespace(get_int=lambda key: 0))
  packer = CANPacker("hyundai_canfd_generated")
  display_types = CANDefine("hyundai_canfd_generated").dv[message]
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
      assert decoded[f"{side}_DETECT"] == (expected_corner if distance != 0 else detect)
      if message == "CCNC_0x162" and distance != 0:
        assert display_types[f"{side}_DETECT"][decoded[f"{side}_DETECT"]] == "GRAY_CAR"
    if message == "CCNC_0x162":
      assert decoded["FF_DETECT"] == (expected_front or 4)
      for key in ("FF_DISTANCE", "FF_LATERAL", "FF_DETECT_ALT", "FF_DISTANCE_ALT", "FF_LATERAL_ALT"):
        assert decoded[key] == pytest.approx(original[key])
    else:
      assert decoded["LANELINE_CURVATURE"] == 2
    assert source == original


def test_hdp_gap_bars_and_popup(monkeypatch):
  from openpilot.cereal import log
  monkeypatch.setattr(hyundaicanfd, 'Params', lambda: SimpleNamespace(
    get_int=lambda key: 0, get=lambda key: '2' if key == 'HDPuse' else '0'))
  for attr in ('_gap_bars', '_gap_popup_until'):
    monkeypatch.delattr(hyundaicanfd.create_ccnc_messages, attr, raising=False)
  packer = CANPacker('hyundai_canfd_generated')
  definition = packer.dbc.name_to_msg['ADRV_0x161']
  state = SimpleNamespace(
    adrv_0x161={key: 0 for key in definition.sigs}, out=structs.CarState(), modelV2=log.ModelDataV2.new_message(),
    lfahda_cluster=None, cruise_buttons_msg=None, adrv_0x200=None, adrv_0x1ea=None, ccnc_0x162=None,
    is_metric=True, paddle_button_prev=0, softHoldActive=0, trailer_connected=False,
  )
  control = structs.CarControl(latActive=True, enabled=True)

  def send(frame, bars):
    control.hudControl.leadDistanceBars = bars
    msg = hyundaicanfd.create_ccnc_messages(
      SimpleNamespace(flags=HyundaiFlags.CAMERA_SCC), packer, SimpleNamespace(ECAN=0, CAM=2), frame, control, state,
      control.hudControl, 0, False, False, 0, False, 0, 0)[0]
    return {key: get_raw_value(msg[1], sig) for key, sig in definition.sigs.items()}

  first = send(5, 2)
  assert (first['DISTANCE'], first['ALERTS_3'], first['HDA_ICON']) == (2, 0, 5)  # HDP colors keep the real bars
  assert send(10, 3)['ALERTS_3'] == 9  # DISTANCE_3 popup after a change
  assert send(205, 3)['ALERTS_3'] == 9
  assert send(215, 3)['ALERTS_3'] == 0  # gone after 2 s


@pytest.mark.parametrize(('standstill', 'lka', 'lca'), ((True, 4, 2), (False, 3, 0)))
def test_engaged_standstill_keeps_steering_and_lane_change_icons(monkeypatch, standstill, lka, lca):
  from openpilot.cereal import log
  monkeypatch.setattr(hyundaicanfd, 'Params', lambda: SimpleNamespace(get_int=lambda key: 0, get=lambda key: '0'))
  packer = CANPacker('hyundai_canfd_generated')
  definition = packer.dbc.name_to_msg['ADRV_0x161']
  out = structs.CarState(standstill=standstill, latEnabled=True)
  state = SimpleNamespace(
    adrv_0x161={key: 0 for key in definition.sigs}, out=out, modelV2=log.ModelDataV2.new_message(),
    lfahda_cluster=None, cruise_buttons_msg=None, adrv_0x200=None, adrv_0x1ea=None, ccnc_0x162=None,
    is_metric=True, paddle_button_prev=0, softHoldActive=0, trailer_connected=False,
  )
  control = structs.CarControl(latActive=False, enabled=True)  # controlsd turns steering off at standstill
  msg = hyundaicanfd.create_ccnc_messages(
    SimpleNamespace(flags=HyundaiFlags.CAMERA_SCC), packer, SimpleNamespace(ECAN=0, CAM=2), 5, control, state,
    control.hudControl, 0, False, False, 0, False, 0, 0)[0]
  values = {key: get_raw_value(msg[1], sig) for key, sig in definition.sigs.items()}
  assert (values['LKA_ICON'], values['LFA_ICON'] > 1, values['LCA_LEFT_ICON'], values['LCA_RIGHT_ICON']) == (lka, standstill, lca, lca)


@pytest.mark.parametrize(('color', 'lane_line', 'expected'), (
  (6, 10, 10), (2, 10, 8), (6, 15, 10),  # white dashed / DLM inner dashed -> green or white dashed
  (2, 11, 2), (4, 10, 4), (6, 0, 6),     # solid line, orange warning, no lane info stay as they were
))
def test_dashed_lane_line_codes(color, lane_line, expected):
  assert hyundaicanfd._dashed_lane_line(color, lane_line) == expected


@pytest.mark.parametrize(('blindspot', 'left'), ((False, 14), (True, 4)))  # orange (BSD) stays even when unseen
def test_unseen_lane_lines_send_14(monkeypatch, blindspot, left):
  from openpilot.cereal import log
  monkeypatch.setattr(hyundaicanfd, 'Params', lambda: SimpleNamespace(get_int=lambda key: 0, get=lambda key: '0'))
  packer = CANPacker('hyundai_canfd_generated')
  definition = packer.dbc.name_to_msg['ADRV_0x161']
  state = SimpleNamespace(
    adrv_0x161={key: 0 for key in definition.sigs}, out=structs.CarState(leftBlindspot=blindspot), modelV2=log.ModelDataV2.new_message(),
    lfahda_cluster=None, cruise_buttons_msg=None, adrv_0x200=None, adrv_0x1ea=None, ccnc_0x162=None,
    is_metric=True, paddle_button_prev=0, softHoldActive=0, trailer_connected=False,
  )
  control = structs.CarControl(latActive=True, enabled=True)
  control.hudControl.rightLaneVisible = True
  msg = hyundaicanfd.create_ccnc_messages(
    SimpleNamespace(flags=HyundaiFlags.CAMERA_SCC), packer, SimpleNamespace(ECAN=0, CAM=2), 5, control, state,
    control.hudControl, 0, False, False, 0, False, 0, 0)[0]
  values = {key: get_raw_value(msg[1], sig) for key, sig in definition.sigs.items()}
  assert (values['LANELINE_LEFT'], values['LANELINE_RIGHT']) == (left, 2)


def test_lane_unseen_hysteresis():
  state = {}
  assert not hyundaicanfd._lane_unseen(state, 'L', 0.02, 0)
  assert not hyundaicanfd._lane_unseen(state, 'L', 0.02, 95)   # parking lot, under 1 s
  assert hyundaicanfd._lane_unseen(state, 'L', 0.02, 100)      # hidden after 1 s
  assert hyundaicanfd._lane_unseen(state, 'L', 0.10, 120)      # between thresholds keeps it hidden
  assert not hyundaicanfd._lane_unseen(state, 'L', 0.13, 125)  # city stop level shows it again
  assert not hyundaicanfd._lane_unseen(state, 'L', 0.05, 130)
  assert not hyundaicanfd._lane_unseen(state, 'L', 0.10, 200)  # a dip that does not last 1 s resets
  assert not hyundaicanfd._lane_unseen(state, 'L', 0.05, 205)
  assert not hyundaicanfd._lane_unseen(state, 'R', 0.02, 205)  # sides are independent


@pytest.mark.parametrize(('main_on', 'v_ego', 'pressed'), ((False, 5.0, 1), (True, 5.0, 0), (False, 0.0, 1)))
def test_stock_main_kept_on_without_engagement(monkeypatch, main_on, v_ego, pressed):
  from openpilot.cereal import log
  monkeypatch.setattr(hyundaicanfd, 'Params', lambda: SimpleNamespace(get_int=lambda key: 0, get=lambda key: '0'))
  packer = CANPacker('hyundai_canfd_generated')
  buttons = packer.dbc.name_to_msg['CRUISE_BUTTONS']
  state = SimpleNamespace(
    adrv_0x161=None, out=structs.CarState(vEgo=v_ego), modelV2=log.ModelDataV2.new_message(), lfahda_cluster=None,
    cruise_buttons_msg={key: 0 for key in buttons.sigs}, cruise_btns_msg_canfd='CRUISE_BUTTONS', MainMode_ACC=main_on,
    ACCMode=0, scc_control=None, adrv_0x200=None, adrv_0x1ea=None, ccnc_0x162=None, is_metric=True,
    paddle_button_prev=0, softHoldActive=0, trailer_connected=False,
  )
  control = structs.CarControl(enabled=False)  # not engaged: stock MAIN is still kept on for the cluster
  msgs = hyundaicanfd.create_ccnc_messages(
    SimpleNamespace(flags=HyundaiFlags.CAMERA_SCC), packer, SimpleNamespace(ECAN=0, CAM=2), 12, control, state,
    control.hudControl, 0, False, False, 0, False, 0, 0)
  msg = next(m for m in msgs if m[0] == buttons.address)
  assert get_raw_value(msg[1], buttons.sigs['ADAPTIVE_CRUISE_MAIN_BTN']) == pressed


def test_cluster_test_raw_bits_keep_a_valid_checksum():
  packer = CANPacker('hyundai_canfd_generated')
  definition = packer.dbc.name_to_msg['CCNC_0x162']
  msg = packer.make_can_msg('CCNC_0x162', 0, {'SPEEDLIMIT': 80})
  addr, dat, _ = hyundaicanfd._set_raw_bits(msg, hyundaicanfd.CLUSTER_TEST_BITS[7], 0b101)
  value = int.from_bytes(dat, 'little')
  assert [(value >> b) & 1 for b in range(53, 64)] == [1, 0, 1] + [0] * 8
  assert get_raw_value(dat, definition.sigs['SPEEDLIMIT']) == 80  # named fields untouched
  assert get_raw_value(dat, definition.sigs['CHECKSUM']) == hyundaicanfd.hkg_can_fd_checksum(addr, None, bytearray(dat))


@pytest.mark.parametrize(('setting', 'field', 'value'), ((210040, 'ALERTS_4', 40), (170002, 'DISTANCE', 2), (120007, 'HDA_ICON', 7)))
def test_cluster_test_overrides_0x161_fields_last(monkeypatch, setting, field, value):
  from openpilot.cereal import log
  monkeypatch.setattr(hyundaicanfd, 'Params', lambda: SimpleNamespace(
    get_int=lambda key: setting if key == 'ClusterLaneTest' else 0, get=lambda key: '2' if key == 'HDPuse' else '0'))
  packer = CANPacker('hyundai_canfd_generated')
  definition = packer.dbc.name_to_msg['ADRV_0x161']
  state = SimpleNamespace(
    adrv_0x161={key: 0 for key in definition.sigs}, out=structs.CarState(), modelV2=log.ModelDataV2.new_message(),
    lfahda_cluster=None, cruise_buttons_msg=None, adrv_0x200=None, adrv_0x1ea=None, ccnc_0x162=None,
    is_metric=True, paddle_button_prev=0, softHoldActive=0, trailer_connected=False,
  )
  control = structs.CarControl(latActive=True, enabled=True)
  control.hudControl.leadDistanceBars = 3
  msg = hyundaicanfd.create_ccnc_messages(
    SimpleNamespace(flags=HyundaiFlags.CAMERA_SCC), packer, SimpleNamespace(ECAN=0, CAM=2), 5, control, state,
    control.hudControl, 0, False, False, 0, False, 0, 0)[0]
  assert get_raw_value(msg[1], definition.sigs[field]) == value


def test_cluster_sweep_steps_every_half_second(monkeypatch):
  monkeypatch.delattr(hyundaicanfd.create_ccnc_messages, '_sweep', raising=False)
  packer = CANPacker('hyundai_canfd_generated')
  assert [hyundaicanfd._cluster_sweep(packer, 3, f) for f in (1000, 1045, 1050, 1100)] == [(0, 0), (0, 0), (1, 1), (2, 2)]
  assert hyundaicanfd._cluster_sweep(packer, 3, 1000 + 256 * 50) == (0, 0)  # SIGNS is 8 bits: wraps after 255
  assert hyundaicanfd._cluster_sweep(packer, 7, 2000) == (0, 0)             # new field restarts at 0
  assert hyundaicanfd._cluster_sweep(packer, 7, 2050) == (1, 53)            # first unnamed bit, shown as its bit number
  assert hyundaicanfd._cluster_sweep(packer, 7, 2000 + 11 * 50) == (1 << 10, 63)
