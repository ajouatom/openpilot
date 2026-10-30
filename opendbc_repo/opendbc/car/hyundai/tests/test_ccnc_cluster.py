import copy
from types import SimpleNamespace

import pytest

from opendbc.can import CANDefine, CANPacker
from opendbc.can.parser import get_raw_value
from opendbc.car import structs
from opendbc.car.hyundai import hyundaicanfd
from opendbc.car.hyundai.hyundaicanfd import _select_cluster_background, _traffic_display_state
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
  # roundabout ban (Kick and auto lane change held off) flashes red engaged or not
  assert _select_cluster_background(cruise_enabled, lat_active, False, 0, roundabout=True) == 6


def test_roundabout_hold_20s_below_60kph():
  from opendbc.car.hyundai.carstate import CarState, ROUNDABOUT_HOLD_FRAMES
  cs = SimpleNamespace(ccnc_0x162={'SIGNS': 0}, ccnc_signs_prev=0, roundabout_frames=0, roundabout_hold_enabled=True)
  ret = SimpleNamespace(vEgo=0.)
  def run(n, signs=0, kph=30):
    cs.ccnc_0x162['SIGNS'], ret.vEgo = signs, kph / 3.6
    out = []
    for _ in range(n):
      CarState._update_roundabout(cs, ret)
      out.append(ret.naviRoundabout)
    return out
  assert not any(run(10))
  assert all(run(500, signs=24))                   # the 5 s pulse
  assert all(run(ROUNDABOUT_HOLD_FRAMES - 501))    # held to 20 s from the rise
  assert not any(run(10))
  run(1, signs=24)
  assert not any(run(10, signs=24, kph=80))        # highway pre-notice
  cs.roundabout_hold_enabled = False              # RoundaboutLaneChangeHold off
  assert not any(run(10, kph=30))


def test_tunnel_ban_only_after_a_no_lane_change_notice():
  from opendbc.car.hyundai.carstate import CarState, TUNNEL_NOTICE_FRAMES
  cs = SimpleNamespace(ccnc_0x162={'SIGNS': 0}, hda_info_4a3={'TunnelExist': 0}, tunnel_notice_frames=0, tunnel_banned=False,
                       tunnel_hold_enabled=True)
  ret = SimpleNamespace()
  def run(n, tunnel=0, signs=0):
    cs.hda_info_4a3['TunnelExist'], cs.ccnc_0x162['SIGNS'] = tunnel, signs
    out = []
    for _ in range(n):
      CarState._update_tunnel(cs, ret)
      out.append(ret.naviTunnel)
    return out
  assert not any(run(300, tunnel=1))               # 10-30 12:58/13:14 underpasses: no notice, no ban
  run(10)
  run(500, signs=16)                               # notice 8 s before the 13:24 tunnel
  run(300)
  assert all(run(TUNNEL_NOTICE_FRAMES, tunnel=1))  # held to the exit, past the notice window
  assert not any(run(10))
  assert all(run(10, tunnel=1, signs=16))          # a notice inside the tunnel counts too
  run(10)
  run(TUNNEL_NOTICE_FRAMES + 10)
  assert not any(run(10, tunnel=1))                # notice too long ago
  run(10)
  run(10, signs=16)
  cs.tunnel_hold_enabled = False                   # TunnelLaneChangeHold off
  assert not any(run(10, tunnel=1))


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
      box_at_zero = message == "CCNC_0x162" and detect in (1, 2)  # boxes become a gray car even without distance
      assert decoded[f"{side}_DETECT"] == (expected_corner if distance != 0 else 3 if box_at_zero else detect)
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
  assert first['DISTANCE_SPACING'] == 3 and send(300, 4)['DISTANCE_SPACING'] == 5  # 19-3 for gaps 1-3 only


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
  for car in hyundaicanfd.NO_DASHED_LANE_CARS:  # Ioniq 9 / EV9 clusters draw nothing for dashed codes
    assert hyundaicanfd._dashed_lane_line(color, lane_line, car) == color


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


@pytest.mark.parametrize(('lane_change', 'camera', 'expected'), (('off', 10, 8), ('laneChangeStarting', 10, 8), ('off', 0, 14)))
def test_lane_hidden_only_when_model_and_camera_miss(monkeypatch, lane_change, camera, expected):
  # the model has already hidden the line (probs 0.01 for 1 s); a camera lane keeps it shown
  from openpilot.cereal import log
  monkeypatch.setattr(hyundaicanfd, 'Params', lambda: SimpleNamespace(get_int=lambda key: 0, get=lambda key: '0'))
  monkeypatch.setitem(hyundaicanfd.create_ccnc_messages.__dict__, '_lane_unseen', {'L': (True, 0), 'R': (True, 0)})
  packer = CANPacker('hyundai_canfd_generated')
  definition = packer.dbc.name_to_msg['ADRV_0x161']
  md = log.ModelDataV2.new_message()
  md.laneLineProbs = [0.01] * 4
  md.meta.laneChangeState = lane_change
  state = SimpleNamespace(
    adrv_0x161={key: 0 for key in definition.sigs}, out=structs.CarState(leftLaneLine=camera, rightLaneLine=camera), modelV2=md,
    lfahda_cluster=None, cruise_buttons_msg=None, adrv_0x200=None, adrv_0x1ea=None, ccnc_0x162=None,
    is_metric=True, paddle_button_prev=0, softHoldActive=0, trailer_connected=False,
  )
  control = structs.CarControl(latActive=True, enabled=True)
  control.hudControl.leftLaneVisible = control.hudControl.rightLaneVisible = True
  msg = hyundaicanfd.create_ccnc_messages(
    SimpleNamespace(flags=HyundaiFlags.CAMERA_SCC), packer, SimpleNamespace(ECAN=0, CAM=2), 5, control, state,
    control.hudControl, 0, False, False, 0, False, 0, 0)[0]
  assert get_raw_value(msg[1], definition.sigs['LANELINE_LEFT']) == expected
  assert get_raw_value(msg[1], definition.sigs['LANELINE_RIGHT']) == expected


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


@pytest.mark.parametrize(('slot', 'field', 'value'), ((21, 'ALERTS_4', 40), (17, 'DISTANCE', 2), (12, 'HDA_ICON', 7), (28, 'ALERTS_2', 5)))
def test_cluster_test_overrides_0x161_fields_last(monkeypatch, slot, field, value):
  from openpilot.cereal import log
  monkeypatch.setattr(hyundaicanfd, 'Params', lambda: SimpleNamespace(
    get_int=lambda key: {'ClusterLaneTest': slot, 'ClusterLaneTestValue': value}.get(key, 0),
    get=lambda key: '2' if key == 'HDPuse' else '0'))
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


def test_cluster_test_raw_0x161_bits(monkeypatch):
  from openpilot.cereal import log
  monkeypatch.setattr(hyundaicanfd, 'Params', lambda: SimpleNamespace(
    get_int=lambda key: {'ClusterLaneTest': 33, 'ClusterLaneTestValue': 0b11}.get(key, 0), get=lambda key: '0'))
  packer = CANPacker('hyundai_canfd_generated')
  definition = packer.dbc.name_to_msg['ADRV_0x161']
  state = SimpleNamespace(
    adrv_0x161={key: 0 for key in definition.sigs}, out=structs.CarState(), modelV2=log.ModelDataV2.new_message(),
    lfahda_cluster=None, cruise_buttons_msg=None, adrv_0x200=None, adrv_0x1ea=None, ccnc_0x162=None,
    is_metric=True, paddle_button_prev=0, softHoldActive=0, trailer_connected=False,
  )
  control = structs.CarControl(latActive=True, enabled=True)
  addr, dat, _ = hyundaicanfd.create_ccnc_messages(
    SimpleNamespace(flags=HyundaiFlags.CAMERA_SCC), packer, SimpleNamespace(ECAN=0, CAM=2), 5, control, state,
    control.hudControl, 0, False, False, 0, False, 0, 0)[0]
  value = int.from_bytes(dat, 'little')
  assert [(value >> b) & 1 for b in range(246, 256)] == [1, 1] + [0] * 8
  assert get_raw_value(dat, definition.sigs['CHECKSUM']) == hyundaicanfd.hkg_can_fd_checksum(addr, None, bytearray(dat))


def test_cluster_sweep_steps_every_second(monkeypatch):
  monkeypatch.delattr(hyundaicanfd.create_ccnc_messages, '_sweep', raising=False)
  packer = CANPacker('hyundai_canfd_generated')
  assert [hyundaicanfd._cluster_sweep(packer, 3, f) for f in (1000, 1095, 1100, 1200)] == [(0, 0), (0, 0), (1, 1), (2, 2)]
  assert hyundaicanfd._cluster_sweep(packer, 3, 1000 + 256 * 100) == (0, 0)  # SIGNS is 8 bits: wraps after 255
  assert hyundaicanfd._cluster_sweep(packer, 7, 2000) == (0, 0)             # new field restarts at 0
  assert hyundaicanfd._cluster_sweep(packer, 7, 2100) == (1, 53)            # first unnamed bit, shown as its bit number
  assert hyundaicanfd._cluster_sweep(packer, 7, 2000 + 11 * 100) == (1 << 10, 63)


def test_cluster_sweep_faster_steps(monkeypatch):
  # ClusterLaneTestValue -2/-3/-4: 0.5 / 0.25 / 0.1 s per step; changing the speed restarts the sweep at 0
  monkeypatch.delattr(hyundaicanfd.create_ccnc_messages, '_sweep', raising=False)
  packer = CANPacker('hyundai_canfd_generated')
  assert hyundaicanfd.CLUSTER_TEST_SWEEP_FRAMES == {-1: 100, -2: 50, -3: 25, -4: 10}
  assert [hyundaicanfd._cluster_sweep(packer, 3, f, 10) for f in (1000, 1005, 1010, 1030)] == [(0, 0), (0, 0), (1, 1), (3, 3)]
  assert hyundaicanfd._cluster_sweep(packer, 3, 1030, 25) == (0, 0)
  assert hyundaicanfd._cluster_sweep(packer, 3, 1055, 25) == (1, 1)


@pytest.mark.parametrize("traffic_state, expected", [(0, 1), (1, 5), (2, 8)])
def test_traffic_light_background(traffic_state, expected):
  # turning (6) is overridden too while a light is detected
  assert _select_cluster_background(True, True, False, 0, traffic_state=traffic_state) == expected
  assert _select_cluster_background(True, True, False, 0, turning=True, traffic_state=traffic_state) == (6 if traffic_state == 0 else expected)


def test_traffic_green_shown_only_two_seconds():
  _traffic_display_state.last, _traffic_display_state.green_until = 0, 0
  assert _traffic_display_state(1, 0) == 1      # red stays
  assert _traffic_display_state(1, 500) == 1
  assert _traffic_display_state(2, 1000) == 2   # turned green
  assert _traffic_display_state(2, 1199) == 2
  assert _traffic_display_state(2, 1200) == 0   # 2 s later: back to normal
  assert _traffic_display_state(0, 1300) == 0
  assert _traffic_display_state(2, 1400) == 2   # a new green shows again


@pytest.mark.parametrize(('slot', 'name', 'sig', 'value'), (
  (55, 'ADRV_0x161', 'SOUNDS_1', 3), (66, 'CCNC_0x162', 'FF_DISTANCE', 300), (89, 'CCNC_0x162', 'FAULT_SCC', 2),
  (121, 'ADRV_0x1ea', 'AUTOLANECHANGE_MSG', 5), (139, 'ADRV_0x200', 'TauGapSet', 2), (145, 'ADRV_0x200', None, 0b101)))
def test_cluster_test_every_message(monkeypatch, slot, name, sig, value):
  # raw value lands as-is in the packed message (scaled signals included), raw bit groups too
  from openpilot.cereal import log
  monkeypatch.setattr(hyundaicanfd, 'Params', lambda: SimpleNamespace(
    get_int=lambda key: {'ClusterLaneTest': slot, 'ClusterLaneTestValue': value}.get(key, 0), get=lambda key: '0'))
  packer = CANPacker('hyundai_canfd_generated')
  zeros = lambda msg: {key: 0 for key in packer.dbc.name_to_msg[msg].sigs if key not in ('CHECKSUM', 'COUNTER')}
  state = SimpleNamespace(
    adrv_0x161=zeros('ADRV_0x161'), adrv_0x200=zeros('ADRV_0x200'), adrv_0x1ea=zeros('ADRV_0x1ea'), ccnc_0x162=zeros('CCNC_0x162'),
    out=structs.CarState(), modelV2=log.ModelDataV2.new_message(), lfahda_cluster=None, cruise_buttons_msg=None,
    is_metric=True, paddle_button_prev=0, softHoldActive=0, trailer_connected=False,
  )
  control = structs.CarControl(latActive=True, enabled=True)
  msgs = hyundaicanfd.create_ccnc_messages(
    SimpleNamespace(flags=HyundaiFlags.CAMERA_SCC), packer, SimpleNamespace(ECAN=0, CAM=2), 5, control, state,
    control.hudControl, 0, False, False, 0, False, 0, 0)
  definition = packer.dbc.name_to_msg[name]
  addr, dat, _ = next(m for m in msgs if m[0] == definition.address)
  if sig is None:
    bits = hyundaicanfd.CLUSTER_TEST_BITS_200[slot]
    raw = int.from_bytes(dat, 'little')
    assert [(raw >> b) & 1 for b in bits] == [1, 0, 1] + [0] * (len(bits) - 3)
  else:
    assert get_raw_value(dat, definition.sigs[sig]) == value
  assert get_raw_value(dat, definition.sigs['CHECKSUM']) == hyundaicanfd.hkg_can_fd_checksum(addr, None, bytearray(dat))


def test_cluster_test_slots_are_unique_and_named():
  packer = CANPacker('hyundai_canfd_generated')
  slots = [s for fields, bits in hyundaicanfd.CLUSTER_TEST_MSGS.values() for s in (*fields, *bits)]
  assert len(slots) == len(set(slots)) and sorted(slots) == list(range(1, 174))
  for name, (fields, _) in hyundaicanfd.CLUSTER_TEST_MSGS.items():
    assert set(fields.values()) <= set(packer.dbc.name_to_msg[name].sigs)


@pytest.mark.parametrize(('lat_hud', 'desire', 'color'), ((0, 0, 0), (1, 0, 1), (2, 0, 3), (3, 0, 4), (4, 0, 5), (4, 3, 1), (2, 4, 1), (0, 3, 0)))
@pytest.mark.parametrize('left_visible', (True, False))
def test_lane_highlight_follows_lateral_state(monkeypatch, lat_hud, desire, color, left_visible):
  from openpilot.cereal import log
  monkeypatch.setattr(hyundaicanfd, 'Params', lambda: SimpleNamespace(get_int=lambda key: 0, get=lambda key: '0'))
  packer = CANPacker('hyundai_canfd_generated')
  definition = packer.dbc.name_to_msg['ADRV_0x161']
  state = SimpleNamespace(
    adrv_0x161={key: 0 for key in definition.sigs}, out=structs.CarState(), modelV2=log.ModelDataV2.new_message(),
    lfahda_cluster=None, cruise_buttons_msg=None, adrv_0x200=None, adrv_0x1ea=None, ccnc_0x162=None,
    is_metric=True, paddle_button_prev=0, softHoldActive=0, trailer_connected=False,
  )
  control = structs.CarControl(latActive=lat_hud > 0, enabled=True)
  control.hudControl.latHudState = lat_hud
  state.modelV2.meta.desire = desire  # lane change takes priority
  control.hudControl.leftLaneVisible, control.hudControl.rightLaneVisible = left_visible, True
  color = color if left_visible else 0  # a hidden lane line (14) drops the highlight
  msg = hyundaicanfd.create_ccnc_messages(
    SimpleNamespace(flags=HyundaiFlags.CAMERA_SCC), packer, SimpleNamespace(ECAN=0, CAM=2), 5, control, state,
    control.hudControl, 0, False, False, 0, False, 0, 0)[0]
  assert get_raw_value(msg[1], definition.sigs['LANE_HIGHLIGHT']) == color  # 0: stock value kept
  assert get_raw_value(msg[1], definition.sigs['LANE_HIGHLIGHT_DISTANCE']) == 0


@pytest.mark.parametrize(('enabled', 'desired', 'expected'), ((True, 40.0, 400), (True, 0.0, 573), (False, 40.0, 573)))
def test_target_ring_marks_the_target_gap(monkeypatch, enabled, desired, expected):
  # engaged: openpilot's desired distance; otherwise (or without one) the stock target gap stays
  from openpilot.cereal import log
  monkeypatch.setattr(hyundaicanfd, 'Params', lambda: SimpleNamespace(get_int=lambda key: 0, get=lambda key: '0'))
  packer = CANPacker('hyundai_canfd_generated')
  definition = packer.dbc.name_to_msg['ADRV_0x161']
  stock = {key: 0 for key in definition.sigs}
  stock['TARGET_DISTANCE'] = 57.3
  state = SimpleNamespace(
    adrv_0x161=stock, out=structs.CarState(), modelV2=log.ModelDataV2.new_message(),
    lfahda_cluster=None, cruise_buttons_msg=None, adrv_0x200=None, adrv_0x1ea=None, ccnc_0x162=None,
    is_metric=True, paddle_button_prev=0, softHoldActive=0, trailer_connected=False,
  )
  control = structs.CarControl(latActive=True, enabled=enabled)
  control.hudControl.desiredDistance = desired
  msg = hyundaicanfd.create_ccnc_messages(
    SimpleNamespace(flags=HyundaiFlags.CAMERA_SCC), packer, SimpleNamespace(ECAN=0, CAM=2), 5, control, state,
    control.hudControl, 0, False, False, 0, False, 0, 0)[0]
  assert get_raw_value(msg[1], definition.sigs['TARGET_DISTANCE']) == expected


@pytest.mark.parametrize(("car", "enabled", "lat_active", "test_field", "expected"), (
  ("HYUNDAI_IONIQ_5_PE", True, False, 0, 1),   # cruise engaged
  ("HYUNDAI_IONIQ_5_PE", False, True, 0, 1),   # steering only
  ("HYUNDAI_IONIQ_5_PE", False, False, 0, 0),
  ("HYUNDAI_IONIQ_5_PE", True, True, 9, 0),    # cluster test on its bit group wins
  ("KIA_EV9", True, True, 0, 0),               # other clusters unchecked
))
def test_hdp_icon_while_engaged(monkeypatch, car, enabled, lat_active, test_field, expected):
  monkeypatch.setattr(hyundaicanfd, "Params", lambda: SimpleNamespace(get_int=lambda key: {'ClusterLaneTest': test_field}.get(key, 0)))
  packer = CANPacker("hyundai_canfd_generated")
  definition = packer.dbc.name_to_msg["CCNC_0x162"]
  state = SimpleNamespace(
    out=SimpleNamespace(brakeHoldActive=False, parkingBrake=False, steeringAngleDeg=0.0, leftBlinker=False, rightBlinker=False),
    modelV2=None, lfahda_cluster=None, cruise_buttons_msg=None, adrv_0x161=None, adrv_0x200=None, adrv_0x1ea=None,
    ccnc_0x162={key: 0 for key in definition.sigs}, radarState=None,
  )
  cp = SimpleNamespace(flags=HyundaiFlags.CAMERA_SCC.value, carFingerprint=getattr(hyundaicanfd.CAR, car))
  control = SimpleNamespace(latActive=lat_active, enabled=enabled)
  hud = SimpleNamespace(leadDistance=0, leadRadar=0, leadRelSpeed=0)
  (address, data, bus), = hyundaicanfd.create_ccnc_messages(cp, packer, SimpleNamespace(ECAN=0, CAM=2), 0, control, state, hud,
                                                            0, False, False, 0, False, 0, 0)
  assert (data[hyundaicanfd.HDP_ICON_BIT // 8] >> (hyundaicanfd.HDP_ICON_BIT % 8)) & 1 == expected
  assert get_raw_value(data, definition.sigs['CHECKSUM']) == hyundaicanfd.hkg_can_fd_checksum(address, None, bytearray(data))


def test_engage_and_disengage_popups(monkeypatch):
  from openpilot.cereal import log
  monkeypatch.setattr(hyundaicanfd, 'Params', lambda: SimpleNamespace(get_int=lambda key: 0, get=lambda key: '0'))
  for attr in ('_was_enabled', '_engage_popup'):
    monkeypatch.delattr(hyundaicanfd.create_ccnc_messages, attr, raising=False)
  packer = CANPacker('hyundai_canfd_generated')
  definition = packer.dbc.name_to_msg['ADRV_0x161']
  state = SimpleNamespace(
    adrv_0x161={key: 0 for key in definition.sigs}, out=structs.CarState(latEnabled=True), modelV2=log.ModelDataV2.new_message(),
    lfahda_cluster=None, cruise_buttons_msg=None, adrv_0x200=None, adrv_0x1ea=None, ccnc_0x162=None,
    is_metric=True, paddle_button_prev=0, softHoldActive=0, trailer_connected=False,
  )
  cp = SimpleNamespace(flags=HyundaiFlags.CAMERA_SCC, carFingerprint=hyundaicanfd.CAR.HYUNDAI_IONIQ_5_PE)

  def alerts_5(frame, enabled):
    control = structs.CarControl(latActive=enabled, enabled=enabled)
    msg = hyundaicanfd.create_ccnc_messages(cp, packer, SimpleNamespace(ECAN=0, CAM=2), frame, control, state,
                                            control.hudControl, 0, False, False, 0, False, 0, 0)[0]
    return get_raw_value(msg[1], definition.sigs['ALERTS_5'])

  assert alerts_5(0, False) == 0
  assert [alerts_5(f, True) for f in (100, 295, 300)] == [19, 19, 0]     # engage: 2 s
  assert [alerts_5(f, False) for f in (500, 695, 700)] == [21, 21, 0]    # disengage: 2 s


@pytest.mark.parametrize(("hazard", "test_field", "test_value", "expected"), (
  (1, 0, 0, 1), (0, 0, 0, 0),
  (1, 129, 0, 0),    # cluster test on bits 99-109 owns bit 104
  (0, 129, 32, 1),   # ...and can set it itself
))
def test_hazard_test_sets_0x1ea_bit_104(monkeypatch, hazard, test_field, test_value, expected):
  params = {'CanfdHazardTest': hazard, 'ClusterLaneTest': test_field, 'ClusterLaneTestValue': test_value}
  monkeypatch.setattr(hyundaicanfd, "Params", lambda: SimpleNamespace(get_int=lambda key: params.get(key, 0)))
  packer = CANPacker("hyundai_canfd_generated")
  definition = packer.dbc.name_to_msg["ADRV_0x1ea"]
  state = SimpleNamespace(
    out=SimpleNamespace(brakeHoldActive=False, parkingBrake=False, steeringAngleDeg=0.0, leftBlinker=False, rightBlinker=False),
    modelV2=None, lfahda_cluster=None, cruise_buttons_msg=None, adrv_0x161=None, adrv_0x200=None, ccnc_0x162=None,
    adrv_0x1ea={key: 0 for key in definition.sigs}, radarState=None,
  )
  control = SimpleNamespace(latActive=False, enabled=False)
  hud = SimpleNamespace(leadDistance=0, leadRadar=0, leadRelSpeed=0)
  (address, data, bus), = hyundaicanfd.create_ccnc_messages(SimpleNamespace(flags=HyundaiFlags.CAMERA_SCC.value), packer,
                                                            SimpleNamespace(ECAN=0, CAM=2), 0, control, state, hud,
                                                            0, False, False, 0, False, 0, 0)
  assert (data[13] >> 0) & 1 == expected  # bit 104 = byte 13 bit 0
  assert get_raw_value(data, definition.sigs['CHECKSUM']) == hyundaicanfd.hkg_can_fd_checksum(address, None, bytearray(data))


@pytest.mark.parametrize(("alert_type", "enabled", "field", "expected"), (
  ("speedTooHigh/warning", True, "ALERTS_4", 83),
  ("steerUnavailable/immediateDisable", False, "ALERTS_5", 9),   # beats the disengage popup (21)
  ("canError/immediateDisable", False, "ALERTS_4", 147),
  ("reverseGear/userDisable", False, "ALERTS_4", 129),
  ("reverseGear/permanent", False, "ALERTS_4", 0),               # just parked in R
  ("driverDistracted3/permanent", True, "ALERTS_4", 0),          # DM is left alone
))
def test_openpilot_alerts_map_to_cluster_texts(monkeypatch, alert_type, enabled, field, expected):
  from openpilot.cereal import log
  monkeypatch.setattr(hyundaicanfd, 'Params', lambda: SimpleNamespace(get_int=lambda key: 0, get=lambda key: '0'))
  for attr in ('_was_enabled', '_engage_popup'):
    monkeypatch.delattr(hyundaicanfd.create_ccnc_messages, attr, raising=False)
  packer = CANPacker('hyundai_canfd_generated')
  definition = packer.dbc.name_to_msg['ADRV_0x161']
  state = SimpleNamespace(
    adrv_0x161={key: 0 for key in definition.sigs}, out=structs.CarState(latEnabled=True), modelV2=log.ModelDataV2.new_message(),
    lfahda_cluster=None, cruise_buttons_msg=None, adrv_0x200=None, adrv_0x1ea=None, ccnc_0x162=None,
    is_metric=True, paddle_button_prev=0, softHoldActive=0, trailer_connected=False,
  )
  cp = SimpleNamespace(flags=HyundaiFlags.CAMERA_SCC, carFingerprint=hyundaicanfd.CAR.HYUNDAI_IONIQ_5_PE)
  for frame, en, alert in ((0, True, ""), (100, enabled, alert_type)):  # second frame: disengaged here if not enabled
    control = structs.CarControl(latActive=en, enabled=en)
    control.hudControl.alertType = alert
    msg = hyundaicanfd.create_ccnc_messages(cp, packer, SimpleNamespace(ECAN=0, CAM=2), frame, control, state,
                                            control.hudControl, 0, False, False, 0, False, 0, 0)[0]
  assert get_raw_value(msg[1], definition.sigs[field]) == expected


def test_tunnel_lane_change_refusal_text(monkeypatch):
  from openpilot.cereal import log
  monkeypatch.setattr(hyundaicanfd, 'Params', lambda: SimpleNamespace(get_int=lambda key: 0, get=lambda key: '0'))
  for attr in ('_was_enabled', '_engage_popup', '_tunnel_refusal_until'):
    monkeypatch.delattr(hyundaicanfd.create_ccnc_messages, attr, raising=False)
  packer = CANPacker('hyundai_canfd_generated')
  definition = packer.dbc.name_to_msg['ADRV_0x161']
  cp = SimpleNamespace(flags=HyundaiFlags.CAMERA_SCC, carFingerprint=hyundaicanfd.CAR.HYUNDAI_IONIQ_5_PE)

  def alerts_4(frame, blinker, tunnel=1, desire=0):
    md = log.ModelDataV2.new_message()
    md.meta.desire = desire
    state = SimpleNamespace(
      adrv_0x161={key: 0 for key in definition.sigs}, out=structs.CarState(latEnabled=True, leftBlinker=blinker, naviTunnel=bool(tunnel)), modelV2=md,
      lfahda_cluster=None, cruise_buttons_msg=None, adrv_0x200=None, adrv_0x1ea=None, ccnc_0x162=None, is_metric=True,
      paddle_button_prev=0, softHoldActive=0, trailer_connected=False,
    )
    control = structs.CarControl(latActive=True, enabled=True)
    msg = hyundaicanfd.create_ccnc_messages(cp, packer, SimpleNamespace(ECAN=0, CAM=2), frame, control, state,
                                            control.hudControl, 0, False, False, 0, False, 0, 0)[0]
    return get_raw_value(msg[1], definition.sigs['ALERTS_4'])

  assert alerts_4(0, False) == 0
  assert alerts_4(100, True) == 215                 # lever in a tunnel: waiting, say why
  assert alerts_4(295, False) == 215                # kept 2 s after the lamp stops
  assert alerts_4(300, False) == 0
  assert alerts_4(400, True, tunnel=0) == 0         # outside a tunnel
  assert alerts_4(500, True) == 215
  assert alerts_4(505, True, desire=3) == 0         # driver torque started the change


def test_auto_slot_sweep_walks_the_range(monkeypatch):
  # ClusterLaneTest -1: From..To, one value sweep per slot (bit group 9 = 9 bits + 0 -> 10 steps; SIGNS 3 = 256 steps)
  params = {'ClusterLaneTestFrom': 3, 'ClusterLaneTestTo': 9}
  monkeypatch.setattr(hyundaicanfd, 'Params', lambda: SimpleNamespace(get_int=lambda key: params.get(key, 0)))
  monkeypatch.delattr(hyundaicanfd.create_ccnc_messages, '_auto_slot', raising=False)
  packer = CANPacker('hyundai_canfd_generated')
  slot = lambda f: hyundaicanfd._cluster_auto_slot(packer, f, -4)   # 0.1 s steps = 10 frames
  assert slot(0) == 3
  assert slot(2550) == 3 and slot(2560) == 4                         # 256 steps * 10 frames on SIGNS
  params['ClusterLaneTestFrom'] = 9
  assert slot(3000) == 9 and slot(3090) == 9 and slot(3100) == 9    # range changed: restart; one slot wraps onto itself
  assert hyundaicanfd._cluster_steps(packer, 9) == 10


@pytest.mark.parametrize(("gear", "standstill", "expected"), (
  (structs.CarState.GearShifter.park, True, 5),
  (structs.CarState.GearShifter.drive, True, 0),    # not parked: control frames untouched
  (structs.CarState.GearShifter.park, False, 0),
))
def test_cluster_test_on_scc_control_only_parked(monkeypatch, gear, standstill, expected):
  monkeypatch.setattr(hyundaicanfd.create_ccnc_messages, '_active_test', (146, 5), raising=False)  # InfoDisplay
  packer = CANPacker('hyundai_canfd_generated')
  definition = packer.dbc.name_to_msg['SCC_CONTROL']
  scc = packer.make_can_msg('SCC_CONTROL', 0, {'aReqValue': -1.0})
  other = packer.make_can_msg('ADRV_0x161', 0, {})
  CS = SimpleNamespace(out=SimpleNamespace(standstill=standstill, gearShifter=gear))
  out = hyundaicanfd.apply_cluster_test_control_msgs(packer, [scc, other], CS)
  address, data, _ = out[0]
  assert get_raw_value(data, definition.sigs['InfoDisplay']) == expected
  assert get_raw_value(data, definition.sigs['aReqValue']) == get_raw_value(scc[1], definition.sigs['aReqValue'])
  assert get_raw_value(data, definition.sigs['CHECKSUM']) == hyundaicanfd.hkg_can_fd_checksum(address, None, bytearray(data))
  assert out[1][1] == other[1]
