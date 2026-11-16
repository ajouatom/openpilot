# Mapper / SideDisplay unit tests from Qualified4 ccnc-hda1 (their create_ccnc_messages fixture tests are not ported: our
# lane policy differs); our own wiring tests below.
from itertools import product
import time

import pytest

from types import SimpleNamespace

from opendbc.can import CANPacker
from opendbc.car import structs
from opendbc.car.hyundai import hyundaicanfd
from opendbc.car.hyundai import ccnc_lane_marking as lm
from opendbc.car.hyundai.values import CAR, HyundaiFlags
from opendbc.can.parser import get_raw_value

FAR = 10**18


def marking(left=(None, None), right=(None, None), expires=FAR):
  def heads(kind, color):
    return {'type': (kind, expires) if kind else None, 'color': (color, expires) if color else None}
  return {'mode': 'scanner', 'left': heads(*left), 'right': heads(*right)}


@pytest.mark.parametrize("existing,kind,color", list(product((2, 6), ('dashed', 'solid', None), ('white', 'yellow', None))))
def test_priority_yellow_then_green_then_white(existing, kind, color):
  code = lm.lane_code(existing, marking(left=(kind, color)), 'left', 0)
  if color == 'yellow':
    assert code == 4  # yellow beats lane-change-available green
  elif color == 'white' and kind == 'dashed':
    assert code == (10 if existing == 6 else 8)
  elif color == 'white' and kind == 'solid':
    assert code == existing
  else:
    assert code == existing  # unknown head: existing display policy


@pytest.mark.parametrize("existing", [0, 1, 4, 14])
def test_hidden_warning_and_other_codes_are_never_changed(existing):
  for m in (marking(left=('solid', 'yellow')), marking(left=('dashed', 'white'))):
    assert lm.lane_code(existing, m, 'left', 0) == existing


def test_expiry_none_and_sides_are_independent():
  m = marking(left=('solid', 'yellow'), right=('dashed', 'white'), expires=100)
  assert lm.lane_code(6, m, 'left', 100) == 4 and lm.lane_code(6, m, 'right', 100) == 10
  assert lm.lane_code(6, m, 'left', 101) == 6 and lm.lane_code(2, m, 'right', 101) == 2  # expired
  assert lm.lane_code(2, None, 'left', 0) == 2
  only_left = marking(left=('solid', 'yellow'))
  assert lm.lane_code(6, only_left, 'right', 0) == 6


def test_lane_change_hold_limit_switch_replays_only(monkeypatch):
  """rollback1026 H (default off): a lane-change hold with the lane prob < 0.5 for LC_HOLD_LIMIT_S ends in lapsed grey; a fresh
  value is shown at once again."""
  assert lm.LC_HOLD_LIMIT_S is None
  for limit, grey in ((None, False), (1.5, True)):
    monkeypatch.setattr(lm, 'LC_HOLD_LIMIT_S', limit)
    d = lm.SideDisplay()
    d.update(0., 20., False, .9, False, 'solid', 'yellow')
    for k in range(1, 41):  # 2 s changing lanes at 20 m/s, lane prob 0.2, nothing fresh
      out = d.update(k*.05, 20., False, .2, True, None, None)
    assert out[4] is grey and (out[3] is None) is grey and (out[3] == 'yellow') is not grey
    assert d.update(2.1, 20., False, .2, True, 'dashed', 'white')[2:5] == ('dashed', 'white', False)


def _ccnc_lanes(monkeypatch, mode, camera, mark, frame=5, display=None):
  from openpilot.cereal import log
  monkeypatch.setattr(hyundaicanfd, 'Params', lambda: SimpleNamespace(
    get_int=lambda key: mode if key == 'CcncLaneTypeDetection' else 0, get=lambda key: '0'))
  monkeypatch.setitem(hyundaicanfd.create_ccnc_messages.__dict__, '_lane_type_mode', mode)
  monkeypatch.setitem(hyundaicanfd.create_ccnc_messages.__dict__, '_lane_display', {} if display is None else display)
  packer = CANPacker('hyundai_canfd_generated')
  definition = packer.dbc.name_to_msg['ADRV_0x161']
  md = log.ModelDataV2.new_message()
  md.laneLineProbs = [0.9] * 4
  state = SimpleNamespace(
    adrv_0x161={key: 0 for key in definition.sigs}, out=structs.CarState(leftLaneLine=camera, rightLaneLine=camera, vEgo=20.),
    modelV2=md, lfahda_cluster=None, cruise_buttons_msg=None, adrv_0x200=None, adrv_0x1ea=None, ccnc_0x162=None,
    is_metric=True, paddle_button_prev=0, softHoldActive=0, trailer_connected=False, ccnc_lane_marking=mark,
  )
  control = structs.CarControl(latActive=True, enabled=True)
  control.hudControl.leftLaneVisible = control.hudControl.rightLaneVisible = True
  msg = hyundaicanfd.create_ccnc_messages(
    SimpleNamespace(flags=HyundaiFlags.CAMERA_SCC, carFingerprint=CAR.HYUNDAI_TUCSON_4TH_GEN), packer, SimpleNamespace(ECAN=0, CAM=2),
    frame, control, state, control.hudControl, 0, False, False, 0, False, 0, 0)[0]
  return get_raw_value(msg[1], definition.sigs['LANELINE_LEFT']), get_raw_value(msg[1], definition.sigs['LANELINE_RIGHT'])


def test_hda1_vision_paints_dashed_and_yellow(monkeypatch):
  # HDA1 (camera colour 0): the recogniser decides the shape / colour
  assert _ccnc_lanes(monkeypatch, 1, 0, marking(left=('dashed', 'white'), right=('solid', 'yellow'))) == (8, 4)
  assert _ccnc_lanes(monkeypatch, 2, 0, marking(left=('solid', 'white'), right=('dashed', 'white'))) == (2, 8)


def test_hda1_vision_off_or_unconfirmed(monkeypatch):
  mark = marking(left=('dashed', 'white'), right=('dashed', 'white'))
  assert _ccnc_lanes(monkeypatch, 0, 0, mark) == (2, 2)  # option off: existing display
  assert _ccnc_lanes(monkeypatch, 1, 0, None) == (14, 14)  # on, nothing confirmed since start: hidden, not a white solid


def test_camera_lane_info_wins_over_vision(monkeypatch):
  # HDA2 camera colour 1 (white) type 0 (dashed) on a car outside the dashed allow list: camera path only, solid 2
  assert _ccnc_lanes(monkeypatch, 1, 10, marking(left=('solid', 'yellow'), right=('solid', 'yellow'))) == (2, 2)


def test_hda1_vision_hold_then_lapsed_grey(monkeypatch):
  display = {}
  assert _ccnc_lanes(monkeypatch, 1, 0, marking(left=('dashed', 'white')), frame=0, display=display)[0] == 8
  assert _ccnc_lanes(monkeypatch, 1, 0, None, frame=200, display=display)[0] == 8  # held 2 s later (HOLD_S 5 s)
  frames = range(205, 800, 5)  # MAX_DT clamps each step to 0.2 s, so step through time
  for f in frames:
    left = _ccnc_lanes(monkeypatch, 1, 0, None, frame=f, display=display)[0]
  assert left == 0  # hold ran out unconfirmed: grey, not a guessed solid
