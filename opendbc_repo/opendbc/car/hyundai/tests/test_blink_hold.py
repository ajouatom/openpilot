from types import SimpleNamespace as NS

from opendbc.car.hyundai.hyundaicanfd import _blink_hold_side, _select_cluster_background


def _md(y_left, y_right):
  return NS(laneLines=[NS(y=[-5.0]), NS(y=[y_left]), NS(y=[y_right]), NS(y=[5.0])])


def test_blink_hold_follows_the_confirmed_change_without_lane_lines():
  state = {}
  assert _blink_hold_side(None, 3, state) == 3                      # no model message: hold for the desire
  assert _blink_hold_side(NS(laneLines=[]), 4, state) == 4          # missing / short lane-line lists
  assert _blink_hold_side(NS(laneLines=[NS(y=[]), NS(y=[]), NS(y=[])]), 4, state) == 4
  assert _blink_hold_side(None, 0, state) == 0


def test_blink_hold_releases_once_the_body_is_across():
  # Left change, 2026-09-19 #9: the left line closes in, the model relabels it over two frames,
  # then the crossed line (now on the right) moves away from the car centre.
  state = {}
  frames = [(-1.75, 1.75), (-1.2, 2.3), (-0.6, 2.8), (-0.28, 3.04), (-1.36, 2.21), (-3.46, 0.12),
            (-3.11, 0.32)]
  assert all(_blink_hold_side(_md(*f), 3, state) == 3 for f in frames)
  assert state["crossed"] and not state.get("released")         # centre across, 0.32 m < 0.1 lane width
  assert _blink_hold_side(_md(-2.62, 0.76), 3, state) == 0          # 0.76 m past the line: over 60 %
  assert _blink_hold_side(_md(-1.9, 1.6), 3, state) == 0            # stays released for this change
  assert _blink_hold_side(_md(-1.9, 1.6), 0, state) == 0            # desire ends: reset
  assert _blink_hold_side(_md(-1.8, 1.8), 4, state) == 4            # the next change holds again


def test_blink_hold_right_change_uses_the_right_line():
  state = {}
  for y_left, y_right in [(-1.9, 1.66), (-2.5, 0.98), (-3.06, 0.17), (-3.09, 0.27), (-1.64, 1.66), (-0.21, 3.08)]:
    assert _blink_hold_side(_md(y_left, y_right), 4, state) == 4
  assert _blink_hold_side(_md(-0.33, 3.0), 4, state) == 4           # 0.33 m < 0.1 x 3.33 m: under 60 %
  assert _blink_hold_side(_md(-0.4, 2.9), 4, state) == 0            # 0.4 m >= 0.33 m: 60 % over


def test_blink_hold_without_a_visible_crossing_lasts_to_the_end_of_the_desire():
  # 2026-09-19 #11: the model lines follow the car and never come within 1 m of the centre.
  state = {}
  for y_left, y_right in [(-2.9, 1.5), (-2.4, 1.1), (-2.2, 0.8), (-1.5, 1.3), (-1.2, 1.9), (-1.7, 1.8)]:
    assert _blink_hold_side(_md(y_left, y_right), 3, state) == 3
  assert _blink_hold_side(_md(-1.7, 1.8), 0, state) == 0


def test_blink_hold_before_the_change_follows_the_lever_wait():
  # LaneChangeLeverWait: DesireHelper asks for the lamp while a lever-requested change waits (desire not 3/4 yet).
  state = {}
  assert _blink_hold_side(NS(laneLines=[], meta=NS(laneChangeBlinkerHold="left")), 0, state) == 3
  assert _blink_hold_side(NS(laneLines=[], meta=NS(laneChangeBlinkerHold="right")), 0, state) == 4
  assert _blink_hold_side(NS(laneLines=[], meta=NS(laneChangeBlinkerHold="none")), 0, state) == 0
  assert _blink_hold_side(NS(laneLines=[]), 0, state) == 0          # older schema without the field


def test_blink_hold_request_overrides_the_release_of_the_current_change():
  # a next change queued while this one crosses keeps the lamp lit past the 60 % release
  state = {}
  frames = [(-1.75, 1.75), (-0.6, 2.8), (-0.28, 3.04), (-3.46, 0.12), (-2.62, 0.76)]
  assert [_blink_hold_side(_md(*f), 3, state) for f in frames][-1] == 0      # released at 60 %
  md = NS(laneLines=_md(-2.4, 1.0).laneLines, meta=NS(laneChangeBlinkerHold="left"))
  assert _blink_hold_side(md, 3, state) == 3


def test_turn_model_flashes_the_cluster_red_like_a_held_paddle():
  assert _select_cluster_background(True, True, False, 0, turning=True) == 6       # FLASHING RED
  assert _select_cluster_background(False, True, False, 0, turning=True) == 6      # lateral only too
  assert _select_cluster_background(True, True, True, 2) == 6                      # held paddle, unchanged
  assert _select_cluster_background(True, True, False, 2) == 1                     # cruise: blue
  assert _select_cluster_background(False, True, False, 0) == 3                    # lateral only: orange
  assert _select_cluster_background(False, False, False, 0) == 7                   # gray


def test_external_hold_changes_only_existing_1ea_fields():
  from unittest.mock import patch
  from opendbc.can import CANPacker
  from opendbc.can.parser import get_raw_value
  from opendbc.car import structs
  from opendbc.car.hyundai import hyundaicanfd as h
  from opendbc.car.hyundai.values import HyundaiFlags
  packer=CANPacker('hyundai_canfd_generated')
  definition=packer.dbc.name_to_msg['ADRV_0x1ea']
  stock={key:0 for key in definition.sigs}
  stock['COUNTER']=17
  state=NS(adrv_0x1ea=stock,adrv_0x161=None,adrv_0x200=None,ccnc_0x162=None,
           out=structs.CarState(),modelV2=None,lfahda_cluster=None,cruise_buttons_msg=None)
  def message(value):
    with patch.object(h,'Params',lambda:NS(get_int=lambda _:0)):
      result=h.create_ccnc_messages(NS(flags=HyundaiFlags.CAMERA_SCC),CANPacker('hyundai_canfd_generated'),
               NS(ECAN=0,CAM=2),5,structs.CarControl(),state,NS(physicalBlinkerHold=value),0,False,False,0,False,0,0)
    assert len(result)==1 and result[0][0]==0x1ea
    return {key:get_raw_value(result[0][1],sig) for key,sig in definition.sigs.items()}
  baseline=message(0)
  for value,left,right in ((1,0,0),(2,1,0),(3,0,1)):
    actual=message(value)
    assert (actual['LEFT_BLINK_HOLD'],actual['RIGHT_BLINK_HOLD'])==(left,right)
    assert {k:v for k,v in actual.items() if k not in ('LEFT_BLINK_HOLD','RIGHT_BLINK_HOLD','CHECKSUM')} == {
      k:v for k,v in baseline.items() if k not in ('LEFT_BLINK_HOLD','RIGHT_BLINK_HOLD','CHECKSUM')}
