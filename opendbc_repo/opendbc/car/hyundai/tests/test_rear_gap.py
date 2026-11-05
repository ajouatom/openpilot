from types import SimpleNamespace

import pytest

from opendbc.car.hyundai.rear_gap import REAR_INPUT_MAX_AGE_NS, rear_distance_blocks, rear_lane_change_blocks


def inputs(distance=20.0, status=1):
  now = 1_000_000_000
  values = {f"{side}_{field}": value for side in ("LR", "RR")
            for field, value in (("DETECT", status), ("DETECT_DISTANCE", distance))}
  camera = SimpleNamespace(_last_update_nanos=now, bus_timeout=False,
                           ts_nanos={name: {"COUNTER": now} for name in ("ADRV_0x1ea", "CCNC_0x162")},
                           dat={0x1EA: bytes(32), 0x162: bytes(32)},
                           vl={name: dict(values) for name in ("ADRV_0x1ea", "CCNC_0x162")})
  bsm = SimpleNamespace(_last_update_nanos=now, bus_timeout=False,
                        ts_nanos={"BLINDSPOTS_REAR_CORNERS": {"COUNTER": now}}, dat={0x1BA: bytes(24)},
                        vl={"BLINDSPOTS_REAR_CORNERS": dict.fromkeys((
                          "FL_INDICATOR", "INDICATOR_LEFT_TWO", "INDICATOR_LEFT_FOUR",
                          "FR_INDICATOR", "INDICATOR_RIGHT_TWO", "INDICATOR_RIGHT_FOUR"), 0)})
  return camera, bsm, ("ADRV_0x1ea", "CCNC_0x162")


@pytest.mark.parametrize('distance,blocked', [
  (0.0, True), (5.0, True), (19.9, True), (20.0, False), (20.1, False), (24.8, False),
  (24.9, True), (25.0, True), (25.5, True), (50.0, True), (-1.0, True), (float('nan'), True), (float('inf'), True),
])
def test_threshold_and_unknown_distance(distance, blocked):
  assert rear_lane_change_blocks(*inputs(distance)) == (blocked, blocked)


@pytest.mark.parametrize('field', ['FL_INDICATOR', 'INDICATOR_LEFT_TWO', 'INDICATOR_LEFT_FOUR',
                                  'FR_INDICATOR', 'INDICATOR_RIGHT_TWO', 'INDICATOR_RIGHT_FOUR'])
def test_bsd_always_vetoes_twenty_metre_gap(field):
  camera, bsm, sources = inputs()
  bsm.vl['BLINDSPOTS_REAR_CORNERS'][field] = 1
  left = field.startswith('FL') or 'LEFT' in field
  assert rear_lane_change_blocks(camera, bsm, sources) == (left, not left)


@pytest.mark.parametrize('name,state', [('ADRV_0x1ea', 4), ('CCNC_0x162', 30), ('CCNC_0x162', 31)])
def test_hidden_slot_is_not_clear(name, state):
  camera, bsm, sources = inputs()
  camera.vl[name]['RR_DETECT'] = state
  assert rear_lane_change_blocks(camera, bsm, sources) == (False, True)


def test_no_detection_is_not_a_positive_distance_measurement():
  assert rear_lane_change_blocks(*inputs(0, 0)) == (False, False)
  assert rear_lane_change_blocks(*inputs(20, 0)) == (True, True)


def test_disagreement_keeps_near_obstacle_instead_of_max_distance():
  camera, bsm, sources = inputs(23)
  camera.vl['CCNC_0x162']['RR_DETECT_DISTANCE'] = 3
  assert rear_lane_change_blocks(camera, bsm, sources) == (False, True)


@pytest.mark.parametrize('which,name,address', [('camera', 'ADRV_0x1ea', 0x1EA),
                                              ('camera', 'CCNC_0x162', 0x162),
                                              ('bsm', 'BLINDSPOTS_REAR_CORNERS', 0x1BA)])
@pytest.mark.parametrize('fault', ['old', 'future', 'missing_stamp', 'short_frame', 'bus_timeout'])
def test_stale_or_invalid_source_cannot_clear(which, name, address, fault):
  camera, bsm, sources = inputs()
  parser = camera if which == 'camera' else bsm
  if fault == 'old':
    parser.ts_nanos[name]['COUNTER'] -= REAR_INPUT_MAX_AGE_NS + 1
  elif fault == 'future':
    parser.ts_nanos[name]['COUNTER'] += 1
  elif fault == 'missing_stamp':
    parser.ts_nanos[name] = {}
  elif fault == 'short_frame':
    parser.dat[address] = b''
  else:
    parser.bus_timeout = True
  assert rear_lane_change_blocks(camera, bsm, sources) == (True, True)


def test_missing_bsd_and_missing_decoded_fields():
  camera, bsm, sources = inputs()
  assert rear_lane_change_blocks(camera, None, sources) == (True, True)
  del bsm.vl['BLINDSPOTS_REAR_CORNERS']['FR_INDICATOR']
  assert rear_lane_change_blocks(camera, bsm, sources) == (False, True)
  assert rear_distance_blocks({}, 'RR', {1})


def test_unobserved_optional_sources_do_not_claim_support():
  assert rear_lane_change_blocks(None, None, ()) == (False, False)
  camera, bsm, _ = inputs()
  assert rear_lane_change_blocks(camera, bsm, ('CCNC_0x162',)) == (False, False)


def test_schema_roundtrip_and_legacy_defaults():
  from opendbc.car import structs
  msg = structs.CarState.new_message()
  assert not msg.tbtRearInputsAvailable and msg.tbtRightRearBlocked
  msg.tbtRearInputsAvailable = True
  msg.tbtRightRearBlocked = False
  with structs.CarState.from_bytes(msg.to_bytes()) as decoded:
    assert decoded.tbtRearInputsAvailable and not decoded.tbtRightRearBlocked



def test_tbt_bsd_only_does_not_need_any_corner_message_or_gap():
  from opendbc.car.hyundai.rear_gap import tbt_bsd_inputs
  camera, bsm, _ = inputs(25.0)
  assert tbt_bsd_inputs(bsm, camera._last_update_nanos) == (True, False)
  camera.vl.clear()
  camera.dat.clear()
  assert tbt_bsd_inputs(bsm, camera._last_update_nanos) == (True, False)


@pytest.mark.parametrize('field', ['FR_INDICATOR', 'INDICATOR_RIGHT_TWO', 'INDICATOR_RIGHT_FOUR'])
def test_each_oem_right_warning_blocks_tbt(field):
  from opendbc.car.hyundai.rear_gap import tbt_bsd_inputs
  camera, bsm, _ = inputs()
  bsm.vl['BLINDSPOTS_REAR_CORNERS'][field] = 1
  assert tbt_bsd_inputs(bsm, camera._last_update_nanos) == (True, True)


@pytest.mark.parametrize('fault', ['old', 'future', 'missing_stamp', 'short_frame', 'bus_timeout', 'missing_field', 'absent'])
def test_tbt_bsd_missing_is_not_clear(fault):
  from opendbc.car.hyundai.rear_gap import tbt_bsd_inputs
  camera, bsm, _ = inputs()
  name = 'BLINDSPOTS_REAR_CORNERS'
  now = camera._last_update_nanos
  if fault == 'old':
    now += REAR_INPUT_MAX_AGE_NS + 1
  elif fault == 'future':
    bsm.ts_nanos[name]['COUNTER'] += 1
  elif fault == 'missing_stamp':
    bsm.ts_nanos[name] = {}
  elif fault == 'short_frame':
    bsm.dat[0x1BA] = bytes(8)
  elif fault == 'bus_timeout':
    bsm.bus_timeout = True
  elif fault == 'missing_field':
    del bsm.vl[name]['FR_INDICATOR']
  else:
    bsm = None
  assert tbt_bsd_inputs(bsm, now) == (False, True)
