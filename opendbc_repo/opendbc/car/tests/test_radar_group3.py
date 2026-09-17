from dataclasses import replace
from types import SimpleNamespace

import pytest

from opendbc.car.hyundai.radar_group3 import Group3Object, Group3TrackIds, decode_group3
from openpilot.selfdrive.carrot.radar.tools.radar_group3_replay import Group3Replay


ACTIVE = bytes.fromhex("e1043b0f02590e692a227e16f80fe00f28fcc753a20a0000")
EMPTY = bytes.fromhex("c03d3b0000000000ff0700000000000000d0020000000000")


def test_native_id_excludes_high_flag_bit():
  obj = decode_group3(ACTIVE)
  assert obj.object_id == 15
  assert (obj.x, obj.y, obj.v, obj.length) == pytest.approx((55.4, -3.0, 4.4, 4.4))
  flagged = bytearray(ACTIVE)
  flagged[3] |= 128
  assert decode_group3(flagged) == obj
  assert decode_group3(EMPTY).object_id == 0


def test_slot_swap_preserves_both_objects():
  manager = Group3TrackIds()
  a = Group3Object(45, 20, 0, -2, 4)
  b = Group3Object(12, 50, 3, 1, 4)
  first = manager.update({0x400: a, 0x401: b})
  second = manager.update({0x400: b, 0x401: a})
  assert second[0x401] == first[0x400]
  assert second[0x400] == first[0x401]


@pytest.mark.parametrize("change", [dict(object_id=46), dict(x=0.2), dict(y=6), dict(v=8)])
def test_replacement_and_discontinuity_start_new_history(change):
  manager = Group3TrackIds()
  obj = Group3Object(45, 204.6, 0, -2, 4)
  first = manager.update({0x400: obj})[0x400]
  assert manager.update({0x400: replace(obj, **change)})[0x400] != first


@pytest.mark.parametrize("interruption", ["missing", "empty", "duplicate"])
def test_interrupted_identity_is_not_reused(interruption):
  manager = Group3TrackIds()
  obj = Group3Object(45, 20, 0, -2, 4)
  first = manager.update({0x400: obj})[0x400]
  middle = {} if interruption == "missing" else ({0x400: decode_group3(EMPTY)} if interruption == "empty" else {0x400: obj, 0x401: obj})
  assert not manager.update(middle)
  assert manager.update({0x410: obj})[0x410] != first


def test_replay_preserves_braking_history_across_slot_moves():
  replay = Group3Replay(1)
  ids, accelerations = [], []
  for frame in range(80):
    t = frame * 0.05
    slot = 0x400 + frame % 30
    for address in range(0x400, 0x41e):
      replay.consume(t, address, ACTIVE if address == slot else EMPTY, 1)
    recorded = SimpleNamespace(trackId=32 + slot - 0x400, radarSource="frontRadar",
                               dRel=53.1, yRel=-3.0, vRel=4.4, vLead=20.0-t)
    point, = replay.correct(t + 0.005, (recorded,))
    ids.append(point.trackId)
    accelerations.append(point.aLead)
    # Another consumer of the same cycle must not advance the observer.
    assert replay.correct(t + 0.006, (recorded,))[0].aLead == point.aLead
  assert len(set(ids)) == 1
  assert accelerations[-1] == pytest.approx(-1.0, abs=0.1)
  assert replay.correct(5.0, (recorded,)) == ()


def test_replay_ignores_other_bus_and_keeps_scc():
  replay = Group3Replay(1)
  replay.consume(0, 0x400, ACTIVE, 0)
  replay.consume(0, 0x41d, EMPTY, 0)
  scc = SimpleNamespace(trackId=0, radarSource="scc")
  assert replay.correct(0, (scc,)) == (scc,)
