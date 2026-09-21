from copy import deepcopy
from types import SimpleNamespace

import pytest

from opendbc.car.hyundai.hyundaicanfd import DisplayLeadLateralFilter
from opendbc.car.hyundai.tests.test_ccnc_lead import send_ccnc
from opendbc.car.hyundai.tests.test_scc_lead import make_cs, make_radar, send


def radar(**kwargs):
  return make_radar(radar=True, radarTrackId=32, **kwargs)


def test_filter_reduces_jitter_and_retains_sustained_lateral_motion():
  smooth = DisplayLeadLateralFilter()
  assert smooth.update(radar(yRel=0)) == 0
  outputs = [smooth.update(radar(yRel=0.8 if tick % 2 else -0.8)) for tick in range(200)]
  assert max(abs(value) for value in outputs) < 0.03
  outputs = [smooth.update(radar(yRel=-1)) for _ in range(40)]
  assert 0.60 < outputs[-1] < 0.65  # About 63% response after the 0.4 s time constant.
  assert all(a < b for a, b in zip(outputs, outputs[1:], strict=False))


def test_role_change_keeps_filter_but_new_target_resets_it():
  smooth = DisplayLeadLateralFilter()
  state = radar(yRel=0, dRel=20)
  smooth.update(state)
  state.leadTwo = radar(yRel=-1, dRel=20).leadOne
  state.leadOne.radarTrackId = 52
  state.leadOne.dRel = 40
  # Same track moved to leadTwo without a physical range jump.
  assert 0 < smooth.update(state) < 0.03
  state.leadTwo.radarTrackId = 60
  assert smooth.update(state) == 1
  state.leadTwo.dRel = 10
  state.leadTwo.yRel = 2
  assert smooth.update(state) == -2  # Reused ID with discontinuous range.


@pytest.mark.parametrize("missing", [None, radar(status=False), radar(yRel=float("nan"))])
def test_missing_target_clears_both_messages_and_reacquires_without_old_history(monkeypatch, missing):
  smooth = DisplayLeadLateralFilter()
  smooth.update(radar(yRel=-2))
  lateral = smooth.update(missing)
  assert lateral == 0
  assert send(make_cs(missing), hud_lateral=lateral)["HUD_LEAD_INFO"] == 0
  cc = send_ccnc(monkeypatch, missing, hud_lateral=lateral)
  assert cc["FF_DETECT"] == cc["FF_LATERAL"] == 0
  assert smooth.update(radar(yRel=1)) == -1


def test_shared_filter_can_packing_preserves_control_fields_and_oem_class(monkeypatch):
  smooth = DisplayLeadLateralFilter()
  model = SimpleNamespace(position=SimpleNamespace(x=[0, 50], y=[0, 1]))
  state = radar(dRel=25, yRel=-0.5)
  assert smooth.update(state, model) == 0
  state.leadOne.yRel = -2.5
  original = deepcopy(state)
  lateral = 0
  for _ in range(40):
    lateral = smooth.update(state, model)
  assert 1.2 < lateral < 1.3
  assert state == original
  cs = make_cs(state)
  cs.modelV2 = model
  raw = send(cs)
  filtered = send(cs, hud_lateral=lateral)
  assert {k: v for k, v in filtered.items() if k not in ("CHECKSUM", "ACC_ObjLatPos")} == {
    k: v for k, v in raw.items() if k not in ("CHECKSUM", "ACC_ObjLatPos")
  }
  cc = send_ccnc(monkeypatch, state, model=model, hud_lateral=lateral,
                 stock=dict(FF_DETECT=12, FF_DISTANCE=25, FF_LATERAL=2))
  assert cc["FF_LATERAL"] == pytest.approx(filtered["ACC_ObjLatPos"])
  assert cc["FF_DETECT"] == 12
  assert cc["FF_DISTANCE"] == filtered["ACC_ObjDist"] == 25


def test_filter_state_is_per_vehicle():
  first, second = DisplayLeadLateralFilter(), DisplayLeadLateralFilter()
  first.update(radar(yRel=-2))
  assert second.update(radar(yRel=1)) == -1
