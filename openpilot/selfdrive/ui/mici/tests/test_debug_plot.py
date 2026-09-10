from types import SimpleNamespace

import pytest

from openpilot.cereal import log
from openpilot.selfdrive.ui.mici.onroad.debug_plot import DebugPlot


class FakeSubMaster(dict):
  def __init__(self, longitudinal_plan):
    super().__init__(
      carState=SimpleNamespace(aEgo=1.0, vEgo=2.0),
      longitudinalPlan=longitudinal_plan,
      carControl=SimpleNamespace(actuators=SimpleNamespace(accel=3.0)),
      controlsState=SimpleNamespace(),
      modelV2=SimpleNamespace(),
      radarState=SimpleNamespace(),
      liveParameters=SimpleNamespace(),
    )
    self.alive = dict.fromkeys(self, True)


@pytest.mark.parametrize(
  "accels,speeds,mode,expected",
  [
    ([], [], 1, [1.0, 0.0, 3.0]),
    ([], [], 2, [0.0, 2.0, 1.0]),
    ([4.0], [5.0], 1, [1.0, 4.0, 3.0]),
    ([4.0], [5.0], 2, [5.0, 2.0, 1.0]),
  ],
)
def test_make_plot_data_handles_empty_longitudinal_plan(accels, speeds, mode, expected):
  sm = FakeSubMaster(SimpleNamespace(accels=accels, speeds=speeds))

  data, _ = DebugPlot._make_plot_data(None, sm, mode)

  assert data == expected


@pytest.mark.parametrize(
  "mode,expected_samples",
  [
    (4, [[4.0, -1.5, -2.0], [4.0, 0.75, 3.0], [4.0, 0.0, 0.0]]),
    (5, [[1.0, -2.5, -0.5], [1.0, 1.25, 0.25], [1.0, 0.0, 0.0]]),
  ],
)
def test_lead_plots_follow_primary_lead_updates(mode, expected_samples):
  sm = FakeSubMaster(SimpleNamespace(accels=[4.0], speeds=[5.0]))
  lead_samples = [
    {"status": True, "aLeadK": -1.5, "vRel": -2.0, "aLead": -2.5, "jLead": -0.5},
    {"status": True, "aLeadK": 0.75, "vRel": 3.0, "aLead": 1.25, "jLead": 0.25},
    {"status": False},
  ]

  for lead, expected in zip(lead_samples, expected_samples, strict=True):
    radar = log.RadarState.new_message()
    radar.leadOne = lead
    radar.leadTwo = {"status": True, "aLeadK": 9.0, "vRel": 8.0, "aLead": 7.0, "jLead": 6.0}
    sm["radarState"] = radar.as_reader()

    data, _ = DebugPlot._make_plot_data(None, sm, mode)

    assert data == pytest.approx(expected)
