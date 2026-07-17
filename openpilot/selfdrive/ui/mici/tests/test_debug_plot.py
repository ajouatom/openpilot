from types import SimpleNamespace

import pytest

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
