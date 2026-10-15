from types import SimpleNamespace

import pytest

from openpilot.selfdrive.controls.lib.latcontrol_angle import LatControlAngle


@pytest.mark.parametrize("limited", [False, True])
@pytest.mark.parametrize("curvature_limited", [False, True])
def test_saturation_respects_controller_output_limit(limited, curvature_limited):
  controller = LatControlAngle(SimpleNamespace(steerLimitTimer=0.4), None)
  state = SimpleNamespace(vEgo=15.0, steeringAngleDeg=0.0, steeringPressed=False)
  model = SimpleNamespace(get_steer_from_curvature=lambda *_args: 0.2)
  params = SimpleNamespace(roll=0.0, angleOffsetDeg=0.0)
  for _ in range(50):
    _, _, status = controller.update(True, state, model, params, limited, 0.01, None, curvature_limited)
  assert status.saturated == (not limited)

  for _ in range(50):
    _, _, status = controller.update(False, state, model, params, False, 0.01, None, False)
  assert not status.saturated
  assert controller.sat_count == 0.0
