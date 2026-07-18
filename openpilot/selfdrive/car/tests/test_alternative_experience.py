from opendbc.safety import ALTERNATIVE_EXPERIENCE
from openpilot.selfdrive.car.alternative_experience import get_alternative_experience


def test_disengage_on_accelerator_enabled_uses_default_safety_behavior():
  assert get_alternative_experience(True) == ALTERNATIVE_EXPERIENCE.DEFAULT


def test_disengage_on_accelerator_disabled_keeps_lateral_controls_allowed():
  assert get_alternative_experience(False) == ALTERNATIVE_EXPERIENCE.DISABLE_DISENGAGE_ON_GAS
