from opendbc.safety import ALTERNATIVE_EXPERIENCE
from openpilot.selfdrive.car.alternative_experience import get_alternative_experience


def test_disengage_on_accelerator_enabled_uses_default_safety_behavior():
  assert get_alternative_experience(True) == ALTERNATIVE_EXPERIENCE.DEFAULT


def test_disengage_on_accelerator_disabled_keeps_lateral_controls_allowed():
  assert get_alternative_experience(False) == ALTERNATIVE_EXPERIENCE.DISABLE_DISENGAGE_ON_GAS


def test_hyundai_canfd_always_lateral_is_explicit_and_independent():
  for disengage in (False, True):
    for always_lateral in (False, True):
      for canfd in (False, True):
        result = get_alternative_experience(disengage, always_lateral=always_lateral, hyundai_canfd=canfd)
        assert bool(result & ALTERNATIVE_EXPERIENCE.HYUNDAI_CANFD_ALWAYS_LATERAL) == (always_lateral and canfd)
        assert bool(result & ALTERNATIVE_EXPERIENCE.DISABLE_DISENGAGE_ON_GAS) == (not disengage)
        assert result & ~(ALTERNATIVE_EXPERIENCE.HYUNDAI_CANFD_ALWAYS_LATERAL |
                          ALTERNATIVE_EXPERIENCE.DISABLE_DISENGAGE_ON_GAS) == 0
