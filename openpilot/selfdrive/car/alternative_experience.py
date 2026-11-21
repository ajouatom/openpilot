from opendbc.safety import ALTERNATIVE_EXPERIENCE


def get_alternative_experience(disengage_on_accelerator: bool, *, always_lateral: bool = False, hyundai_canfd: bool = False) -> int:
  """Carry accelerator policy and explicit strict-profile lateral permission."""
  alternative_experience = ALTERNATIVE_EXPERIENCE.DEFAULT
  if not disengage_on_accelerator:
    alternative_experience |= ALTERNATIVE_EXPERIENCE.DISABLE_DISENGAGE_ON_GAS
  if hyundai_canfd and always_lateral:
    alternative_experience |= ALTERNATIVE_EXPERIENCE.HYUNDAI_CANFD_ALWAYS_LATERAL
  return alternative_experience
