from opendbc.safety import ALTERNATIVE_EXPERIENCE


def get_alternative_experience(disengage_on_accelerator: bool) -> int:
  """Keep Panda and selfdrived accelerator-disengage policy in sync."""
  alternative_experience = ALTERNATIVE_EXPERIENCE.DEFAULT
  if not disengage_on_accelerator:
    alternative_experience |= ALTERNATIVE_EXPERIENCE.DISABLE_DISENGAGE_ON_GAS
  return alternative_experience
