from __future__ import annotations

from . import params as params_service

PARAM_NAME = "CarrotQuiet"


def read_enabled() -> bool:
  return params_service._coerce_bool(params_service.get_param_value(PARAM_NAME, False))


def write_enabled(enabled: bool) -> None:
  params_service.set_param_value(PARAM_NAME, bool(enabled))


def toggle() -> bool:
  enabled = not read_enabled()
  write_enabled(enabled)
  return enabled
