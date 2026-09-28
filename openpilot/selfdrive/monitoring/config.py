"""Live DM mode and boot migration; the retired DisableDM never disables monitoring."""
import os


def experimental_mode(params):
  return params.get_int("DriverMonitoringMode") == 1


def configure_monitoring(params, environ=None):
  env = os.environ if environ is None else environ
  # Old mode 2 also enabled road streaming. Preserve only that independent feature.
  if params.get("DriverMonitoringMode") is None:
    if params.get("CarrotVisionEnabled") is None:
      params.put_bool("CarrotVisionEnabled", params.get_int("DisableDM") == 2)
    params.put_int("DriverMonitoringMode", 0)
  env.pop("CARROT_DM_MODE", None)  # Retired startup latch must not override live Params.
