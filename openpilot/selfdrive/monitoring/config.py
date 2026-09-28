"""DM configuration applied together before manager starts its child processes."""
import os


def disabled_mode(params):
  # The external web server also reads this snapshot. Reading the saved setting
  # here would switch manager before selfdrived/controlsd restart.
  mode = params.get_int("DisableDMActive")
  return mode if mode in (1, 2) else 0


def experimental_mode(params):
  return os.environ.get("CARROT_DM_MODE", str(params.get_int("DriverMonitoringMode"))) == "1"


def configure_monitoring(params, environ=None):
  env = os.environ if environ is None else environ
  # Preserve the old road-streaming preference on the first DM2 migration.
  if params.get("DriverMonitoringMode") is None:
    if params.get("CarrotVisionEnabled") is None:
      params.put_bool("CarrotVisionEnabled", params.get_int("DisableDM") == 2)
    params.put_int("DriverMonitoringMode", 0)
  env["CARROT_DM_MODE"] = "1" if params.get_int("DriverMonitoringMode") == 1 else "0"
  mode = params.get_int("DisableDM")
  params.put_int("DisableDMActive", mode if mode in (1, 2) else 0)
