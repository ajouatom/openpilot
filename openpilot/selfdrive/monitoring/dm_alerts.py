"""Presentation of DM health and warnings; does not change monitoring policy."""

def driver_monitoring_hud_alert(sm):
  """Only export a current, enabled DM warning (including AlwaysOnDM)."""
  dm = sm['driverMonitoringState']
  if not sm.all_checks(['driverMonitoringState']) or dm.dm2Disabled:
    return 0
  return {'one': 1, 'two': 2, 'three': 3}.get(str(dm.alertLevel), 0)
