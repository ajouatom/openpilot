"""Presentation of DM health and warnings; does not change monitoring policy."""

DM_STARTUP_NOTICE_DELAY = 30.0


class CameraFallbackNotice:
  def __init__(self):
    self.camera_ready_once = False

  def update(self, elapsed, valid, unavailable, disabled):
    if not valid or disabled:
      return False
    if not unavailable:
      self.camera_ready_once = True
      return False
    # Initial model/calibration loading is not evidence of a camera fault.
    # Bound the quiet startup period; a later loss gets the normal notice delay.
    return self.camera_ready_once or elapsed >= DM_STARTUP_NOTICE_DELAY


def driver_monitoring_hud_alert(sm):
  """Only export a current, enabled DM warning (including AlwaysOnDM)."""
  dm = sm['driverMonitoringState']
  if not sm.all_checks(['driverMonitoringState']) or dm.dm2Disabled:
    return 0
  return {'one': 1, 'two': 2, 'three': 3}.get(str(dm.alertLevel), 0)
