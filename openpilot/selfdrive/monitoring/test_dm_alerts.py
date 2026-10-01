from types import SimpleNamespace

import pytest

from openpilot.cereal import log
from openpilot.selfdrive.monitoring.dm_alerts import CameraFallbackNotice, driver_monitoring_hud_alert


def test_startup_notice_is_bounded_without_requiring_camera_recovery():
  notice = CameraFallbackNotice()
  for t in (0, 3, 10, 17, 29.99):
    assert not notice.update(t, True, True, False)
  assert notice.update(30, True, True, False)
  assert notice.update(300, True, True, False)
  assert not notice.update(301, True, False, False)


def test_recovered_camera_lost_during_startup_gets_normal_notice():
  notice = CameraFallbackNotice()
  assert not notice.update(4, True, False, False)
  assert notice.update(5, True, True, False)
  assert not notice.update(6, False, True, False)
  assert not notice.update(7, True, True, True)
  assert notice.update(8, True, True, False)


@pytest.mark.parametrize('invalid,disabled', [(False, False), (True, False), (False, True), (True, True)])
@pytest.mark.parametrize('level,expected', [('none', 0), ('one', 1), ('two', 2), ('three', 3)])
def test_hud_level_requires_fresh_enabled_monitoring(level, expected, invalid, disabled):
  class Messages(dict):
    def all_checks(self, services):
      assert services == ['driverMonitoringState']
      return not invalid

  state = log.DriverMonitoringState.new_message(alertLevel=level, dm2Disabled=disabled)
  assert driver_monitoring_hud_alert(Messages(driverMonitoringState=state)) == (0 if invalid or disabled else expected)


def test_unknown_dm_alert_is_not_exported():
  sm = {'driverMonitoringState': SimpleNamespace(alertLevel='unknown', dm2Disabled=False)}
  class Messages(dict):
    def all_checks(self, services):
      return True
  assert driver_monitoring_hud_alert(Messages(sm)) == 0
