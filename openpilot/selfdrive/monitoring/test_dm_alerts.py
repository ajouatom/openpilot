from types import SimpleNamespace

import pytest

from openpilot.cereal import log
from openpilot.selfdrive.monitoring.dm_alerts import driver_monitoring_hud_alert


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
