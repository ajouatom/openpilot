"""Exercise boot migration with the native typed Params implementation."""
import pytest

from openpilot.selfdrive.monitoring.config import configure_monitoring


@pytest.mark.parametrize('legacy', [None, 0, 1, 2])
def test_boot_configuration_with_native_params(tmp_path, legacy):
  native = pytest.importorskip('openpilot.common.params_pyx')
  params = native.Params(str(tmp_path / 'params'))
  if legacy is not None:
    params.put_int('DisableDM', legacy)
  env = {}
  configure_monitoring(params, env)
  assert type(params.get('DriverMonitoringMode')) is int
  assert params.get('DriverMonitoringMode') == 0
  assert type(params.get('DisableDMActive')) is int
  assert params.get('DisableDMActive') == (legacy or 0)
  assert params.get('CarrotVisionEnabled') is (legacy == 2)
  assert env['CARROT_DM_MODE'] == '0'
  params.put_int('DriverMonitoringMode', 1)
  params.put_bool('CarrotVisionEnabled', False)
  configure_monitoring(params, env)
  assert env['CARROT_DM_MODE'] == '1'
  assert params.get('CarrotVisionEnabled') is False
  params.put_int('DisableDM', 2 if legacy != 2 else 0)
  assert params.get('DisableDMActive') == (legacy or 0)
  configure_monitoring(params, env)
  assert params.get('DisableDMActive') == (2 if legacy != 2 else 0)
