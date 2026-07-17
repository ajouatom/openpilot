from pytest_mock import MockerFixture

from openpilot.system.ui.lib.wifi_manager import ConnectStatus, WifiManager, WifiState


def _run_scanner_once(mocker: MockerFixture, *, active: bool, status: ConnectStatus, elapsed: float):
  wm = WifiManager.__new__(WifiManager)
  wm._exit = False
  wm._active = active
  wm._wifi_state = WifiState(status=status)
  wm._last_network_scan = 100.0
  wm._request_scan = mocker.MagicMock()

  mocker.patch("openpilot.system.ui.lib.wifi_manager.time.monotonic", side_effect=[100.0 + elapsed, 100.0 + elapsed])

  def stop_after_iteration(_):
    wm._exit = True

  mocker.patch("openpilot.system.ui.lib.wifi_manager.time.sleep", side_effect=stop_after_iteration)
  wm._network_scanner()
  return wm


def test_background_scans_after_five_seconds_when_disconnected(mocker):
  wm = _run_scanner_once(mocker, active=False, status=ConnectStatus.DISCONNECTED, elapsed=5.1)

  wm._request_scan.assert_called_once_with()


def test_background_does_not_scan_while_connected(mocker):
  wm = _run_scanner_once(mocker, active=False, status=ConnectStatus.CONNECTED, elapsed=60.0)

  wm._request_scan.assert_not_called()
