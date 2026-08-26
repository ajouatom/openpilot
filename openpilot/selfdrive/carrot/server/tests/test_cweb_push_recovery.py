import sys
import types


params_module = types.ModuleType("openpilot.common.params")
params_module.Params = object
sys.modules.setdefault("openpilot.common.params", params_module)

from openpilot.selfdrive.carrot import cweb_push


class FakeParams:
  def get(self, _key):
    return b"device-id"


def test_recovery_push_hands_first_7000_update_to_heartbeat(monkeypatch):
  calls = []
  monkeypatch.setattr(cweb_push, "_recovery_boot_push_sent", lambda: True)
  monkeypatch.setattr(cweb_push, "get_local_ip", lambda _iface: "192.168.0.5")
  monkeypatch.setattr(cweb_push, "post_json", lambda url, payload, timeout: calls.append((url, payload, timeout)) or (True, 200, "{}"))

  reporter = cweb_push.CwebPushReporter(
    params=FakeParams(),
    report_url="https://cwp.example/report",
    heartbeat_url="https://cwp.example/heartbeat",
    iface="wlan0",
    port=7000,
    timeout_s=4,
    heartbeat_interval_s=10,
    debounce_s=0,
  )

  assert reporter.poll_once() is False
  assert reporter.poll_once() is True
  assert calls == [("https://cwp.example/heartbeat", {"deviceId": "device-id", "ip": "192.168.0.5", "port": 7000}, 4)]
  assert reporter.first_report is False
  assert reporter.last_success_ip == "192.168.0.5"
