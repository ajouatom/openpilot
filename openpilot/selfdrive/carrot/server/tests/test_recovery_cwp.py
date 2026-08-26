from openpilot.selfdrive.carrot.recovery import server as recovery


def test_cwp_status_disables_unregistered_device(monkeypatch):
  writes = []
  monkeypatch.setattr(recovery, "_read_param", lambda key, default="": "1" if key == recovery.CWP_RECOVERY_BOOT_PARAM else default)
  monkeypatch.setattr(recovery, "_write_param", lambda key, value: writes.append((key, value)))
  monkeypatch.setattr(recovery, "_cwp_device_id", lambda: "device-id")
  monkeypatch.setattr(recovery, "_cwp_request", lambda path, payload: {
    "ok": True,
    "body": {"ok": True, "registered": False},
  })

  assert recovery._cwp_status() == {
    "ok": True,
    "enabled": False,
    "registered": False,
    "state": "unregistered",
  }
  assert writes == [(recovery.CWP_RECOVERY_BOOT_PARAM, "0")]


def test_cwp_enable_requires_registration(monkeypatch):
  writes = []
  monkeypatch.setattr(recovery, "_cwp_status", lambda: {
    "ok": True, "enabled": False, "registered": False, "state": "unregistered",
  })
  monkeypatch.setattr(recovery, "_write_param", lambda key, value: writes.append((key, value)))

  result = recovery._cwp_set_enabled(True)

  assert result["ok"] is False
  assert result["error"] == "Not registered"
  assert writes == []


def test_cwp_enable_writes_persistent_param(monkeypatch):
  writes = []
  monkeypatch.setattr(recovery, "_cwp_status", lambda: {
    "ok": True, "enabled": False, "registered": True, "state": "ready",
  })
  monkeypatch.setattr(recovery, "_write_param", lambda key, value: writes.append((key, value)))

  result = recovery._cwp_set_enabled(True)

  assert result["ok"] is True
  assert result["enabled"] is True
  assert writes == [(recovery.CWP_RECOVERY_BOOT_PARAM, "1")]


def test_cwp_boot_sends_recovery_port(monkeypatch):
  requests = []
  monkeypatch.setattr(recovery, "_read_param", lambda key, default="": "1" if key == recovery.CWP_RECOVERY_BOOT_PARAM else default)
  monkeypatch.setattr(recovery, "_local_ip", lambda: "192.168.0.5")
  monkeypatch.setattr(recovery, "_cwp_device_id", lambda: "device-id")
  monkeypatch.setattr(recovery.time, "sleep", lambda _seconds: None)
  monkeypatch.setattr(recovery, "_cwp_request", lambda path, payload: requests.append((path, payload)) or {
    "ok": True,
    "body": {"ok": True, "registered": True, "pushed": 1},
  })

  recovery._cwp_boot_worker(6999)

  assert requests == [("/recovery/boot", {"deviceId": "device-id", "ip": "192.168.0.5", "port": 6999})]


def test_recovery_page_has_short_cwp_toggle_states():
  assert '>CWP Push</span>' in recovery.HTML_PAGE
  assert 'id="rcCwpPush" type="checkbox" disabled' in recovery.HTML_PAGE
  assert 'cwpState.hidden = !!(data && data.registered)' in recovery.RECOVERY_JS
  assert '"Not registered"' in recovery.RECOVERY_JS
  assert '"Unavailable"' in recovery.RECOVERY_JS
