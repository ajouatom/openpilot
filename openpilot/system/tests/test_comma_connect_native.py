"""Actual module imports and native Params, using isolated storage and blocked HTTP."""
import importlib

import pytest
import requests

from openpilot.common.api import api_get
from openpilot.common.params import Params
from openpilot.system.athena import registration
from openpilot.system.hardware.hw import Paths


@pytest.fixture(autouse=True)
def offline(monkeypatch, mocker):
  monkeypatch.delenv("COMMA_CONNECT_ENABLED", raising=False)
  transport = mocker.patch.object(requests.sessions.Session, "request", side_effect=AssertionError("unexpected network request"))
  yield
  transport.assert_not_called()


@pytest.mark.parametrize(("cached", "persisted", "expected"), [
  ("cached-id", "persisted-id", "cached-id"), (None, "persisted-id", "persisted-id"),
  (None, None, registration.UNREGISTERED_DONGLE_ID),
  (registration.UNREGISTERED_DONGLE_ID, "persisted-id", registration.UNREGISTERED_DONGLE_ID),
])
def test_offline_registration_native_params(monkeypatch, mocker, tmp_path, cached, persisted, expected):
  monkeypatch.setattr(Paths, "persist_root", lambda: str(tmp_path))
  params = Params()
  if cached:
    params.put("DongleId", cached)
  if persisted:
    (tmp_path / "comma").mkdir()
    (tmp_path / "comma/dongle_id").write_text(persisted, encoding="utf-8")
  key_read = mocker.patch.object(registration, "get_key_pair", side_effect=AssertionError("key read"))
  modem = mocker.patch.object(registration.HARDWARE, "get_imei", side_effect=AssertionError("modem wait"))
  assert registration.register(show_spinner=True) == expected
  assert params.get("DongleId") == (expected if cached or persisted else None)
  key_read.assert_not_called()
  modem.assert_not_called()


@pytest.mark.parametrize("session", [None, requests.Session()])
def test_real_api_transport_boundary(session):
  with pytest.raises(requests.exceptions.ConnectionError, match="comma Connect is disabled"):
    api_get("v2/pilotauth/", method="POST", access_token="test-token", session=session)


@pytest.mark.parametrize("name", ["openpilot.system.athena.athenad", "openpilot.system.athena.manage_athenad",
                                  "openpilot.system.loggerd.uploader"])
def test_daemon_entry_points(name):
  assert importlib.import_module(name).main() is None


def test_real_process_configuration():
  from openpilot.system.manager.process_config import managed_processes
  assert not managed_processes["manage_athenad"].enabled


def test_real_sentry_init(mocker):
  from openpilot.system import sentry
  init = mocker.patch.object(sentry.sentry_sdk, "init")
  assert sentry.init(sentry.SentryProject.SELFDRIVE) is False
  init.assert_not_called()


def test_real_prime_and_map_boundaries():
  from openpilot.selfdrive.ui.lib.prime_state import PrimeState, PrimeType
  from openpilot.selfdrive.navigation.tiles import RemoteTiles
  Params().put("PrimeType", str(int(PrimeType.LITE)))
  prime = PrimeState()
  prime.start()
  prime._fetch_prime_status()
  assert prime.get_type() == PrimeType.LITE
  assert prime._thread is None
  prime._session.close()
  with pytest.raises(OSError, match="comma Connect map tiles are disabled"):
    RemoteTiles()
