"""Exercise production cloud boundaries without native device modules or live uploads."""
import ast
from enum import IntEnum
import os
from pathlib import Path
import threading
from types import SimpleNamespace

import pytest
import requests


ROOT = Path(__file__).resolve().parents[3]


def load_definitions(path, names, **namespace):
  tree = ast.parse((ROOT / path).read_text())
  nodes = [node for node in tree.body if isinstance(node, (ast.FunctionDef, ast.ClassDef)) and node.name in names]
  assert {node.name for node in nodes} == set(names)
  module = ast.Module(body=[ast.ImportFrom(module="__future__", names=[ast.alias(name="annotations")], level=0), *nodes], type_ignores=[])
  exec(compile(ast.fix_missing_locations(module), str(ROOT / path), "exec"), namespace)
  return namespace


@pytest.fixture
def api(monkeypatch):
  monkeypatch.delenv("COMMA_CONNECT_ENABLED", raising=False)
  return load_definitions("openpilot/common/api.py", ["comma_connect_enabled", "api_get"],
                          os=os, requests=requests, API_HOST="https://api.example.invalid", get_version=lambda: "test")


@pytest.mark.parametrize("value", [None, "", "0", "false", "true", "yes"])
def test_cloud_requires_explicit_opt_in(api, monkeypatch, value):
  if value is not None:
    monkeypatch.setenv("COMMA_CONNECT_ENABLED", value)
  assert not api["comma_connect_enabled"]()


@pytest.mark.parametrize("method", ["GET", "POST"])
@pytest.mark.parametrize("use_session", [False, True])
def test_disabled_api_never_reaches_transport(api, monkeypatch, mocker, method, use_session):
  transport = mocker.Mock(side_effect=AssertionError("unexpected network request"))
  monkeypatch.setattr(requests.sessions.Session, "request", transport)
  session = requests.Session() if use_session else None
  with pytest.raises(requests.exceptions.ConnectionError, match="comma Connect is disabled"):
    api["api_get"]("v2/pilotauth/", method=method, access_token="test-token", session=session, serial="test-serial")
  transport.assert_not_called()


def test_opt_in_preserves_api_contract(api, monkeypatch, mocker):
  monkeypatch.setenv("COMMA_CONNECT_ENABLED", "1")
  session = mocker.Mock()
  assert api["api_get"]("device", method="POST", timeout=3, access_token="test-token", session=session, value=2) == session.request.return_value
  session.request.assert_called_once_with("POST", "https://api.example.invalid/device", timeout=3,
                                          headers={"Authorization": "JWT test-token", "User-Agent": "openpilot-test"}, params={"value": 2})


@pytest.mark.parametrize(("cached", "persisted", "expected"), [
  ("cached-id", "persisted-id", "cached-id"),
  (None, "persisted-id", "persisted-id"),
  (None, None, "UnregisteredDevice"),
  ("UnregisteredDevice", "persisted-id", "UnregisteredDevice"),
])
def test_offline_registration_keeps_identity_without_keys_or_modem(api, tmp_path, mocker, cached, persisted, expected):
  params = {"DongleId": cached} if cached else {}
  writes = []

  def put(key, value):
    writes.append((key, value))
    params[key] = value

  if persisted:
    (tmp_path / "comma").mkdir()
    (tmp_path / "comma/dongle_id").write_text(persisted)
  forbidden = mocker.Mock(side_effect=AssertionError("offline registration touched cloud or hardware"))
  module = load_definitions("openpilot/system/athena/registration.py", ["register"],
                            Params=lambda: SimpleNamespace(get=params.get, put=put), Path=Path,
                            Paths=SimpleNamespace(persist_root=lambda: str(tmp_path)),
                            comma_connect_enabled=api["comma_connect_enabled"], get_key_pair=forbidden,
                            api_get=forbidden, HARDWARE=forbidden, Spinner=forbidden,
                            UNREGISTERED_DONGLE_ID="UnregisteredDevice")
  assert module["register"](show_spinner=True) == expected
  assert writes == ([] if cached is None and persisted is None else [("DongleId", expected)])
  forbidden.assert_not_called()


@pytest.mark.parametrize("path", [
  "openpilot/system/athena/manage_athenad.py",
  "openpilot/system/athena/athenad.py",
  "openpilot/system/loggerd/uploader.py",
])
def test_direct_daemon_start_exits_before_loading_queues_or_connecting(api, path):
  module = load_definitions(path, ["main"], comma_connect_enabled=api["comma_connect_enabled"])
  assert module["main"]() is None


def test_manager_does_not_start_athena(api):
  path = ROOT / "openpilot/system/manager/process_config.py"
  tree = ast.parse(path.read_text())
  daemon = next(node for node in ast.walk(tree) if isinstance(node, ast.Call) and isinstance(node.func, ast.Name)
                and node.func.id == "DaemonProcess" and node.args[0].value == "manage_athenad")
  process = eval(compile(ast.Expression(daemon), str(path), "eval"),
                 {"DaemonProcess": lambda *args, **kwargs: kwargs, "comma_connect_enabled": api["comma_connect_enabled"]})
  assert process["enabled"] is False


def test_error_reporting_does_not_initialize_sentry(api):
  module = load_definitions("openpilot/system/sentry.py", ["init"], comma_connect_enabled=api["comma_connect_enabled"])
  assert module["init"](object()) is False


def test_comma_map_tiles_fail_before_identity_or_cache_access(api):
  module = load_definitions("openpilot/selfdrive/navigation/tiles.py", ["RemoteTiles"],
                            comma_connect_enabled=api["comma_connect_enabled"])
  with pytest.raises(OSError, match="comma Connect map tiles are disabled"):
    module["RemoteTiles"]()


def test_prime_cache_is_retained_without_polling_or_threads(api):
  session = requests.Session()
  module = load_definitions("openpilot/selfdrive/ui/lib/prime_state.py", ["PrimeType", "PrimeState"],
                            os=os, IntEnum=IntEnum, threading=threading, requests=SimpleNamespace(Session=lambda: session),
                            Params=lambda: SimpleNamespace(get=lambda key: "2" if key == "PrimeType" else "cached-id"),
                            comma_connect_enabled=api["comma_connect_enabled"])
  prime = module["PrimeState"]()
  prime.start()
  prime._fetch_prime_status()
  assert prime.get_type() == module["PrimeType"].LITE
  assert prime._thread is None and not prime._running
  session.close()


@pytest.mark.parametrize(("path", "class_name", "method", "expected"), [
  ("openpilot/selfdrive/ui/widgets/pairing_dialog.py", "PairingDialog", "_get_pairing_url", ""),
  ("openpilot/selfdrive/ui/widgets/pairing_dialog.py", "PairingDialog", "_generate_qr_code", None),
  ("openpilot/selfdrive/ui/mici/widgets/pairing_dialog.py", "PairingDialog", "_get_pairing_url", ""),
  ("openpilot/selfdrive/ui/mici/widgets/pairing_dialog.py", "PairingDialog", "_generate_qr_code", None),
  ("openpilot/selfdrive/ui/mici/layouts/settings/firehose.py", "FirehoseLayoutBase", "_fetch_firehose_stats", None),
])
def test_cloud_ui_does_not_sign_tokens_or_fetch_data(api, path, class_name, method, expected):
  tree = ast.parse((ROOT / path).read_text())
  cls = next(node for node in tree.body if isinstance(node, ast.ClassDef) and node.name == class_name)
  fn = next(node for node in cls.body if isinstance(node, ast.FunctionDef) and node.name == method)
  namespace = {"comma_connect_enabled": api["comma_connect_enabled"]}
  exec(compile(ast.Module(body=[fn], type_ignores=[]), str(ROOT / path), "exec"), namespace)
  assert namespace[method](object()) == expected
