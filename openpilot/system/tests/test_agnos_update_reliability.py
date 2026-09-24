import ast
import importlib.util
import json
from pathlib import Path
import sys
import types

import pytest
import requests


def load_agnos_module():
  path = Path(__file__).resolve().parents[1] / "hardware/tici/agnos.py"
  spec = importlib.util.spec_from_file_location("agnos_update_reliability_test", path)
  assert spec is not None and spec.loader is not None
  module = importlib.util.module_from_spec(spec)
  spec.loader.exec_module(module)
  return module


agnos = load_agnos_module()


class FakeLog:
  def __init__(self):
    self.messages: list[str] = []

  def info(self, message: str) -> None:
    self.messages.append(message)

  def error(self, message: str) -> None:
    self.messages.append(message)

  def exception(self, message: str) -> None:
    self.messages.append(message)


def write_manifest(path: Path, urls: tuple[str, ...] = ()) -> None:
  path.write_text(json.dumps([
    {"name": f"partition-{i}", "url": url, "full_check": True}
    for i, url in enumerate(urls)
  ]), encoding="utf-8")


def test_confirmation_is_scoped_to_exact_manifest(monkeypatch, tmp_path: Path) -> None:
  confirmation = tmp_path / "confirmed"
  manifest = tmp_path / "agnos.json"
  write_manifest(manifest, ("https://one.example/image.xz",))
  monkeypatch.setattr(agnos, "UPDATE_CONFIRMATION_FILE", confirmation)

  assert not agnos.update_confirmed(manifest)
  agnos.mark_update_confirmed(manifest)
  assert agnos.update_confirmed(manifest)

  write_manifest(manifest, ("https://two.example/image.xz",))
  assert not agnos.update_confirmed(manifest)

  agnos.clear_update_confirmation()
  assert not confirmation.exists()


def test_manifest_probe_urls_use_one_real_asset_per_origin(tmp_path: Path) -> None:
  manifest = tmp_path / "agnos.json"
  write_manifest(manifest, (
    "https://downloads.example/a.xz",
    "https://downloads.example/b.xz",
    "https://other.example/c.xz",
  ))

  assert agnos.manifest_download_urls(manifest) == (
    "https://downloads.example/a.xz",
    "https://other.example/c.xz",
  )


def test_update_lock_blocks_a_second_flasher(monkeypatch, tmp_path: Path) -> None:
  def locked(*_args):
    raise BlockingIOError

  fake_fcntl = types.SimpleNamespace(LOCK_EX=1, LOCK_NB=2, flock=locked)
  monkeypatch.setitem(sys.modules, "fcntl", fake_fcntl)
  monkeypatch.setattr(agnos, "UPDATE_LOCK_FILE", tmp_path / "agnos.lock")

  with pytest.raises(RuntimeError, match="already running"):
    agnos.acquire_update_lock()


def test_swap_retries_are_bounded(monkeypatch, tmp_path: Path) -> None:
  manifest = tmp_path / "agnos.json"
  write_manifest(manifest)
  attempts = 0

  def failed_swap(*_args, **_kwargs):
    nonlocal attempts
    attempts += 1
    return "temporary failure"

  monkeypatch.setattr(agnos.subprocess, "check_output", failed_swap)
  monkeypatch.setattr(agnos.time, "sleep", lambda _seconds: None)

  with pytest.raises(RuntimeError, match="Failed to switch boot slot"):
    agnos.swap(str(manifest), 1, FakeLog())

  assert attempts == agnos.SWAP_MAX_ATTEMPTS


def test_swap_accepts_a_transient_abctl_failure(monkeypatch, tmp_path: Path) -> None:
  manifest = tmp_path / "agnos.json"
  write_manifest(manifest)
  outputs = iter(("No such file or directory", "Set slot 1 lun as boot lun"))
  monkeypatch.setattr(agnos.subprocess, "check_output", lambda *_args, **_kwargs: next(outputs))
  monkeypatch.setattr(agnos.time, "sleep", lambda _seconds: None)

  agnos.swap(str(manifest), 1, FakeLog())


@pytest.mark.parametrize("online_after", [30, 60])
def test_download_waits_for_wifi_and_can_restart_after_exhaustion(monkeypatch, tmp_path: Path, online_after: int) -> None:
  manifest = tmp_path / "agnos.json"
  write_manifest(manifest, ("https://downloads.example/image.xz",))
  elapsed = 0
  attempts = []

  def sleep(seconds):
    nonlocal elapsed
    elapsed += seconds

  def flash(*_args):
    attempts.append(elapsed)
    if elapsed < online_after:
      raise requests.ConnectionError("Wi-Fi not connected")

  monkeypatch.setattr(agnos.os, "system", lambda _cmd: 0)
  monkeypatch.setattr(agnos.time, "sleep", sleep)
  monkeypatch.setattr(agnos, "flash_partition", flash)

  if online_after > 40:
    with pytest.raises(RuntimeError, match="Check the connection or update server"):
      agnos.flash_agnos_update(str(manifest), 1, FakeLog(), standalone=True)
    assert attempts == [0, 10, 20, 30, 40]
    # Wi-Fi connects while the failure screen waits for the user's Retry tap.
    elapsed = online_after

  agnos.flash_agnos_update(str(manifest), 1, FakeLog(), standalone=True)
  assert attempts[-1] == online_after


@pytest.mark.parametrize("online_after", [60, 600])
def test_automatic_download_waits_until_connectivity_returns(monkeypatch, tmp_path, online_after):
  manifest = tmp_path / "agnos.json"
  write_manifest(manifest, ("https://downloads.example/image.xz",))
  elapsed = 0
  attempts = []

  def sleep(seconds):
    nonlocal elapsed
    elapsed += seconds

  def flash(*_args):
    attempts.append(elapsed)
    if elapsed < online_after:
      raise requests.ConnectionError("offline")

  monkeypatch.setattr(agnos.os, "system", lambda _cmd: 0)
  monkeypatch.setattr(agnos.time, "sleep", sleep)
  monkeypatch.setattr(agnos, "flash_partition", flash)
  agnos.flash_agnos_update(str(manifest), 1, FakeLog(), standalone=True, retry_network=True)
  assert attempts == list(range(0, online_after + 1, 10))


@pytest.mark.parametrize("status, transient", [(403, False), (404, False), (408, True), (429, True), (503, True)])
def test_http_retry_policy(status, transient):
  response = requests.Response()
  response.status_code = status
  assert agnos.transient_download_error(requests.exceptions.HTTPError(response=response)) == transient


@pytest.mark.parametrize("error_type, transient", [
  (requests.exceptions.ConnectionError, True), (requests.exceptions.Timeout, True),
  (requests.exceptions.ChunkedEncodingError, True), (requests.exceptions.SSLError, False),
  (requests.exceptions.InvalidURL, False),
])
def test_network_retry_policy(error_type, transient):
  assert agnos.transient_download_error(error_type("test")) == transient


def test_automatic_download_does_not_loop_forever_on_permanent_http_failure(monkeypatch, tmp_path):
  manifest = tmp_path / "agnos.json"
  write_manifest(manifest, ("https://downloads.example/missing.xz",))
  attempts = 0

  def flash(*_args):
    nonlocal attempts
    attempts += 1
    response = requests.Response()
    response.status_code = 404
    raise requests.exceptions.HTTPError(response=response)

  monkeypatch.setattr(agnos.os, "system", lambda _cmd: 0)
  monkeypatch.setattr(agnos.time, "sleep", lambda _seconds: None)
  monkeypatch.setattr(agnos, "flash_partition", flash)
  with pytest.raises(RuntimeError, match="Download failed after 5 attempts"):
    agnos.flash_agnos_update(str(manifest), 1, FakeLog(), standalone=True, retry_network=True)
  assert attempts == 5


@pytest.mark.parametrize("retry_network", [False, True])
def test_non_network_failure_is_not_retried(monkeypatch, tmp_path: Path, retry_network: bool) -> None:
  manifest = tmp_path / "agnos.json"
  write_manifest(manifest, ("https://downloads.example/image.xz",))

  def flash(*_args):
    raise OSError("partition write failed")

  def unexpected_sleep(_seconds):
    pytest.fail("A partition write failure must not be retried as a network failure")

  monkeypatch.setattr(agnos.os, "system", lambda _cmd: 0)
  monkeypatch.setattr(agnos.time, "sleep", unexpected_sleep)
  monkeypatch.setattr(agnos, "flash_partition", flash)
  with pytest.raises(OSError, match="partition write failed"):
    agnos.flash_agnos_update(str(manifest), 1, FakeLog(), standalone=True, retry_network=retry_network)


@pytest.mark.parametrize("already_ready", [False, True])
def test_automatic_cli_only_swaps_verified_images(monkeypatch, mocker, already_ready):
  path = Path(agnos.__file__)
  entrypoint = ast.parse(path.read_text(encoding="utf-8")).body[-1]
  monkeypatch.setattr(sys, "argv", [str(path), "--swap", "--retry-network", "/agnos.json"])
  namespace = vars(agnos).copy()
  namespace.update({
    "acquire_update_lock": mocker.Mock(), "get_target_slot_number": lambda: 1,
    "verify_agnos_update": mocker.Mock(side_effect=[True] if already_ready else [False, True]),
    "flash_agnos_update": mocker.Mock(), "swap": mocker.Mock(),
  })
  exec(compile(ast.Module(body=entrypoint.body, type_ignores=[]), str(path), "exec"), namespace)
  assert namespace["flash_agnos_update"].call_count == (0 if already_ready else 1)
  if not already_ready:
    assert namespace["flash_agnos_update"].call_args.kwargs == {"standalone": True, "retry_network": True}
  namespace["swap"].assert_called_once()
  assert namespace["swap"].call_args.args[:2] == ("/agnos.json", 1)


def test_automatic_cli_never_swaps_after_verification_failure(monkeypatch, mocker):
  path = Path(agnos.__file__)
  entrypoint = ast.parse(path.read_text(encoding="utf-8")).body[-1]
  monkeypatch.setattr(sys, "argv", [str(path), "--swap", "--retry-network", "/agnos.json"])
  namespace = vars(agnos).copy()
  namespace.update({
    "acquire_update_lock": mocker.Mock(), "get_target_slot_number": lambda: 1,
    "verify_agnos_update": mocker.Mock(return_value=False),
    "flash_agnos_update": mocker.Mock(), "swap": mocker.Mock(),
  })
  with pytest.raises(RuntimeError, match="verification failed"):
    exec(compile(ast.Module(body=entrypoint.body, type_ignores=[]), str(path), "exec"), namespace)
  assert namespace["flash_agnos_update"].call_count == agnos.VERIFY_FLASH_MAX_ATTEMPTS
  namespace["swap"].assert_not_called()
