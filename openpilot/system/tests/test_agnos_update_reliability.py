import importlib.util
import json
from pathlib import Path
import sys
import types

import pytest

try:
  import requests
except ModuleNotFoundError:
  requests = types.ModuleType("requests")

  class RequestException(Exception):
    pass

  requests.ConnectionError = RequestException
  requests.exceptions = types.SimpleNamespace(RequestException=RequestException)
  sys.modules["requests"] = requests


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
