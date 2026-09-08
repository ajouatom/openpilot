import json
import subprocess

import pytest

from ..auto_update import Updater, pinned_compose
from ..deploy_probe import check_payload


OLD = "sha256:" + "1" * 64
NEW = "sha256:" + "2" * 64
COMPOSE = 'services:\n  carrot-upload:\n    image: old:tag\n    volumes: ["/data:/data"]\n'
REPORT = {"sourceCommit": "a" * 40, "replay": {"frames": 2, "sourceVersion": "v", "replayHash": "hash"}}


class FakeUpdater(Updater):
  def wait_healthy(self):
    pass

  def __init__(self, root):
    self.running = OLD
    self.candidate = NEW
    self.commands = []
    self.probes = []
    self.fail = None
    (root / "compose.yaml").write_text(COMPOSE)
    super().__init__(root, {"image": "registry/vault:carrot-wip"})

  def run(self, *args, timeout=600):
    self.commands.append(args)
    if args[:3] == ("docker", "image", "inspect"):
      return self.candidate
    if args[:2] == ("docker", "inspect"):
      return self.running
    if "ps" in args:
      return "production-container"
    if "up" in args:
      self.running = NEW if NEW in self.compose_path.read_text() else OLD
    return ""

  def probe(self, image, online=False):
    self.probes.append((image, online))
    if self.fail == ("online" if online else "offline"):
      raise RuntimeError("probe failed")
    return REPORT


def test_update_is_pinned_and_verified_before_success(tmp_path):
  updater = FakeUpdater(tmp_path)
  updater.update()
  assert updater.running == NEW
  assert updater.probes == [(NEW, False), (NEW, True)]
  assert updater.state["sourceCommit"] == REPORT["sourceCommit"]
  assert updater.state["previousImage"] == OLD
  assert '/data:/data' in updater.compose_path.read_text()
  assert not updater.journal_path.exists()
  count = len(updater.commands)
  updater.update()
  assert not any("up" in c for c in updater.commands[count:])


@pytest.mark.parametrize("failure", ["offline", "online"])
def test_failure_preserves_or_restores_service_and_quarantines(tmp_path, failure):
  updater = FakeUpdater(tmp_path)
  updater.fail = failure
  with pytest.raises(RuntimeError, match="probe failed"):
    updater.update()
  assert updater.running == OLD
  assert updater.state["failedImage"] == NEW
  assert not updater.journal_path.exists()
  probes = list(updater.probes)
  updater.update()
  assert updater.probes == probes
  assert updater.state["status"] == "quarantined"


def test_interrupted_transaction_rolls_back_before_new_pull(tmp_path):
  updater = FakeUpdater(tmp_path)
  updater.running = NEW
  updater.compose_path.write_text(pinned_compose(COMPOSE, NEW))
  updater.journal_path.write_text(json.dumps({"candidate": NEW, "previousImage": OLD,
                                            "previousCompose": pinned_compose(COMPOSE, OLD)}))
  updater.update()
  assert updater.running == OLD
  assert updater.probes == []
  up = next(i for i, c in enumerate(updater.commands) if "up" in c)
  pull = next(i for i, c in enumerate(updater.commands) if "pull" in c)
  assert up < pull


def test_failed_rollback_keeps_recovery_journal(tmp_path, monkeypatch):
  updater = FakeUpdater(tmp_path)
  updater.fail = "online"
  real_run = updater.run

  def run(*args, **kwargs):
    if "up" in args and OLD in updater.compose_path.read_text():
      raise subprocess.CalledProcessError(1, args)
    return real_run(*args, **kwargs)

  monkeypatch.setattr(updater, "run", run)
  with pytest.raises(subprocess.CalledProcessError):
    updater.update()
  assert updater.journal_path.exists()


def test_rejects_ambiguous_compose_and_mutable_images():
  with pytest.raises(ValueError):
    pinned_compose(COMPOSE, "registry:latest")
  with pytest.raises(ValueError):
    pinned_compose(COMPOSE + "    image: extra\n", NEW)


def test_probe_passes_private_config_on_stdin_without_acl_mount(tmp_path, monkeypatch):
  updater = FakeUpdater(tmp_path)
  updater.config["data_root"] = "/volume1/openpilot"
  updater.expected_path.write_text(json.dumps(REPORT))
  calls = []

  def run(*args, **kwargs):
    calls.append((args, kwargs))
    return json.dumps(REPORT)

  monkeypatch.setattr(updater, "run", run)
  assert Updater.probe(updater, NEW, online=True) == REPORT
  args, kwargs = next((a, k) for a, k in calls if a[:2] == ("docker", "run"))
  assert "-i" in args and "--online" in args
  assert not any("/probe/" in a for a in args)
  assert json.loads(kwargs["stdin_data"]) == {"config": updater.config, "expected": REPORT["replay"]}


def test_real_route_probe_requires_nonempty_regression_window():
  payload = {"schemaVersion": 1, "sourceVersion": "v", "graphs": {}, "frames": [
    {"video_time_s": 52, "selection": {"cutin_diagnostics": [{"track_id": 42, "stage": "CUT-IN"}]}}]}
  config = {"forbidden_cutin": [{"start": 50, "end": 55, "track_id": 42}]}
  with pytest.raises(ValueError, match="regression"):
    check_payload(payload, config)
  payload["frames"][0]["video_time_s"] = 20
  with pytest.raises(ValueError, match="no video-aligned"):
    check_payload(payload, config)


def test_bundle_excludes_uncommitted_viewer(tmp_path, monkeypatch):
  from .. import build_bundle
  repo = tmp_path / "repo"
  repo.mkdir()
  subprocess.run(["git", "init", str(repo)], check=True, capture_output=True)
  names = ("server.py", "viewer.py", "radar.py", "radar_view.js", "requirements.txt", "Dockerfile.vault", "deploy_probe.py")
  source = repo / "tools/carrot_route_vault"
  source.mkdir(parents=True)
  for name in names:
    (source / name).write_text("committed")
  subprocess.run(["git", "-C", str(repo), "add", "."], check=True)
  subprocess.run(["git", "-C", str(repo), "-c", "user.name=Test", "-c", "user.email=test@example.com",
                  "commit", "-m", "fixture"], check=True, capture_output=True)
  (source / "server.py").write_text("dirty local change")
  monkeypatch.setattr(build_bundle, "ROOT", repo)
  build_bundle.build(tmp_path / "bundle", "HEAD")
  assert (tmp_path / "bundle/server.py").read_text() == "committed"
