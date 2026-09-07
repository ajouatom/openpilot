import asyncio
import importlib.util
import json
import os
from pathlib import Path
import subprocess
import sys
import types

import pytest

from openpilot.selfdrive.carrot.server.services.git_config import repair_git_config


def git(repo, *args, check=True):
  proc = subprocess.run(["git", "-C", str(repo), *args], capture_output=True, text=True, encoding="utf-8", errors="replace")
  if check:
    assert proc.returncode == 0, proc.stdout + proc.stderr
  return proc.stdout.strip() if check else proc


@pytest.fixture(autouse=True)
def isolated_git(monkeypatch):
  monkeypatch.setenv("GIT_CONFIG_GLOBAL", os.devnull)
  monkeypatch.setenv("GIT_CONFIG_NOSYSTEM", "1")
  monkeypatch.setenv("GIT_TERMINAL_PROMPT", "0")


def checkout(tmp_path, branch="carrot-wip"):
  seed, remote, device = (tmp_path / name for name in ("seed", "origin.git", "device"))
  seed.mkdir()
  git(seed, "init", "-b", branch)
  git(seed, "config", "user.name", "Git repair test")
  git(seed, "config", "user.email", "test@example.invalid")
  (seed / "version.txt").write_text("old\n")
  git(seed, "add", "version.txt")
  git(seed, "commit", "-qm", "initial")
  git(tmp_path, "clone", "--bare", str(seed), str(remote))
  git(tmp_path, "clone", "--depth=1", "--branch", branch, remote.as_uri(), str(device))
  (seed / "version.txt").write_text("new\n")
  git(seed, "commit", "-qam", "update")
  git(seed, "push", str(remote), branch)
  return seed, remote, device


@pytest.mark.parametrize("branch", ["carrot-wip", "carrot-cinque-terre", "carrot-bmr_v6"])
def test_migrated_fetch_and_upstream_repaired_without_switching_model(tmp_path, branch):
  _, remote, device = checkout(tmp_path, branch)
  old_spec = "+refs/heads/release-tizi-staging:refs/remotes/origin/release-tizi-staging"
  git(device, "config", "--replace-all", "remote.origin.fetch", old_spec)
  git(device, "config", f"branch.{branch}.merge", "refs/heads/release-tizi-staging")
  before = git(device, "rev-parse", "HEAD")
  failed = git(device, "pull", "--ff-only", check=False)
  assert failed.returncode != 0
  assert "couldn't find remote ref refs/heads/release-tizi-staging" in failed.stderr

  rc, output = repair_git_config(str(device))

  assert rc == 0, output
  assert "Removed obsolete fetch ref" in output
  assert f"Upstream repaired: {branch} -> origin/{branch}" in output
  assert git(device, "branch", "--show-current") == branch
  assert git(device, "rev-parse", "HEAD") == before
  assert git(device, "remote", "get-url", "origin") == remote.as_uri()
  git(device, "pull", "--ff-only")
  assert git(device, "rev-parse", "HEAD") == git(remote, "rev-parse", f"refs/heads/{branch}")


def test_removes_duplicate_stale_refs_but_preserves_custom_mappings_and_changes(tmp_path):
  _, _, device = checkout(tmp_path)
  old_spec = "+refs/heads/deleted:refs/remotes/origin/deleted"
  for spec in [old_spec, old_spec, "+refs/tags/*:refs/tags/*", "^refs/heads/private/*"]:
    git(device, "config", "--add", "remote.origin.fetch", spec)
  (device / "version.txt").write_text("local staged change\n")
  git(device, "add", "version.txt")
  (device / "untracked.txt").write_text("keep me\n")
  status = git(device, "status", "--porcelain")
  index = git(device, "write-tree")
  head = git(device, "rev-parse", "HEAD")

  rc, output = repair_git_config(str(device))

  assert rc == 0, output
  specs = git(device, "config", "--get-all", "remote.origin.fetch").splitlines()
  assert old_spec not in specs
  assert "+refs/tags/*:refs/tags/*" in specs
  assert "^refs/heads/private/*" in specs
  assert git(device, "status", "--porcelain") == status
  assert git(device, "write-tree") == index
  assert git(device, "rev-parse", "HEAD") == head


def test_valid_differently_named_upstream_and_non_origin_remote_are_preserved(tmp_path):
  _, _, device = checkout(tmp_path)
  git(device, "remote", "rename", "origin", "my-fork")
  git(device, "branch", "-m", "my-local-branch")
  rc, output = repair_git_config(str(device))
  assert rc == 0, output
  assert git(device, "rev-parse", "--abbrev-ref", "@{upstream}") == "my-fork/carrot-wip"
  assert git(device, "branch", "--show-current") == "my-local-branch"
  before = (device / ".git/config").read_bytes()
  rc, output = repair_git_config(str(device))
  assert rc == 0, output
  assert (device / ".git/config").read_bytes() == before


def test_missing_upstream_reconnected_by_exact_current_branch_name(tmp_path):
  _, _, device = checkout(tmp_path)
  git(device, "branch", "--unset-upstream")
  rc, output = repair_git_config(str(device))
  assert rc == 0, output
  assert git(device, "rev-parse", "--abbrev-ref", "@{upstream}") == "origin/carrot-wip"


def test_missing_remote_branch_does_not_guess_a_model_or_modify_config(tmp_path):
  _, _, device = checkout(tmp_path)
  git(device, "branch", "-m", "carrot-missing-model")
  git(device, "config", "branch.carrot-missing-model.merge", "refs/heads/deleted")
  before = (device / ".git/config").read_bytes()
  rc, output = repair_git_config(str(device))
  assert rc != 0
  assert "No valid upstream or matching remote branch" in output
  assert (device / ".git/config").read_bytes() == before
  assert git(device, "branch", "--show-current") == "carrot-missing-model"


def test_unreachable_remote_preserves_config_and_worktree(tmp_path):
  _, _, device = checkout(tmp_path)
  git(device, "remote", "set-url", "origin", str(tmp_path / "missing.git"))
  before = (device / ".git/config").read_bytes()
  (device / "version.txt").write_text("keep local change\n")
  rc, output = repair_git_config(str(device))
  assert rc != 0
  assert "Git configuration repair failed" in output
  assert (device / ".git/config").read_bytes() == before
  assert (device / "version.txt").read_text() == "keep local change\n"


def test_remote_change_can_repair_fetch_before_user_selects_a_new_branch(tmp_path):
  _, _, device = checkout(tmp_path)
  git(device, "branch", "-m", "release-tizi-staging")
  git(device, "config", "--replace-all", "remote.origin.fetch",
      "+refs/heads/release-tizi-staging:refs/remotes/origin/release-tizi-staging")
  rc, output = repair_git_config(str(device), remote="origin", repair_upstream=False)
  assert rc == 0, output
  assert git(device, "branch", "--show-current") == "release-tizi-staging"
  git(device, "fetch", "origin")
  assert git(device, "rev-parse", "--verify", "refs/remotes/origin/carrot-wip")


def test_detached_head_remains_detached(tmp_path):
  _, _, device = checkout(tmp_path)
  git(device, "checkout", "--detach")
  head = git(device, "rev-parse", "HEAD")
  rc, output = repair_git_config(str(device))
  assert rc == 0, output
  assert git(device, "branch", "--show-current") == ""
  assert git(device, "rev-parse", "HEAD") == head


def test_fetch_failure_does_not_reconnect_upstream(tmp_path, monkeypatch):
  _, _, device = checkout(tmp_path)
  git(device, "config", "branch.carrot-wip.merge", "refs/heads/deleted")
  real_run = subprocess.run

  def fail_fetch(argv, **kwargs):
    if argv[:2] == ["git", "fetch"]:
      return subprocess.CompletedProcess(argv, 1, "", "fetch failed")
    return real_run(argv, **kwargs)

  monkeypatch.setattr(subprocess, "run", fail_fetch)
  rc, output = repair_git_config(str(device))
  assert rc != 0
  assert "fetch failed" in output
  assert git(device, "config", "branch.carrot-wip.merge") == "refs/heads/deleted"


@pytest.fixture
def web_dispatcher(tmp_path, monkeypatch):
  """Load the real tool dispatcher without starting vehicle-only feature imports."""
  _, _, device = checkout(tmp_path)
  server_dir = Path(__file__).parents[1]
  base = "openpilot.selfdrive.carrot.server.features"
  for name, directory in [(base, server_dir / "features"), (base + ".tools", server_dir / "features/tools")]:
    package = types.ModuleType(name)
    package.__path__ = [str(directory)]
    monkeypatch.setitem(sys.modules, name, package)
  hardware = types.ModuleType("openpilot.system.hardware")
  hardware.HARDWARE = types.SimpleNamespace(get_device_type=lambda: "tici")
  monkeypatch.setitem(sys.modules, hardware.__name__, hardware)
  loaded = {}
  for name in ("jobs", "actions", "dispatcher"):
    module_name = f"{base}.tools.{name}"
    spec = importlib.util.spec_from_file_location(module_name, server_dir / f"features/tools/{name}.py")
    module = importlib.util.module_from_spec(spec)
    monkeypatch.setitem(sys.modules, module_name, module)
    spec.loader.exec_module(module)
    loaded[name] = module
  dispatcher = loaded["dispatcher"]
  monkeypatch.setattr(dispatcher.jobs, "persist_changed", lambda: None)
  monkeypatch.setattr(dispatcher, "write_git_pull_time", lambda: None)
  monkeypatch.setattr(dispatcher, "clear_recovered_git_ref_error", lambda: None)
  real_run = subprocess.run
  commands = []

  def run(argv, **kwargs):
    if kwargs.get("cwd") == "/data/openpilot":
      kwargs["cwd"] = str(device)
      commands.append(argv)
    return real_run(argv, **kwargs)

  async def capture(argv, *, cwd, **kwargs):
    result = run(argv, cwd=cwd, timeout=kwargs["timeout"], capture_output=True, text=True)
    return result.returncode, result.stdout + result.stderr

  async def stream(job, argv, *, cwd, **kwargs):
    rc, out = await capture(argv, cwd=cwd, **kwargs)
    dispatcher.jobs.append(job, out)
    return rc

  monkeypatch.setattr(subprocess, "run", run)
  monkeypatch.setattr(dispatcher.jobs, "capture_exec", capture)
  monkeypatch.setattr(dispatcher.jobs, "stream_exec", stream)
  return dispatcher, device, commands


def dispatch(dispatcher, api, action, **payload):
  if api == "job":
    job = {"id": "git-repair-test", "action": action, "payload": payload}
    asyncio.run(dispatcher.run_tool_job(job))
    return job["result"]
  response = asyncio.run(dispatcher.dispatch_sync(None, {"action": action, **payload}))
  return json.loads(response.text)


@pytest.mark.parametrize("api", ["job", "sync"])
@pytest.mark.parametrize("mode", ["hard", "mixed", "soft"])
def test_web_reset_repairs_migration_before_requested_reset(web_dispatcher, api, mode):
  dispatcher, device, commands = web_dispatcher
  git(device, "config", "--replace-all", "remote.origin.fetch",
      "+refs/heads/release-tizi-staging:refs/remotes/origin/release-tizi-staging")
  git(device, "config", "branch.carrot-wip.merge", "refs/heads/release-tizi-staging")
  (device / "version.txt").write_text("user change\n")

  result = dispatch(dispatcher, api, "git_reset", mode=mode, target="HEAD")

  assert result["ok"], result
  assert git(device, "rev-parse", "--abbrev-ref", "@{upstream}") == "origin/carrot-wip"
  assert commands.index(["git", "fetch", "--prune", "--no-recurse-submodules", "origin"]) < commands.index(["git", "reset", f"--{mode}", "HEAD"])
  assert (device / "version.txt").read_text() == ("old\n" if mode == "hard" else "user change\n")


@pytest.mark.parametrize("api", ["job", "sync"])
def test_web_pull_repairs_config_and_updates_current_branch(web_dispatcher, api):
  dispatcher, device, _ = web_dispatcher
  git(device, "config", "--replace-all", "remote.origin.fetch",
      "+refs/heads/release-tizi-staging:refs/remotes/origin/release-tizi-staging")
  result = dispatch(dispatcher, api, "git_pull")
  assert result["ok"], result
  assert (device / "version.txt").read_text() == "new\n"
  assert git(device, "branch", "--show-current") == "carrot-wip"


@pytest.mark.parametrize("api", ["job", "sync"])
@pytest.mark.parametrize("action", ["git_reset", "git_pull"])
def test_web_repair_failure_does_not_reset_or_pull(web_dispatcher, api, action):
  dispatcher, device, commands = web_dispatcher
  git(device, "remote", "set-url", "origin", str(device / "missing.git"))
  (device / "version.txt").write_text("keep my changes\n")
  result = dispatch(dispatcher, api, action, mode="hard", target="HEAD")
  assert not result["ok"], result
  assert not any(cmd[:2] in (["git", "reset"], ["git", "pull"]) for cmd in commands)
  assert (device / "version.txt").read_text() == "keep my changes\n"


@pytest.mark.parametrize("api", ["job", "sync"])
@pytest.mark.parametrize("action", ["git_remote_set", "git_remote_add"])
def test_web_remote_change_removes_inherited_fetch_ref(web_dispatcher, api, action):
  dispatcher, device, _ = web_dispatcher
  url = git(device, "remote", "get-url", "origin")
  git(device, "config", "--replace-all", "remote.origin.fetch",
      "+refs/heads/release-tizi-staging:refs/remotes/origin/release-tizi-staging")
  result = dispatch(dispatcher, api, action, name="origin", url=url)
  assert result["ok"], result
  git(device, "fetch", "origin")
