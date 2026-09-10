import ast
import os
from pathlib import Path
import signal
import subprocess
import sys
import time
from types import SimpleNamespace

import pytest

from openpilot.system.updated.process import run


def test_output_cwd_and_failure(tmp_path):
  assert run([sys.executable, "-c", "import os; print(os.getcwd())"], str(tmp_path)).strip() == str(tmp_path)
  with pytest.raises(subprocess.CalledProcessError) as error:
    run([sys.executable, "-c", "import sys; print('failure detail', flush=True); sys.exit(7)"])
  assert error.value.returncode == 7
  assert error.value.output == "failure detail\n"


def test_git_children_inherit_disabled_maintenance(tmp_path, monkeypatch):
  monkeypatch.setenv("GIT_CONFIG_NOSYSTEM", "1")
  monkeypatch.setenv("GIT_CONFIG_GLOBAL", os.devnull)
  monkeypatch.setenv("GIT_CONFIG_COUNT", "2")
  monkeypatch.setenv("GIT_CONFIG_KEY_0", "test.preserved")
  monkeypatch.setenv("GIT_CONFIG_VALUE_0", "keep")
  monkeypatch.setenv("GIT_CONFIG_KEY_1", "gc.auto")
  monkeypatch.setenv("GIT_CONFIG_VALUE_1", "1")
  subprocess.run(["git", "init", str(tmp_path)], check=True, capture_output=True)
  for key, value in (("gc.auto", "0"), ("gc.autoDetach", "false"), ("maintenance.auto", "false"), ("test.preserved", "keep")):
    # A nested invocation represents Git started by submodule/shell commands.
    code = f"import subprocess; subprocess.run(['git', 'config', '--get', {key!r}], check=True)"
    assert run([sys.executable, "-c", code], str(tmp_path)).strip() == value
  assert os.environ["GIT_CONFIG_COUNT"] == "2"


def alive(pid):
  try:
    # Zombies no longer execute or retain memory/file descriptors.
    return Path(f"/proc/{pid}/stat").read_text().split(")", 1)[1].split()[0] != "Z"
  except FileNotFoundError:
    return False


def wait_until(predicate):
  deadline = time.monotonic() + 8
  while not predicate():
    assert time.monotonic() < deadline, "child process did not reach expected state"
    time.sleep(0.02)


@pytest.mark.skipif(sys.platform != "linux", reason="Linux signals, process groups and /proc")
@pytest.mark.parametrize("parent_exits", [False, True])
@pytest.mark.parametrize("legacy", [False, True])
def test_sigint_git_style_worker_lifetime(tmp_path, parent_exits, legacy):
  # updated -> git gc -> repack/pack-objects. The worker ignores interrupts and
  # holds the output pipe even after gc exits. The old helper must reproduce
  # the orphan; the replacement must stop it before returning from SIGINT.
  pid_file = tmp_path / "worker.pid"
  worker = ("import os, signal, time; from pathlib import Path; "
            + "signal.signal(signal.SIGINT, signal.SIG_IGN); signal.signal(signal.SIGTERM, signal.SIG_IGN); "
            + f"Path({str(pid_file)!r}).write_text(str(os.getpid())); time.sleep(60)")
  git = (f"import subprocess, sys, time; subprocess.Popen([sys.executable, '-c', {worker!r}]); "
         + ("sys.exit(0)" if parent_exits else "time.sleep(60)"))
  call = ("subprocess.check_output(cmd, stderr=subprocess.STDOUT, encoding='utf8')" if legacy else "run(cmd)")
  updater = ("import subprocess, sys; from openpilot.system.updated.process import run\n"
             + f"cmd = [sys.executable, '-c', {git!r}]\ntry:\n  {call}\nexcept KeyboardInterrupt:\n  pass\n")
  proc = subprocess.Popen([sys.executable, "-c", updater], start_new_session=True,
                          stdout=subprocess.PIPE, stderr=subprocess.PIPE)
  worker_pid = None
  try:
    wait_until(lambda: pid_file.exists() and bool(pid_file.read_text()))
    worker_pid = int(pid_file.read_text())
    os.kill(proc.pid, signal.SIGINT)
    proc.wait(timeout=4)
    assert proc.returncode == 0
    if legacy:
      assert alive(worker_pid), "legacy helper should reproduce the surviving Git worker"
    else:
      wait_until(lambda: not alive(worker_pid))
  finally:
    # Explicitly reap the reproduction's orphan, even when an assertion fails.
    if worker_pid is not None and alive(worker_pid):
      os.kill(worker_pid, signal.SIGKILL)
    try:
      os.killpg(proc.pid, signal.SIGKILL)
    except ProcessLookupError:
      pass
    proc.communicate(timeout=4)


def finalize_namespace(interrupt=False):
  # Exercise the real finalization function without AGNOS/mount dependencies.
  source = Path(__file__).parents[1] / "updated.py"
  tree = ast.parse(source.read_text())
  function = next(n for n in tree.body if isinstance(n, ast.FunctionDef) and n.name == "finalize_update")
  flags, commands = [], []

  def record(cmd, cwd):
    commands.append(cmd)
    if interrupt and cmd == ["git", "lfs", "prune"]:
      raise KeyboardInterrupt
    return ""

  def noop(*args, **kwargs):
    pass

  ns = {"os": SimpleNamespace(path=SimpleNamespace(exists=lambda _: False)),
        "shutil": SimpleNamespace(copytree=noop),
        "cloudlog": SimpleNamespace(info=noop, event=noop, exception=noop),
        "time": time, "subprocess": subprocess, "FINALIZED": "finalized", "OVERLAY_MERGED": "merged",
        "set_consistent_flag": flags.append, "run": record, "flags": flags, "commands": commands}
  exec(compile(ast.Module(body=[function], type_ignores=[]), str(source), "exec"), ns)
  return ns


def test_finalization_keeps_lfs_cleanup_without_full_repack():
  ns = finalize_namespace()
  ns["finalize_update"]()
  commands = ns["commands"]
  assert commands == [["git", "reset", "--hard"],
                      ["git", "submodule", "foreach", "--recursive", "git", "reset", "--hard"],
                      ["git", "lfs", "prune"]]
  assert ns["flags"] == [False, True]


def test_interrupted_finalization_never_marks_update_ready():
  ns = finalize_namespace(interrupt=True)
  with pytest.raises(KeyboardInterrupt):
    ns["finalize_update"]()
  assert ns["flags"] == [False]
