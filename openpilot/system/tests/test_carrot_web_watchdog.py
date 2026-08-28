import os
from pathlib import Path
import signal
import shutil
import subprocess
import sys
import time

import pytest

from openpilot.common.basedir import BASEDIR


def _wait_for_line(path: Path, expected: str, timeout: float = 5.0) -> None:
  deadline = time.monotonic() + timeout
  while time.monotonic() < deadline:
    if path.exists() and expected in path.read_text(encoding="utf-8").splitlines():
      return
    time.sleep(0.05)
  output = path.read_text(encoding="utf-8") if path.exists() else "<missing>"
  pytest.fail(f"timed out waiting for {expected!r} in watchdog output:\n{output}")


def _wait_for_line_count(path: Path, expected: int, timeout: float = 5.0) -> None:
  deadline = time.monotonic() + timeout
  while time.monotonic() < deadline:
    if path.exists() and len(path.read_text(encoding="utf-8").splitlines()) >= expected:
      return
    time.sleep(0.05)
  output = path.read_text(encoding="utf-8") if path.exists() else "<missing>"
  pytest.fail(f"timed out waiting for {expected} watchdog output lines:\n{output}")


def _write_fake_command(path: Path, body: str) -> None:
  path.write_text(f"#!/usr/bin/env bash\n{body}\n", encoding="utf-8")
  path.chmod(0o755)


@pytest.mark.skipif(sys.platform == "win32", reason="requires a POSIX shell")
def test_restart_waits_for_old_web_watchdog_before_relaunch(tmp_path: Path) -> None:
  bash = shutil.which("bash")
  if bash is None:
    pytest.skip("bash is unavailable")

  fake_bin = tmp_path / "bin"
  fake_bin.mkdir()
  trace = tmp_path / "trace"
  pgrep_count = tmp_path / "pgrep_count"

  for name in ("git", "pkill", "rm", "sleep", "tmux", "bash"):
    _write_fake_command(
      fake_bin / name,
      f"printf '{name}:%s\\n' \"$*\" >> \"$CARROT_WEB_TEST_TRACE\"\nexit 0",
    )

  _write_fake_command(
    fake_bin / "pgrep",
    """printf 'pgrep:%s\\n' "$*" >> "$CARROT_WEB_TEST_TRACE"
if [[ "$*" == *"carrot_web_watchdog"* ]]; then
  count="$(cat "$CARROT_WEB_TEST_PGREP_COUNT" 2>/dev/null || printf 0)"
  count=$((count + 1))
  printf '%s\\n' "$count" > "$CARROT_WEB_TEST_PGREP_COUNT"
  if [ "$count" -eq 1 ]; then
    exit 0
  fi
fi
exit 1""",
  )

  env = os.environ.copy()
  env.update({
    "PATH": f"{fake_bin}{os.pathsep}{env.get('PATH', '')}",
    "CARROT_WEB_TEST_TRACE": str(trace),
    "CARROT_WEB_TEST_PGREP_COUNT": str(pgrep_count),
  })
  result = subprocess.run(
    [bash, str(Path(BASEDIR) / "restart.sh")],
    env=env,
    capture_output=True,
    text=True,
    timeout=5,
    check=False,
  )

  assert result.returncode == 0, result.stdout + result.stderr
  lines = trace.read_text(encoding="utf-8").splitlines()
  watchdog_checks = [i for i, line in enumerate(lines) if "pgrep:-f [c]arrot_web_watchdog" in line]
  server_checks = [i for i, line in enumerate(lines) if "pgrep:-f [o]penpilot.selfdrive.carrot.carrot_server" in line]
  relaunch = next(i for i, line in enumerate(lines) if line.startswith("tmux:new -s comma"))

  assert len(watchdog_checks) == 2
  assert server_checks
  assert max(*watchdog_checks, *server_checks) < relaunch


@pytest.mark.skipif(sys.platform == "win32", reason="requires a POSIX shell and deleted-cwd semantics")
def test_watchdog_reenters_checkout_after_directory_replacement(tmp_path: Path) -> None:
  checkout = tmp_path / "openpilot"
  checkout.mkdir()
  (checkout / "generation").write_text("old", encoding="utf-8")

  launches = tmp_path / "launches"
  fake_python = tmp_path / "fake_python.sh"
  fake_python.write_text(
    "\n".join([
      "#!/usr/bin/env bash",
      'if physical_pwd="$(pwd -P 2>/dev/null)" && generation="$(cat generation 2>/dev/null)"; then',
      "  printf '%s:%s\\n' \"$generation\" \"$physical_pwd\" >> \"$CARROT_WEB_TEST_LAUNCHES\"",
      "else",
      "  printf 'invalid-cwd\\n' >> \"$CARROT_WEB_TEST_LAUNCHES\"",
      "fi",
      "exit 1",
      "",
    ]),
    encoding="utf-8",
  )
  fake_python.chmod(0o755)

  env = os.environ.copy()
  env.update({
    "CARROT_WEB_PID_FILE": str(tmp_path / "watchdog.pid"),
    "CARROT_WEB_LOCK_FILE": str(tmp_path / "watchdog.lock"),
    "CARROT_WEB_RESTART_DELAY": "0.05",
    "CARROT_WEB_TEST_LAUNCHES": str(launches),
  })
  watchdog = Path(BASEDIR) / "scripts" / "carrot_web_watchdog.sh"
  process = subprocess.Popen(
    ["bash", str(watchdog), str(checkout), str(fake_python)],
    cwd=checkout,
    env=env,
    stdout=subprocess.PIPE,
    stderr=subprocess.STDOUT,
    text=True,
  )

  try:
    _wait_for_line(launches, f"old:{checkout}")

    shutil.rmtree(checkout)
    checkout.mkdir()
    (checkout / "generation").write_text("new", encoding="utf-8")

    _wait_for_line(launches, f"new:{checkout}")
  finally:
    process.terminate()
    try:
      process.wait(timeout=2)
    except subprocess.TimeoutExpired:
      process.kill()
      process.wait(timeout=2)


@pytest.mark.skipif(sys.platform == "win32", reason="requires a POSIX shell and flock")
def test_watchdog_allows_only_one_instance(tmp_path: Path) -> None:
  if shutil.which("flock") is None:
    pytest.skip("flock is unavailable")

  checkout = tmp_path / "openpilot"
  checkout.mkdir()
  launches = tmp_path / "launches"
  fake_python = tmp_path / "fake_python.sh"
  fake_python.write_text(
    "#!/usr/bin/env bash\nprintf 'launch:%s\\n' \"$$\" >> \"$CARROT_WEB_TEST_LAUNCHES\"\nsleep 10\n",
    encoding="utf-8",
  )
  fake_python.chmod(0o755)

  watchdog = Path(BASEDIR) / "scripts" / "carrot_web_watchdog.sh"
  base_env = os.environ.copy()
  base_env.update({
    "CARROT_WEB_LOCK_FILE": str(tmp_path / "watchdog.lock"),
    "CARROT_WEB_RESTART_DELAY": "0.05",
    "CARROT_WEB_TEST_LAUNCHES": str(launches),
  })
  first_env = {**base_env, "CARROT_WEB_PID_FILE": str(tmp_path / "first.pid")}
  second_env = {**base_env, "CARROT_WEB_PID_FILE": str(tmp_path / "second.pid")}

  first = subprocess.Popen(
    ["bash", str(watchdog), str(checkout), str(fake_python)],
    env=first_env,
    start_new_session=True,
  )
  try:
    _wait_for_line_count(launches, 1)
    second = subprocess.run(
      ["bash", str(watchdog), str(checkout), str(fake_python)],
      env=second_env,
      capture_output=True,
      text=True,
      timeout=2,
      check=False,
    )

    assert second.returncode == 0
    assert "another watchdog already holds" in second.stdout
    time.sleep(0.2)
    assert len(launches.read_text(encoding="utf-8").splitlines()) == 1
  finally:
    os.killpg(first.pid, signal.SIGTERM)
    try:
      first.wait(timeout=2)
    except subprocess.TimeoutExpired:
      os.killpg(first.pid, signal.SIGKILL)
      first.wait(timeout=2)
