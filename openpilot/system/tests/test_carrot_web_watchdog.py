import os
from pathlib import Path
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
