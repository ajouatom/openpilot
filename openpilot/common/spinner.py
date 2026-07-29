import os
import subprocess
import sys

from openpilot.common.basedir import BASEDIR


class Spinner:
  MAX_START_ATTEMPTS = 2

  def __init__(self):
    self.spinner_proc = None
    self._start_attempts = 0
    self._start()

  def _start(self) -> bool:
    if self._start_attempts >= self.MAX_START_ATTEMPTS:
      return False

    self._start_attempts += 1
    try:
      self.spinner_proc = subprocess.Popen([sys.executable, "./spinner.py"],
                                           stdin=subprocess.PIPE,
                                           cwd=os.path.join(BASEDIR, "openpilot", "system", "ui"),
                                           close_fds=True)
    except OSError as e:
      print(f"WARNING: failed to start build spinner: {e}")
      self.spinner_proc = None
      return False
    return True

  def _stop_process(self):
    spinner_proc = self.spinner_proc
    self.spinner_proc = None
    if spinner_proc is None:
      return

    try:
      if spinner_proc.poll() is None:
        spinner_proc.kill()
      spinner_proc.communicate(timeout=2.)
    except subprocess.TimeoutExpired:
      print("WARNING: failed to kill build spinner")
    except (OSError, ValueError):
      pass

  def _ensure_running(self) -> bool:
    if self.spinner_proc is not None and self.spinner_proc.poll() is None:
      return True

    if self.spinner_proc is not None:
      print(f"WARNING: build spinner exited with code {self.spinner_proc.returncode}; restarting")
      self._stop_process()
    return self._start()

  def _send(self, payload: bytes) -> bool:
    if self.spinner_proc is None or self.spinner_proc.stdin is None:
      return False

    try:
      self.spinner_proc.stdin.write(payload)
      self.spinner_proc.stdin.flush()
      return True
    except (BrokenPipeError, OSError, ValueError) as e:
      print(f"WARNING: build spinner stopped accepting updates: {e}")
      return False

  def __enter__(self):
    return self

  def update(self, spinner_text: str) -> bool:
    payload = spinner_text.encode('utf8') + b"\n"
    if not self._ensure_running():
      return False

    if self._send(payload):
      return True

    # SCons should continue even if the UI dies. Restart once and replay the
    # latest status so a transient GPU/font failure does not leave the splash
    # screen up for the rest of the build.
    self._stop_process()
    if not self._start():
      return False

    sent = self._send(payload)
    if not sent:
      self._stop_process()
    return sent

  def update_progress(self, cur: float, total: float) -> bool:
    return self.update(str(round(100 * cur / total)))

  def close(self):
    self._stop_process()

  def __del__(self):
    try:
      self.close()
    except Exception:
      pass

  def __exit__(self, exc_type, exc_value, traceback):
    self.close()


if __name__ == "__main__":
  import time
  with Spinner() as s:
    s.update("Spinner text")
    time.sleep(5.0)
  print("gone")
  time.sleep(5.0)
