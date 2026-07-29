import sys

import openpilot.common.spinner as spinner_module
from openpilot.common.spinner import Spinner


class FakeStdin:
  def __init__(self, broken=False):
    self.broken = broken
    self.writes = []
    self.flushes = 0

  def write(self, payload):
    if self.broken:
      raise BrokenPipeError
    self.writes.append(payload)

  def flush(self):
    self.flushes += 1


class FakeProcess:
  def __init__(self, returncode=None, broken_pipe=False):
    self.returncode = returncode
    self.stdin = FakeStdin(broken_pipe)
    self.killed = False

  def poll(self):
    return self.returncode

  def kill(self):
    self.killed = True
    self.returncode = -9

  def communicate(self, timeout):
    return b"", b""


class PopenFactory:
  def __init__(self, *processes):
    self.processes = list(processes)
    self.calls = []

  def __call__(self, *args, **kwargs):
    self.calls.append((args, kwargs))
    return self.processes.pop(0)


def test_uses_current_python_interpreter(monkeypatch):
  process = FakeProcess()
  popen = PopenFactory(process)
  monkeypatch.setattr(spinner_module.subprocess, "Popen", popen)

  spinner = Spinner()
  assert spinner.update("building")
  spinner.close()

  assert popen.calls[0][0][0] == [sys.executable, "./spinner.py"]
  assert process.stdin.writes == [b"building\n"]


def test_restarts_exited_spinner_and_replays_update(monkeypatch):
  exited_process = FakeProcess(1)
  replacement_process = FakeProcess()
  popen = PopenFactory(exited_process, replacement_process)
  monkeypatch.setattr(spinner_module.subprocess, "Popen", popen)

  spinner = Spinner()
  assert spinner.update("still building")
  spinner.close()

  assert len(popen.calls) == 2
  assert replacement_process.stdin.writes == [b"still building\n"]


def test_restarts_after_broken_pipe(monkeypatch):
  broken_process = FakeProcess(broken_pipe=True)
  replacement_process = FakeProcess()
  popen = PopenFactory(broken_process, replacement_process)
  monkeypatch.setattr(spinner_module.subprocess, "Popen", popen)

  spinner = Spinner()
  assert spinner.update("recover")
  spinner.close()

  assert len(popen.calls) == 2
  assert replacement_process.stdin.writes == [b"recover\n"]
