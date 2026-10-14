"""Isolated real-process test of production supervision with a harmless worker.

Only device imports/logging/Cap'n Proto allocation are substituted. Start, crash
cleanup, restart, duplicate-start prevention and stop use the production classes.
The outer pytest process bounds the entire experiment if a launch ever hangs.
"""
import ast
import json
import multiprocessing
import os
from pathlib import Path
import signal
import sys
import threading
import time
from types import SimpleNamespace


PARENT_ONLY = False


def worker(directory, name):
  if os.name == 'posix':
    import resource
    resource.setrlimit(resource.RLIMIT_CORE, (0, 0))
  signal.signal(signal.SIGINT, lambda *_: sys.exit(0))
  Path(directory, f'{os.getpid()}.json').write_text(json.dumps(PARENT_ONLY))
  while True:
    time.sleep(0.01)


def main(directory, legacy_fork):
  global PARENT_ONLY
  PARENT_ONLY = True
  path = Path(__file__).parents[1] / 'process.py'
  tree = ast.parse(path.read_text(encoding='utf-8'))
  names = {'ManagerProcess', 'PythonProcess', 'ensure_running', 'join_process'}
  definitions = [n for n in tree.body if isinstance(n, (ast.ClassDef, ast.FunctionDef)) and n.name in names]
  reports = []
  namespace = {'__name__': __name__, 'launcher': worker, 'Params': object,
               'report_process_exit': lambda *args: reports.append(args), 'flush_crash_reports': lambda: None,
               'cloudlog': SimpleNamespace(info=lambda *_: None, error=lambda *_: None),
               'car': SimpleNamespace(CarParams=object),
               'log': SimpleNamespace(ManagerState=SimpleNamespace(ProcessState=SimpleNamespace(new_message=SimpleNamespace)))}
  exec('import os, signal, time\nfrom abc import ABC, abstractmethod\n'
       + 'from collections.abc import Callable, ValuesView\nfrom multiprocessing import Process, get_context', namespace)
  exec(compile(ast.Module(body=definitions, type_ignores=[]), str(path), 'exec'), namespace)
  config = ast.parse(path.with_name('process_config.py').read_text(encoding='utf-8'))
  call = next(n for n in ast.walk(config) if isinstance(n, ast.Call) and isinstance(n.func, ast.Name)
              and n.func.id == 'PythonProcess' and n.args[0].value == 'carrot_man')
  namespace['always_run'] = lambda *_: True
  process = eval(compile(ast.Expression(call), 'process_config.py', 'eval'), namespace)
  process.module = directory
  if legacy_fork:
    process.process_factory = multiprocessing.get_context('fork').Process
  if os.name == 'nt':
    # Windows cannot deliver POSIX SIGINT to a standalone child PID.
    process.signal = lambda _: process.proc.terminate()

  forks = []
  if hasattr(os, 'register_at_fork'):
    os.register_at_fork(before=lambda: forks.append(time.monotonic()))
  stopped = threading.Event()
  threads = [threading.Thread(target=stopped.wait) for _ in range(3)]
  for thread in threads:
    thread.start()
  calls = []
  children = []

  def supervise(not_run=None):
    start = time.monotonic()
    namespace['ensure_running']([process], True, not_run=not_run)
    calls.append(time.monotonic() - start)

  def ready():
    child = process.proc
    children.append(child)
    deadline = time.monotonic() + 5
    marker = Path(directory, f'{child.pid}.json')
    while not marker.exists() and time.monotonic() < deadline:
      supervise()
      assert process.proc is child, 'duplicate child while launch is pending'
      time.sleep(0.01)
    assert marker.exists(), 'child did not initialize'
    state = process.get_process_state_msg()
    assert state.running and state.shouldBeRunning and state.pid == child.pid
    return child

  try:
    supervise()
    first = ready()
    if os.name == 'posix':
      os.kill(first.pid, signal.SIGABRT)
    else:
      first.terminate()
    first.join(3)
    assert first.exitcode is not None and first.exitcode != 0
    state = process.get_process_state_msg()
    assert not state.running and state.shouldBeRunning
    supervise()
    second = ready()
    assert second.pid != first.pid
    supervise(not_run=['carrot_man'])
    second.join(3)
    supervise(not_run=['carrot_man'])
    assert process.proc is None
    assert reports == [('carrot_man', first.pid, first.exitcode)], reports
    result = {'pids': [first.pid, second.pid], 'crash_exitcode': first.exitcode,
              'max_supervision_s': max(calls), 'fork_callbacks': len(forks),
              'inherited_parent_state': [json.loads(Path(directory, f'{p.pid}.json').read_text()) for p in children],
              'stopped': process.proc is None}
    print(json.dumps(result))
  finally:
    for child in children + ([process.proc] if process.proc is not None else []):
      if child.is_alive():
        child.terminate()
      child.join(3)
    stopped.set()
    for thread in threads:
      thread.join(1)


if __name__ == '__main__':
  main(sys.argv[1], bool(int(sys.argv[2])))
