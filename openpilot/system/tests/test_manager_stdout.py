"""Exercise the real PTY relay without importing device-only dependencies."""
import ast
import os
from pathlib import Path
import select
import subprocess
import sys
import unittest


@unittest.skipUnless(hasattr(os, 'forkpty'), 'requires a POSIX PTY')
class TestManagerStdout(unittest.TestCase):
  def test_plain_print_reaches_pipe_before_child_exit(self):
    source = (Path(__file__).resolve().parents[1] / 'manager/helpers.py').read_text()
    function = next(node for node in ast.parse(source).body
                    if isinstance(node, ast.FunctionDef) and node.name == 'unblock_stdout')
    code = ('import errno, fcntl, os, signal, sys, time\n'
            + ast.get_source_segment(source, function)
            + '\nunblock_stdout()\nprint("MANAGER_STDOUT_PROBE")\ntime.sleep(3)\n')
    env = dict(os.environ)
    env.pop('PYTHONUNBUFFERED', None)
    with subprocess.Popen([sys.executable, '-c', code], stdout=subprocess.PIPE,
                          stderr=subprocess.PIPE, env=env) as process:
      ready = select.select([process.stdout], [], [], 2)[0]
      early = os.read(process.stdout.fileno(), 4096) if ready else b''
      remaining, errors = process.communicate(timeout=5)
    self.assertIn(b'MANAGER_STDOUT_PROBE', early)
    self.assertEqual(remaining, b'')
    self.assertEqual(errors, b'')
    self.assertEqual(process.returncode, 0)


if __name__ == '__main__':
  unittest.main()
