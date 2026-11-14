import os
from pathlib import Path
import shutil
import subprocess

import pytest


def test_native_tap_feedback(tmp_path):
  compiler = shutil.which('g++') or shutil.which('clang++')
  if compiler is None and os.name == 'nt':
    pytest.skip('Run the MSVC native harness separately on Windows')
  assert compiler is not None
  source = Path(__file__).with_suffix('.cpp')
  binary = tmp_path / 'tap-feedback'
  subprocess.run([compiler, '-std=c++11', '-Wall', '-Wextra', '-Werror', str(source), '-o', str(binary)], check=True)
  subprocess.run([str(binary)], check=True)
