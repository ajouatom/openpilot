import os
import shutil
import subprocess
from pathlib import Path

import pytest


PANDA_ROOT = Path(__file__).resolve().parents[2]
SOURCE = Path(__file__).with_suffix(".c")


def test_spi_v3_wire_parser_replay_and_completion(tmp_path: Path):
  cc = os.environ.get("CC", "gcc")
  if shutil.which(cc) is None:
    pytest.skip(f"native C compiler {cc!r} is unavailable")

  executable = tmp_path / "test_spi_v3"
  subprocess.run([
    cc,
    "-std=gnu11",
    "-Wall",
    "-Wextra",
    "-Werror",
    f"-I{PANDA_ROOT}",
    str(SOURCE),
    "-o",
    str(executable),
  ], check=True)
  result = subprocess.run([str(executable)], check=True, capture_output=True, text=True)
  assert result.stdout.strip() == "SPI v3 parser tests passed"
