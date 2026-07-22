from pathlib import Path

from opendbc import DBC_PATH, get_generated_dbcs
from opendbc.dbc.generator import generator


def get_jotpluggler_generated_dbcs() -> dict[str, str]:
  """Load generated DBCs from either the current or legacy opendbc layout."""
  if hasattr(generator, "generate_all"):
    return get_generated_dbcs()

  # This fork's older opendbc snapshot materializes generated DBCs on disk.
  return {
    path.stem: path.read_text(encoding="utf-8")
    for path in sorted(Path(DBC_PATH).glob("*_generated.dbc"))
  }
