"""Carrot adapter for the pinned, interoperable Jetlink wire protocol."""
from pathlib import Path
import sys

VENDOR = Path(__file__).resolve().parents[4] / 'third_party' / 'jetlink'
if str(VENDOR) not in sys.path:
  sys.path.insert(0, str(VENDOR))
