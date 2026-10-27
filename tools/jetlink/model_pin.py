"""Standalone host tools also use the client's Jetson model pin."""
from pathlib import Path
import sys

sys.path.insert(0, str(Path(__file__).resolve().parents[2]))
from openpilot.common.jetson_model import read_spec  # noqa: F401
