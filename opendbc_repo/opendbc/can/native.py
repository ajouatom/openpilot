"""Optional native CAN loops, selected once when the process starts."""
import os

pack = raw_values = None
if os.environ.get("CARROT_NATIVE_CPU", "1") != "0":
  try:
    from ._can_native import pack as pack, raw_values as raw_values
  except ImportError:
    pass

BACKEND = "cython" if pack is not None else "python"
