"""Backend selection at process start; Python remains the comparison path."""
import os

motion = nearest_segment = None
if os.environ.get("CARROT_NATIVE_CPU", "1") != "0":
  try:
    from ._motion_native import motion as motion, nearest_segment as nearest_segment
  except ImportError:
    pass

BACKEND = "cython" if motion is not None else "python"
