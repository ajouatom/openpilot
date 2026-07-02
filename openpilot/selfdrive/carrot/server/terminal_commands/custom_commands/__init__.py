from __future__ import annotations


_loaded = False


def load_commands() -> None:
  global _loaded
  if _loaded:
    return

  from . import help as _help
  from . import vision_test as _vision_test

  del _help, _vision_test
  _loaded = True
