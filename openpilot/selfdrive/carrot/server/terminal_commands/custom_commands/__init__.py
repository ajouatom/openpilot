from __future__ import annotations


_loaded = False


def load_commands() -> None:
  global _loaded
  if _loaded:
    return

  from . import help as _help
  from . import vision_test as _vision_test
  from . import web_intro as _web_intro
  from . import web_lab as _web_lab
  from . import youtube_test as _youtube_test

  del _help, _vision_test, _web_intro, _web_lab, _youtube_test
  _loaded = True
