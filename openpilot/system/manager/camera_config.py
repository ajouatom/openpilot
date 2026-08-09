import os
from collections.abc import MutableMapping


def configure_wide_camera(params, environ: MutableMapping[str, str] | None = None) -> bool:
  """Latch the wide-camera setting before camerad is started."""
  use_wide_camera = bool(params.get("UseWideCamera", return_default=True))
  if not use_wide_camera:
    (os.environ if environ is None else environ)["DISABLE_WIDE_ROAD"] = "1"
  return use_wide_camera
