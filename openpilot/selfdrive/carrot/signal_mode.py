"""Single user selection for signal assistance; legacy enable files cannot override it."""
from pathlib import Path
import time

DIRECTORY = Path('/data/signal-color-shadow')
PARAM = 'SignalAssistMode'
_params = None
_mode = 0
_next_read = 0.


def read_mode(params):
  try:
    value = params.get_int(PARAM)
    return value if value in (0, 1, 2) else 0
  except Exception:
    # Includes native UnknownKeyName before Params has been rebuilt after pull.
    # A missing/unreadable setting must never enable control.
    return 0


def signal_mode():
  global _params, _mode, _next_read
  now = time.monotonic()
  if now >= _next_read:
    if _params is None:
      from openpilot.common.params import Params
      _params = Params()
    _mode = read_mode(_params)
    _next_read = now + .5
  return _mode


def flag_override(path):
  """Production enable policy. Other paths retain isolated offline-test flags."""
  path = Path(path)
  if path.parent != DIRECTORY:
    return None
  if path.name == 'assist_enabled':
    return False  # retired moving-red override is not one of the new modes
  if path.name in ('enabled', 'tracking_enabled', 'display_enabled', 'start_guard_enabled', 'stop_prepare_enabled'):
    return signal_mode() > 0
  return None
