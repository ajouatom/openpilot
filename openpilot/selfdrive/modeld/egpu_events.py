"""Low-volume eGPU lifecycle diagnostics for tmux; never the worker protocol."""
import json
import os
import sys
import time


def error_summary(error) -> str:
  lines = str(error).strip().splitlines()
  return (lines[-1] if lines else type(error).__name__)[-600:]


def log_event(event: str, **fields) -> None:
  try:
    print('[eGPU] ' + json.dumps({'event': event, 'pid': os.getpid(), 'monotonic_s': round(time.monotonic(), 3), **fields},
                                ensure_ascii=True), file=sys.stderr, flush=True)
  except Exception:
    pass  # A closed diagnostic stream must not change model/fallback behavior.
