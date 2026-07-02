"""Local IP monitor.

Refreshes the device's primary local IPv4 every 5 seconds in a daemon thread.
Used by spinner/TextWindow UIs to show the recovery server target on screen.
"""
import socket
import threading
import time


_state = {"ip": None, "started": False}
_lock = threading.Lock()


def _query_ip():
  try:
    with socket.socket(socket.AF_INET, socket.SOCK_DGRAM) as s:
      s.settimeout(0.3)
      # connect() on a UDP socket does not transmit; it just configures
      # the routing-table source address that getsockname() returns.
      s.connect(("8.8.8.8", 80))
      return s.getsockname()[0]
  except Exception:
    return None


def _refresh_loop():
  fail_count = 0
  max_grace = 3  # keep previous IP for up to 3 failures (~15s)
  while True:
    try:
      ip = _query_ip()
      if ip is not None:
        fail_count = 0
        with _lock:
          _state["ip"] = ip
      else:
        fail_count += 1
        if fail_count >= max_grace:
          with _lock:
            _state["ip"] = None
    except Exception:
      fail_count += 1
      if fail_count >= max_grace:
        with _lock:
          _state["ip"] = None
    time.sleep(5)


def start_ip_monitor():
  """Start the background refresh thread. Idempotent."""
  with _lock:
    if _state["started"]:
      return
    _state["started"] = True
  initial = _query_ip()
  with _lock:
    _state["ip"] = initial
  threading.Thread(target=_refresh_loop, daemon=True, name="ip_monitor").start()


def current_ip():
  with _lock:
    return _state["ip"]


def label_with_port(port):
  ip = current_ip()
  return f"{ip}:{port}" if ip else "no network"
