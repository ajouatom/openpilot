"""Apply USB input-current protection only between sessions, on an empty port."""
import logging
import os
from pathlib import Path
import subprocess
import time

POWER_ROLE = Path('/sys/class/usbpd/usbpd0/current_pr')
HELPER = Path(__file__).resolve().parents[4] / 'tools/jetlink/usb_power.py'
log = logging.getLogger('carrot.jetlink')


def cut_input():
  command = ['python3', str(HELPER)]
  if os.geteuid() != 0:
    command = ['sudo', '-n', *command]
  result = subprocess.run(command, capture_output=True, text=True, timeout=5, check=False)
  if result.returncode == 3:
    return False  # Attachment changed while the helper was being launched.
  if result.returncode != 0 or result.stdout.strip() != 'applied':
    raise RuntimeError((result.stderr or 'USB input-current helper failed')[-500:].strip())
  return True


class UsbInputPower:
  EMPTY_SECONDS = 1.5

  def __init__(self):
    self.empty_since = None
    self.finished = False
    self.noticed = False

  def step(self):
    # Never undo on retry, mode change, ignition transition or daemon exit.
    # The kernel clears the forced vote at reboot. No per-frame subprocesses.
    if self.finished:
      return
    try:
      empty = POWER_ROLE.read_text().strip() == 'none'
    except OSError:
      empty = False  # Missing/unknown policy-engine state never means empty.
    if not empty:
      self.empty_since = None
      if not self.noticed:
        log.info('Jetlink USB power: protection deferred until the port is empty; existing supply unchanged')
        self.noticed = True
      return
    now = time.monotonic()
    if self.empty_since is None:
      self.empty_since = now
    if now - self.empty_since < self.EMPTY_SECONDS:
      return
    try:
      if not cut_input():
        self.empty_since = None
        return
    except (OSError, RuntimeError, subprocess.TimeoutExpired) as exc:
      # Diagnostics must not crash/restart inference; do not storm sudo/sysfs.
      log.warning('Jetlink USB power: could not verify protection; no automatic retry until daemon restart: %s', exc)
    else:
      log.info('Jetlink USB power: zero input-current vote verified; retained until reboot')
    self.finished = True
