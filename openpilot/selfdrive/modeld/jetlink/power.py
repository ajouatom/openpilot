"""Verify unconditional USB input-current protection before opening Jetlink."""
import logging
import os
from pathlib import Path
import subprocess

HELPER = Path(__file__).resolve().parents[4] / 'tools/jetlink/usb_power.py'
log = logging.getLogger('carrot.jetlink')


def cut_input():
  command = ['python3', str(HELPER)]
  if os.geteuid() != 0:
    command = ['sudo', '-n', *command]
  result = subprocess.run(command, capture_output=True, text=True, timeout=5, check=False)
  if result.returncode != 0 or result.stdout.strip() != 'applied':
    raise RuntimeError((result.stderr or 'USB input-current helper failed')[-500:].strip())
  return True


class UsbInputPower:
  def __init__(self):
    self.finished = False
    self.protected = False
    self.error = ''

  def step(self):
    # Never undo on retry, mode change, ignition transition or daemon exit.
    # The launcher reapplies at every boot; also verify here for a manual
    # daemon start. There are no per-frame subprocesses or live role changes.
    if self.finished:
      return
    try:
      if not cut_input():
        raise RuntimeError('USB input-current helper did not confirm protection')
    except (OSError, RuntimeError, subprocess.TimeoutExpired) as exc:
      # Block this optional link, not the manager/local model. Do not storm sudo/sysfs.
      self.error = 'Jetlink USB input protection not verified; independent main power required: ' + str(exc)
      log.warning('Jetlink USB power: could not verify protection; no automatic retry until daemon restart: %s', exc)
    else:
      self.protected = True
      log.info('Jetlink USB power: zero input-current vote verified; retained until reboot')
    self.finished = True
