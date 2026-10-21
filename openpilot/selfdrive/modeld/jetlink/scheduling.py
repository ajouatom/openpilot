"""Optional session-scoped DWC3 completion scheduling; never in frame I/O."""
import logging
import os
from pathlib import Path
import subprocess
import sys

log = logging.getLogger('carrot.jetlink')
HELPER = Path(__file__).resolve().parents[4] / 'tools/jetlink/usb_priority.py'


class UsbCompletionPriority:
  def __init__(self):
    self.process = None

  def start(self, peer, *, bulk_usb):
    if not bulk_usb or peer.get('carrot_host') != 'jetson' or self.process is not None:
      return
    command = [sys.executable, str(HELPER)]
    if os.geteuid() != 0:
      command = ['sudo', '-n', *command]
    try:
      self.process = subprocess.Popen(command, stdin=subprocess.PIPE, start_new_session=True)
    except OSError:
      log.exception('Jetlink USB completion priority unavailable; keeping existing scheduling')

  def close(self):
    if self.process is None:
      return
    process, self.process = self.process, None
    # EOF also occurs on unexpected jetlinkd death. The root helper restores
    # worker policies itself; never SIGKILL it while it owns priority leases.
    process.stdin.close()
    try:
      result = process.wait(timeout=3)
      if result == 2:
        raise RuntimeError('USB completion priority restoration failed')
      if result != 0:
        log.warning('Jetlink USB completion priority helper exited unsuccessfully')
    except subprocess.TimeoutExpired as exc:
      raise RuntimeError('USB completion priority cleanup did not finish') from exc
