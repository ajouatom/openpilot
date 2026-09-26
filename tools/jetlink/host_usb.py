"""Keep a USB IN request pending while idle; cancel only when closing the link."""
import threading

from jetlink.transport.base import LinkError


class BulkInput:
  def __init__(self, transport, stopped):
    import usb1
    self.usb = usb1
    self.transport = transport
    self.stopped = stopped
    self.transfer = transport.handle.getTransfer()
    self.completed = threading.Event()

  def read(self, dest, timeout):
    # The consumer's queue and C4 model IPC enforce their own deadlines. An
    # idle USB poll must not cancel/repost the physical transfer every 100 ms.
    if self.stopped.is_set():
      raise LinkError('USB reader closing')
    n = self.transport._clamp_read(dest)
    if not n:
      return 0
    self.completed.clear()
    self.transfer.setBulk(self.transport.ep_in, dest[:n],
                          callback=lambda transfer: self.completed.set(), timeout=0)
    self.transfer.submit()
    cancelled = False
    while not self.completed.is_set():
      if self.stopped.is_set() and not cancelled:
        try:
          self.transfer.cancel()
        except self.usb.USBErrorNotFound:
          pass  # Completion may have won the race with close().
        cancelled = True
      try:
        self.transport.context.handleEventsTimeout(tv=.05)
      except self.usb.USBErrorInterrupted:
        continue
    status = self.transfer.getStatus()
    if self.stopped.is_set():
      raise LinkError('USB reader closed')
    if status != self.usb.TRANSFER_COMPLETED:
      raise LinkError(f'USB IN failed: transfer status={status}')
    count = self.transfer.getActualLength()
    if not 0 <= count <= n:
      raise LinkError(f'USB IN returned invalid length {count}/{n}')
    return count

  def close(self):
    if self.transfer.isSubmitted():
      raise LinkError('USB IN still submitted; refusing buffer/handle teardown')
    self.transfer.close()
