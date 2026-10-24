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
    self.raw_read_limit = getattr(transport, '_read_limit', None)
    self.prefetch_remaining = 0
    self.prefetch_seq = None
    if self.raw_read_limit is not None:
      transport._read_limit = self._read_limit

  def _read_limit(self, missing):
    # Only bytes promised by the current checked envelope header. Never ask
    # beyond its logical request, where an idle sender would deadlock this read.
    return max(self.raw_read_limit(missing), min(self.prefetch_remaining, self.transport.read_chunk))

  def note_message(self, msg):
    from jetlink import request_chunks as C, protocol as P
    if msg.msg_type == P.Msg.HELLO_REQ:
      self.prefetch_seq, self.prefetch_remaining = None, 0
      return
    if (msg.msg_type != C.MESSAGE or msg.flags or len(msg.payload) < C.HEADER.size or
        (self.prefetch_seq is not None and msg.seq <= self.prefetch_seq)):
      return
    kind, flags, total, offset, checksum = C.HEADER.unpack_from(msg.payload)
    body = len(msg.payload) - C.HEADER.size
    if (kind not in C.KINDS or flags or offset or not 0 < total <= C.MAX_BYTES or
        body != min(C.DATA_BYTES, total)):
      return
    remaining = total - body
    whole, tail = divmod(remaining, C.DATA_BYTES)
    wire = whole * C.WIRE_BYTES
    if tail:
      wire += -(-(P.HEADER_SIZE + C.HEADER.size + tail) // P.GADGET_TX_ALIGN) * P.GADGET_TX_ALIGN
    self.prefetch_seq, self.prefetch_remaining = msg.seq, wire

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
    self.prefetch_remaining = max(0, self.prefetch_remaining - count)
    return count

  def close(self):
    if self.transfer.isSubmitted():
      raise LinkError('USB IN still submitted; refusing buffer/handle teardown')
    self.transfer.close()
    if self.raw_read_limit is not None:
      self.transport._read_limit = self.raw_read_limit
