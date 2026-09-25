import os
import socket
import sys
import threading
import time

import pytest

from openpilot.selfdrive.modeld.jetlink.phase import Gate, Publisher, PACKET, FRESH_NS, WAIT_SECONDS


def test_phase_rejects_stale_future_malformed_and_reordered_releases():
  gate = Gate.__new__(Gate)
  gate.source_sof = gate.sent_at = 0
  now = 1_000_000_000
  for packet in (b'bad', PACKET.pack(0, now), PACKET.pack(now, now + 1),
                 PACKET.pack(now + 1, now), PACKET.pack(1, now - FRESH_NS)):
    gate.accept(packet, now)
    assert gate.sent_at == 0
  gate.accept(PACKET.pack(now - 50_000_000, now - 1), now)
  accepted = gate.source_sof, gate.sent_at
  gate.accept(PACKET.pack(now - 100_000_000, now - 2), now)
  assert (gate.source_sof, gate.sent_at) == accepted
  gate.accept(PACKET.pack(now - 50_000_000, now - 1) + b'x', now)
  assert (gate.source_sof, gate.sent_at) == accepted


@pytest.fixture
def phase_pair():
  if sys.platform != 'linux' or not hasattr(socket, 'AF_UNIX'):
    pytest.skip('Linux abstract datagram sockets required')
  address = f'\0carrot-phase-test-{os.getpid()}-{time.monotonic_ns()}'
  gate, publisher = Gate(address), Publisher(address)
  assert gate.sock is not None
  try:
    yield gate, publisher
  finally:
    gate.close()
    publisher.close()


def test_no_publisher_keeps_dm_independent(phase_pair):
  gate, _ = phase_pair
  start = time.monotonic()
  gate.wait(time.monotonic_ns())
  assert time.monotonic() - start < WAIT_SECONDS / 2


def test_current_camera_upload_releases_dm(phase_pair):
  gate, publisher = phase_pair
  current = time.monotonic_ns() - 45_000_000
  publisher.sent(current - 50_000_000)
  def complete():
    time.sleep(.005)
    publisher.sent(current)
  worker = threading.Thread(target=complete)
  worker.start()
  try:
    gate.wait(current)
    assert gate.source_sof == current
  finally:
    worker.join()


def test_previous_frame_or_lost_upload_cannot_hold_dm_indefinitely(phase_pair):
  gate, publisher = phase_pair
  current = time.monotonic_ns() - 45_000_000
  publisher.sent(current - 50_000_000)
  start = time.monotonic()
  gate.wait(current)
  elapsed = time.monotonic() - start
  assert WAIT_SECONDS * .8 <= elapsed < .1
  assert gate.source_sof < current - 5_000_000


def test_absent_dm_does_not_block_publisher(phase_pair):
  gate, publisher = phase_pair
  gate.close()
  for _ in range(20):
    publisher.sent(time.monotonic_ns() - 40_000_000)


def test_gate_bind_failure_does_not_disable_dm(phase_pair):
  gate, publisher = phase_pair
  duplicate = Gate(publisher.address)
  assert duplicate.sock is None
  duplicate.wait(time.monotonic_ns())
  assert gate.sock is not None
