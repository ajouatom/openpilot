from collections import Counter
import os
import random
import threading
import time

import numpy as np
import pytest

from opendbc.car.can_definitions import CanData
from openpilot.cereal import car
import openpilot.cereal.messaging as messaging
from openpilot.cereal.services import SERVICE_LIST
from openpilot.selfdrive.pandad import can_list_to_can_capnp
from openpilot.selfdrive.pandad.tests.test_pandad_loopback import setup_pandad, send_random_can_messages
from openpilot.selfdrive.test.helpers import with_processes

JUNGLE_SPAM = "JUNGLE_SPAM" in os.environ


@pytest.mark.tici
class TestBoarddSpiNominalRate:
  @classmethod
  def setup_class(cls):
    cls.previous_env = {key: os.environ.get(key) for key in ('STARTED', 'SPI_ERR_PROB', 'BOARDD_LOOPBACK')}
    os.environ['STARTED'] = '1'
    os.environ['SPI_ERR_PROB'] = '0'
    if not JUNGLE_SPAM:
      os.environ['BOARDD_LOOPBACK'] = '1'

  @classmethod
  def teardown_class(cls):
    for key, value in cls.previous_env.items():
      if value is None:
        os.environ.pop(key, None)
      else:
        os.environ[key] = value

  @with_processes(['pandad'])
  def test_nominal_rate_and_checksum(self, subtests):
    setup_pandad()

    sendcan = messaging.pub_sock('sendcan')
    socks = {s: messaging.sub_sock(s, conflate=False, timeout=100) for s in ('can', 'pandaStates', 'peripheralState')}
    time.sleep(2)

    baseline_checksum = None
    for service, sock in socks.items():
      warmup_messages = messaging.drain_sock(sock)
      if service == "pandaStates":
        assert warmup_messages
        baseline = warmup_messages[-1]
        assert baseline.valid
        assert len(baseline.pandaStates) == 1
        assert not baseline.pandaStates[0].heartbeatLost
        assert baseline.pandaStates[0].safetyModel == car.CarParams.SafetyModel.allOutput
        baseline_checksum = baseline.pandaStates[0].spiChecksumErrorCount
    assert baseline_checksum is not None
    assert baseline_checksum != 0xFFFF

    stop_event = threading.Event()
    send_errors = []
    sent_frames = set()
    sent_batch_count = 0
    returned_frames = Counter()

    def sendcan_load():
      nonlocal sent_batch_count
      try:
        sequence = 0
        next_frame_time = time.monotonic()
        while not stop_event.is_set():
          frames = [CanData(0x500 + bus, sequence.to_bytes(4, 'little') + bytes([bus]) * 4, bus) for bus in range(3)]
          sendcan.send(can_list_to_can_capnp(frames, msgtype='sendcan'))
          sent_frames.update((frame.src, frame.address, frame.dat) for frame in frames)
          sent_batch_count += 1
          sequence += 1
          next_frame_time += 0.01
          now = time.monotonic()
          if next_frame_time < now:
            next_frame_time = now
          stop_event.wait(next_frame_time - now)
      except Exception as exc:
        send_errors.append(exc)
        stop_event.set()

    send_thread = threading.Thread(target=sendcan_load, name="spi_nominal_sendcan")

    timestamps = {service: [] for service in socks}
    last_receive_time = dict.fromkeys(socks)
    checksum_counts = [baseline_checksum]

    def collect_loopback(msg):
      for frame in msg.can:
        if 128 <= frame.src < 131:
          returned_frames[(frame.src - 128, frame.address, bytes(frame.dat))] += 1

    test_time = max(10, int(os.getenv("NOMINAL_TEST_TIME", "20")))
    measurement_start = time.monotonic()
    send_thread.start()
    try:
      end_time = measurement_start + test_time
      while time.monotonic() < end_time:
        for service, sock in socks.items():
          for msg in messaging.drain_sock(sock):
            timestamps[service].append(msg.logMonoTime)
            last_receive_time[service] = time.monotonic()
            assert msg.valid
            if service == "can":
              collect_loopback(msg)
            if service == "pandaStates":
              assert len(msg.pandaStates) == 1
              panda_state = msg.pandaStates[0]
              assert not panda_state.heartbeatLost
              assert panda_state.safetyModel == car.CarParams.SafetyModel.allOutput
              checksum_counts.append(panda_state.spiChecksumErrorCount)
        time.sleep(0.01)
    finally:
      stop_event.set()
      measurement_end = time.monotonic()
      send_thread.join(timeout=2)

    assert not send_thread.is_alive()
    assert not send_errors
    assert len(checksum_counts) >= 2
    assert len(set(checksum_counts)) == 1

    # Allow the last in-flight CAN batch to be returned before checking that
    # endpoint 3 was neither dropped nor executed twice.
    return_deadline = time.monotonic() + 0.5
    while time.monotonic() < return_deadline:
      for msg in messaging.drain_sock(socks['can']):
        collect_loopback(msg)
      if sent_frames and all(returned_frames[frame] >= 1 for frame in sent_frames):
        break
      time.sleep(0.01)

    assert sent_frames
    returned_sent_frames = sum(returned_frames[frame] >= 1 for frame in sent_frames)
    assert returned_sent_frames >= len(sent_frames) * 0.995
    assert all(returned_frames[frame] <= 1 for frame in sent_frames)

    measurement_elapsed = measurement_end - measurement_start
    assert sent_batch_count >= measurement_elapsed * 100 * 0.995 - 1
    assert len(sent_frames) == sent_batch_count * 3
    for service, times in timestamps.items():
      with subtests.test(msg="nominal timing check", service=service):
        assert len(times) >= 2
        elapsed = (times[-1] - times[0]) / 1e9
        observed_rate = (len(times) - 1) / elapsed
        expected_rate = SERVICE_LIST[service].frequency
        print(f"{service}: {observed_rate:.3f} Hz (expected {expected_rate:.1f} Hz)")
        assert observed_rate >= expected_rate * 0.995
        assert observed_rate <= expected_rate * 1.01
        assert len(times) >= measurement_elapsed * expected_rate * 0.995 - 1
        assert np.max(np.diff(times) / 1e9) < 3 / expected_rate
        assert last_receive_time[service] is not None
        assert measurement_end - last_receive_time[service] < max(0.1, 2 / expected_rate)


@pytest.mark.tici
class TestBoarddSpi:
  @classmethod
  def setup_class(cls):
    cls.previous_env = {key: os.environ.get(key) for key in ('STARTED', 'SPI_ERR_PROB', 'BOARDD_LOOPBACK')}
    os.environ['STARTED'] = '1'
    os.environ['SPI_ERR_PROB'] = '0.001'
    if not JUNGLE_SPAM:
      os.environ['BOARDD_LOOPBACK'] = '1'

  @classmethod
  def teardown_class(cls):
    for key, value in cls.previous_env.items():
      if value is None:
        os.environ.pop(key, None)
      else:
        os.environ[key] = value

  @with_processes(['pandad'])
  def test_spi_corruption(self, subtests):
    setup_pandad()

    sendcan = messaging.pub_sock('sendcan')
    socks = {s: messaging.sub_sock(s, conflate=False, timeout=100) for s in ('can', 'pandaStates', 'peripheralState')}
    time.sleep(2)
    for s in socks.values():
      messaging.drain_sock_raw(s)

    total_recv_count = 0
    total_sent_count = 0
    sent_msgs = {bus: list() for bus in range(3)}

    st = time.monotonic()
    ts = {s: list() for s in socks.keys()}
    for _ in range(int(os.getenv("TEST_TIME", "20"))):
      # send some CAN messages
      if not JUNGLE_SPAM:
        sent = send_random_can_messages(sendcan, random.randrange(2, 20))
        for k, v in sent.items():
          sent_msgs[k].extend(list(v))
          total_sent_count += len(v)

      for service, sock in socks.items():
        for m in messaging.drain_sock(sock):
          ts[service].append(m.logMonoTime)

          # sanity check for corruption
          assert m.valid or (service == "can")
          if service == "can":
            for msg in m.can:
              if JUNGLE_SPAM:
                # PandaJungle.set_generated_can(True)
                i = msg.address - 0x200
                assert msg.address >= 0x200
                assert msg.src == (i % 3)
                assert msg.dat == b"\xff" * (i % 8)
                total_recv_count += 1
                continue

              if msg.src > 4:
                continue
              key = (msg.address, msg.dat)
              assert key in sent_msgs[msg.src], f"got unexpected msg: {msg.src=} {msg.address=} {msg.dat=}"
              # TODO: enable this
              # sent_msgs[msg.src].remove(key)
              total_recv_count += 1
          elif service == "pandaStates":
            assert len(m.pandaStates) == 1
            ps = m.pandaStates[0]
            assert ps.uptime < 1000
            assert ps.pandaType == "tres"
            assert ps.ignitionLine
            assert not ps.ignitionCan
            assert 4000 < ps.voltage < 14000
          elif service == "peripheralState":
            ps = m.peripheralState
            assert ps.pandaType == "tres"
            assert 4000 < ps.voltage < 14000
            assert 50 < ps.current < 1000
            assert ps.fanSpeedRpm < 10000

      time.sleep(0.5)
    et = time.monotonic() - st

    print("\n======== timing report ========")
    for service, times in ts.items():
      dts = np.diff(times) / 1e6
      print(service.ljust(17), f"{np.mean(dts):7.2f} {np.min(dts):7.2f} {np.max(dts):7.2f}")
      with subtests.test(msg="timing check", service=service):
        edt = 1e3 / SERVICE_LIST[service].frequency
        assert edt * 0.9 < np.mean(dts) < edt * 1.1
        assert np.max(dts) < edt * 8
        assert np.min(dts) < edt
        assert len(dts) >= ((et - 0.5) * SERVICE_LIST[service].frequency * 0.8)

    with subtests.test(msg="CAN traffic"):
      print(f"Sent {total_sent_count} CAN messages, got {total_recv_count} back. {total_recv_count / (total_sent_count + 1e-4):.2%} received")
      assert total_recv_count > 20
