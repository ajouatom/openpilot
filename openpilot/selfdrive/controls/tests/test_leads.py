import time

import openpilot.cereal.messaging as messaging

from opendbc.car.toyota.values import CAR as TOYOTA
from opendbc.car.mock.values import CAR as MOCK
from openpilot.selfdrive.test.process_replay import replay_process_with_name


class TestLeads:
  def test_radar_fault(self):
    # if there's no radar-related can traffic, radard should either not respond or respond with an error
    # this is tightly coupled with underlying car radar_interface implementation, but it's a good sanity check
    # Preserve real 100 Hz spacing; allocation-speed timestamps put hundreds
    # of CAN packets ahead of the replay's synthetic 1 ms card output latency.
    start_ns = time.monotonic_ns()
    cp = messaging.new_message("carParams", logMonoTime=start_ns)
    msgs = [cp.as_reader()]
    for frame in range(1500):
      can = messaging.new_message("can", 1, logMonoTime=start_ns + (frame + 1) * 10_000_000)
      msgs.append(can.as_reader())
    card_out = replay_process_with_name("card", msgs, fingerprint=TOYOTA.TOYOTA_COROLLA_TSS2)
    car_states = [m for m in card_out if m.which() == "carState"]
    assert len(car_states) > 1000
    assert all(m.carState.radarInput.canPacketCount > 0 for m in car_states)
    radar_inputs = sorted(
      [m for m in msgs if m.which() == "can"] + car_states,
      key=lambda m: m.logMonoTime,
    )
    out = replay_process_with_name("radarcan", radar_inputs, fingerprint=TOYOTA.TOYOTA_COROLLA_TSS2)
    states = [m for m in out if m.which() == "liveTracks"]
    failures = [not state.valid for state in states]

    assert len(states) == 0 or all(failures)

  def test_radarcan_replay_processes_final_ego_batch(self):
    start_ns = time.monotonic_ns()
    msgs = []
    for frame in range(1500):
      mono_ns = start_ns + frame * 10_000_000
      can = messaging.new_message("can", 1, logMonoTime=mono_ns)
      cs = messaging.new_message("carState", logMonoTime=mono_ns + 1_000_000)
      cs.carState.radarInput.firstCanMonoTime = mono_ns
      cs.carState.radarInput.lastCanMonoTime = mono_ns
      cs.carState.radarInput.canPacketCount = 1
      cs.carState.radarInput.receiveMonoTime = mono_ns
      msgs.extend([can.as_reader(), cs.as_reader()])
    out = replay_process_with_name("radarcan", msgs, fingerprint=MOCK.MOCK)
    states = [m for m in out if m.which() == "liveTracks"]
    assert len(states) > 100
    assert all(state.valid for state in states)
    # The mock emits every fifth tick, including the final tick. Replay adds
    # its configured 1 ms output latency to the final carState input timestamp.
    assert states[-1].logMonoTime == msgs[-1].logMonoTime + 1_000_000
