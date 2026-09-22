#!/usr/bin/env python3
"""CAN-driven radar preprocessing, isolated from card's camera-shared core."""
import os
import time

from openpilot.cereal import car, messaging
from openpilot.common.params import Params
from openpilot.common.realtime import Priority, config_realtime_process
from openpilot.common.runtime_diagnostics import RuntimeDiagnostics
from openpilot.common.swaglog import cloudlog
from openpilot.selfdrive.carrot.radar.can_batch import MAX_INPUT_AGE_NS, RadarCanBatches, RadarEgoSample
from openpilot.selfdrive.pandad import can_capnp_to_list
from opendbc.car.car_helpers import interfaces


def main():
  # Share core4/FIFO51 with planner. Card/radard use core5; short control/state
  # work shares core6 with camera; model/DM remain on core7.
  config_realtime_process(4, Priority.CTRL_LOW)
  poller = messaging.Poller()
  can_sock = messaging.sub_sock('can', poller=poller, conflate=False)
  state_sock = messaging.sub_sock('carState', poller=poller, conflate=False)
  pm = messaging.PubMaster(['liveTracks'])
  CP = messaging.log_from_bytes(Params().get('CarParams', block=True), car.CarParams)
  radar_interface = interfaces[CP.carFingerprint].RadarInterface
  radar = radar_interface(CP)
  batches = RadarCanBatches()
  diagnostics = RuntimeDiagnostics('radarcan', cloudlog.event)
  last_input_ns = time.monotonic_ns()
  last_can_input_ns = last_input_ns
  last_error_publish_ns = 0
  needs_reset = False
  replay = 'REPLAY' in os.environ
  replay_ns = last_input_ns

  def now():
    return replay_ns if replay else time.monotonic_ns()

  def publish_error(reason, now_ns):
    nonlocal needs_reset, last_error_publish_ns
    if not needs_reset:
      cloudlog.error(f'radarcan input invalid: {reason}')
    needs_reset = True
    if now_ns - last_error_publish_ns >= 50_000_000:
      msg = messaging.new_message('liveTracks')
      msg.valid = False
      msg.liveTracks.errors.canError = True
      pm.send('liveTracks', msg)
      last_error_publish_ns = now_ns

  while True:
    poller.poll(20)
    start = time.monotonic()
    cpu_start = time.thread_time()
    raw_can = messaging.drain_sock_raw(can_sock)
    batches.add_can(can_capnp_to_list(raw_can))
    for raw_state in messaging.drain_sock_raw(state_sock):
      event = messaging.log_from_bytes(raw_state)
      ego = RadarEgoSample.from_car_state(event.carState)
      batches.add_state(ego)
      if replay:
        replay_ns = ego.receive_ns
    decode_done = time.monotonic()
    processed = 0
    max_input_age_ms = 0.0
    while (batch := batches.take(now())) is not None:
      ego, packets, error = batch
      now_ns = now()
      if error:
        publish_error(error, now_ns)
        continue
      if packets:
        last_can_input_ns = ego.receive_ns
      elif now_ns - last_can_input_ns > MAX_INPUT_AGE_NS:
        publish_error('canTimeout', now_ns)
        continue
      if needs_reset:
        # Never bridge missing CAN with stale object IDs or filter histories.
        radar = radar_interface(CP)
        needs_reset = False
      last_input_ns = ego.receive_ns
      max_input_age_ms = max(max_input_age_ms, (now_ns - ego.receive_ns) / 1e6)
      result = radar.update_carrot(ego.v_ego, ego.a_ego, ego.receive_ns * 1e-9, packets)
      processed += 1
      if now() - ego.receive_ns > MAX_INPUT_AGE_NS:
        publish_error('processingTimeout', now())
        continue
      if result is not None:
        msg = messaging.new_message('liveTracks')
        msg.valid = not any(result.errors.to_dict().values())
        msg.liveTracks = result
        pm.send('liveTracks', msg)
    now_ns = now()
    if now_ns - min(last_input_ns, last_can_input_ns) > MAX_INPUT_AGE_NS:
      publish_error('inputTimeout', now_ns)
    diagnostics.record(work_ms=(time.monotonic() - start) * 1000,
                       thread_cpu_ms=(time.thread_time() - cpu_start) * 1000,
                       decode_ms=(decode_done - start) * 1000,
                       radar_ms=(time.monotonic() - decode_done) * 1000,
                       input_age_ms=max_input_age_ms, processed_batches=processed,
                       invalid=int(needs_reset), pending_states=len(batches.states),
                       pending_can=len(batches.can))


if __name__ == '__main__':
  main()
