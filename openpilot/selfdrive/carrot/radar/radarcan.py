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
from openpilot.selfdrive.carrot.radar.lateral import set_radar_track_flip
from openpilot.selfdrive.pandad import can_capnp_to_list
from opendbc.car.car_helpers import interfaces
from opendbc.car.hyundai.radar_classic_238 import (
  CLASSIC_238_END_ADDR,
  CLASSIC_238_START_ADDR,
  Classic238DisplayTracker,
  classic_238_display_ids,
)


CLASSIC_DISPLAY_PERIOD_NS = 50_000_000


def _set_bosch_context(radar, sm, now_ns):
  radar.set_bosch_context(
    now_ns,
    sm['livePose'] if sm.valid['livePose'] else None,
    sm.logMonoTime['livePose'],
    sm['modelV2'] if sm.valid['modelV2'] else None,
    sm.logMonoTime['modelV2'],
  )


def _rebuild_radar_interface(radar_interface, CP, previous=None):
  """기본 생성은 그대로 두고 명시한 native 복구 API만 호출한다."""
  radar = radar_interface(CP)
  recovery_api = getattr(radar, 'ARS510_INPUT_RECOVERY_API', None)
  recover = getattr(radar, 'recover_from_input_fault', None)
  if previous is not None and type(recovery_api) is int and recovery_api == 1 and callable(recover):
    recover(previous)
  return radar


def main():
  # Share core4/FIFO51 with planner. Card/radard use core5; short control/state
  # work shares core6 with camera; model/DM remain on core7.
  config_realtime_process(4, Priority.CTRL_LOW)
  poller = messaging.Poller()
  can_sock = messaging.sub_sock('can', poller=poller, conflate=False)
  state_sock = messaging.sub_sock('carState', poller=poller, conflate=False)
  pm = messaging.PubMaster(['liveTracks', 'classicRadarTracks'])
  CP = messaging.log_from_bytes(Params().get('CarParams', block=True), car.CarParams)
  radar_interface = interfaces[CP.carFingerprint].RadarInterface
  radar = _rebuild_radar_interface(radar_interface, CP)
  bosch_context_sm = messaging.SubMaster(['livePose', 'modelV2']) if getattr(radar, 'bosch', None) is not None else None
  # Latch once per onroad start; never change a track's side during a drive.
  radar_track_flip = Params().get_bool('RadarTrackFlip')
  classic_tracker = Classic238DisplayTracker() if CP.brand == 'hyundai' else None
  batches = RadarCanBatches()
  diagnostics = RuntimeDiagnostics('radarcan', cloudlog.event)
  last_input_ns = time.monotonic_ns()
  last_can_input_ns = last_input_ns
  last_error_publish_ns = 0
  needs_reset = False
  replay = 'REPLAY' in os.environ
  replay_ns = last_input_ns
  # K7 0x238 objects are shown on the web in every EnableRadarTracks mode. When the
  # selected radar path publishes nothing (e.g. mode 1 on a car without 0x500 tracks),
  # publish them on their own 20 Hz cadence; display only, never used for control here.
  classic_seen = False
  last_classic_publish_ns = 0
  bosch_replay_ns = 0

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
      # 호스트 오류도 표시 스트림 지원을 유지해 이전 선행차 목록으로 돌아가지 않는다.
      msg.liveTracks.hasDisplayData = getattr(radar, 'DISPLAY_POINTS_SUPPORTED', False) is True
      msg.liveTracks.radarTrackFlipped = radar_track_flip
      pm.send('liveTracks', msg)
      classic_msg = messaging.new_message('classicRadarTracks')
      classic_msg.valid = False
      pm.send('classicRadarTracks', classic_msg)
      last_error_publish_ns = now_ns

  def publish_classic_tracks(now_ns):
    nonlocal last_classic_publish_ns
    last_classic_publish_ns = now_ns
    tracks = classic_tracker.current(now_ns) if classic_tracker is not None else []
    msg = messaging.new_message('classicRadarTracks')
    msg.valid = True
    points = msg.classicRadarTracks.init('points', len(tracks))
    display_ids = classic_238_display_ids(tracks)
    for point, track, display_id in zip(points, tracks, display_ids, strict=True):
      kinematics = track.kinematics
      point.trackId = display_id
      point.dRel = kinematics.d_rel
      point.yRel = kinematics.y_rel
      point.vRel = kinematics.v_rel
      point.aRel = kinematics.a_rel
      point.yvRel = kinematics.yv_rel
      point.measured = True
      point.vLead = kinematics.v_lead
      point.aLead = float('nan')
      point.jLead = float('nan')
      point.radarSource = 'frontRadar'
      point.trackState = track.status
    pm.send('classicRadarTracks', msg)

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
    if bosch_context_sm is not None:
      bosch_context_sm.update(0)
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
        radar = _rebuild_radar_interface(radar_interface, CP, radar)
        if classic_tracker is not None:
          classic_tracker = Classic238DisplayTracker()
        needs_reset = False
      last_input_ns = ego.receive_ns
      max_input_age_ms = max(max_input_age_ms, (now_ns - ego.receive_ns) / 1e6)
      if bosch_context_sm is not None:
        if replay and packets:
          bosch_replay_ns = max(bosch_replay_ns, ego.last_can_ns)
        context_ns = bosch_replay_ns if replay else ego.receive_ns
        _set_bosch_context(radar, bosch_context_sm, context_ns)
      result = radar.update_carrot(ego.v_ego, ego.a_ego, ego.receive_ns * 1e-9, packets)
      for packet_ns, frames in packets:
        for address, dat, src in frames:
          if (classic_tracker is not None and src == 1
              and CLASSIC_238_START_ADDR <= address <= CLASSIC_238_END_ADDR):
            try:
              classic_tracker.update(packet_ns, address, dat, ego.v_ego)
              classic_seen = True
            except ValueError:
              # Foreign or malformed frame in this ID range: display only, never crash.
              classic_tracker = Classic238DisplayTracker()
      if classic_tracker is not None:
        classic_tracker.finish_scan(ego.receive_ns)
      processed += 1
      if now() - ego.receive_ns > MAX_INPUT_AGE_NS:
        publish_error('processingTimeout', now())
        continue
      if result is not None:
        msg = messaging.new_message('liveTracks')
        msg.valid = not any(result.errors.to_dict().values())
        msg.liveTracks = result
        # Assignment copies the result: decoder points/filter history stay raw.
        set_radar_track_flip(msg.liveTracks, radar_track_flip)
        pm.send('liveTracks', msg)
        publish_classic_tracks(ego.receive_ns)
      elif classic_seen and ego.receive_ns - last_classic_publish_ns >= CLASSIC_DISPLAY_PERIOD_NS:
        publish_classic_tracks(ego.receive_ns)
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
