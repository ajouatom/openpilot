#!/usr/bin/env python3
import time

from openpilot.cereal import car
from openpilot.common.params import Params
from openpilot.common.realtime import Priority, config_realtime_process
from openpilot.common.swaglog import cloudlog
from openpilot.selfdrive.controls.lib.ldw import LaneDepartureWarning
from openpilot.selfdrive.controls.lib.longitudinal_planner import LongitudinalPlanner
from openpilot.selfdrive.controls.lib.longitudinal_fast_radar import (
  FastRadarOverlay,
  RadarStateOverride,
)
from openpilot.selfdrive.controls.lib.lateral_planner import LateralPlanner
import openpilot.cereal.messaging as messaging
from openpilot.selfdrive.carrot.carrot_functions import CarrotPlanner
from openpilot.selfdrive.carrot.radar import effective_radar_track_mode


LIVE_TRACKS_FALLBACK_TIMEOUT_S = 0.10
MIN_LONGITUDINAL_PLAN_INTERVAL_NS = 25_000_000


def main():
  config_realtime_process(7, Priority.CTRL_LOW)

  cloudlog.info("plannerd is waiting for CarParams")
  params = Params()
  CP = messaging.log_from_bytes(params.get("CarParams", block=True), car.CarParams)
  cloudlog.info("plannerd got CarParams: %s", CP.brand)

  # The fast path requires stable physical radar identities. Radar source
  # options are Hyundai-only; other radar-equipped brands always use front
  # tracks, while radarless cars preserve the model-triggered planner.
  radar_track_mode = effective_radar_track_mode(
    CP.brand,
    CP.radarUnavailable,
    params.get_int("EnableRadarTracks"),
  )
  live_tracks_longitudinal = radar_track_mode >= 1

  ldw = LaneDepartureWarning()
  longitudinal_planner = LongitudinalPlanner(CP)
  lateral_planner = LateralPlanner(CP, debug=False)
  fast_radar = FastRadarOverlay(
    front_radar_delay_s=float(CP.radarDelay),
  )

  pm = messaging.PubMaster(['longitudinalPlan', 'driverAssistance', 'lateralPlan'])
  # One process owns both planners to avoid another ~100 MB Python runtime.
  # The two 20 Hz inputs wake this serial loop independently; no MPC overlaps.
  sm = messaging.SubMaster(
    ['carControl', 'carState', 'controlsState', 'liveParameters', 'radarState',
     'liveTracks', 'modelV2', 'selfdriveState', 'carrotMan'],
    poll=(['modelV2', 'liveTracks'] if live_tracks_longitudinal else 'modelV2'),
    ignore_avg_freq=['radarState'],
  )
  carrot = CarrotPlanner()
  model_frame = 0
  last_longitudinal_trigger_mono_ns = 0

  while True:
    sm.update()

    if sm.updated['radarState']:
      fast_radar.observe_radar_state(
        sm['radarState'],
        sm.logMonoTime['radarState'],
        sm.valid['radarState'] and sm.alive['radarState'],
      )

    # For a stably selected physical lead in normal radar ACC, liveTracks is
    # the longitudinal clock. Vision-only, experimental, Carrot blended, and
    # radarless/SCC-only operation retain the newest modelV2 clock.
    live_tracks_recent = (
      sm.seen['liveTracks']
      and time.monotonic() - sm.recv_time['liveTracks'] <= LIVE_TRACKS_FALLBACK_TIMEOUT_S
    )
    use_live_tracks_trigger = (
      live_tracks_longitudinal
      and not sm['selfdriveState'].experimentalMode
      and getattr(carrot, 'mode', 'acc') == 'acc'
      and live_tracks_recent
      and sm.valid['radarState']
      and sm.alive['radarState']
      and fast_radar.lead_one_ready(sm['radarState'])
    )
    planning_trigger = 'liveTracks' if use_live_tracks_trigger else 'modelV2'
    trigger_mono_ns = sm.logMonoTime[planning_trigger]
    trigger_interval_ok = (
      last_longitudinal_trigger_mono_ns == 0
      or trigger_mono_ns - last_longitudinal_trigger_mono_ns >= MIN_LONGITUDINAL_PLAN_INTERVAL_NS
    )
    run_longitudinal = sm.updated[planning_trigger] and trigger_interval_ok

    if run_longitudinal and sm.seen['modelV2']:
      # Bound unexpected replay/recovery bursts without filtering either
      # service's normal 20 Hz cadence.
      last_longitudinal_trigger_mono_ns = trigger_mono_ns
      planner_start = time.monotonic()

      if use_live_tracks_trigger:
        fast_radar_start = time.monotonic()
        fast_result = fast_radar.build(
          sm['radarState'],
          sm['liveTracks'],
          sm['carState'].vEgo,
          sm.logMonoTime['radarState'],
          sm.logMonoTime['liveTracks'],
          radar_state_valid=sm.valid['radarState'] and sm.alive['radarState'],
          live_tracks_valid=sm.valid['liveTracks'] and sm.alive['liveTracks'],
        )
        fast_radar_execution_time = time.monotonic() - fast_radar_start
        planner_sm = RadarStateOverride(sm, fast_result.radar_state)
      else:
        fast_result = None
        fast_radar_execution_time = 0.0
        planner_sm = sm

      longitudinal_planner.update(planner_sm, carrot)
      planner_execution_time = time.monotonic() - planner_start
      longitudinal_planner.publish(
        planner_sm,
        pm,
        carrot,
        planner_execution_time=planner_execution_time,
        live_tracks_mono_time=(sm.logMonoTime['liveTracks'] if sm.seen['liveTracks'] else 0),
        fast_lead_mask=(fast_result.lead_mask if fast_result is not None else 0),
        fast_lead_track_id=(fast_result.lead_one_track_id if fast_result is not None else -1),
        planning_trigger=planning_trigger,
        fast_radar_execution_time=fast_radar_execution_time,
        fast_lead_reason=(fast_result.lead_one_reason if fast_result is not None else 'inactive'),
      )

    if sm.updated['modelV2']:
      model_frame += 1
      lateral_planner.update(sm, carrot)
      lateral_planner.publish(sm, pm, carrot)

      ldw.update(model_frame, sm['modelV2'], sm['carState'], sm['carControl'])
      msg = messaging.new_message('driverAssistance')
      msg.valid = sm.all_checks(['carState', 'carControl', 'modelV2', 'liveParameters'])
      msg.driverAssistance.leftLaneDeparture = ldw.left
      msg.driverAssistance.rightLaneDeparture = ldw.right
      pm.send('driverAssistance', msg)


if __name__ == "__main__":
  main()
