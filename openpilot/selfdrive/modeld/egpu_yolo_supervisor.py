"""Manager-owned CPU supervisor. Never restarts cameras, modeld, or the GPU."""
import json
import math
import os
import signal
import time

from openpilot.selfdrive.modeld.egpu_yolo_auto import AutomaticRecovery, boot_id, configured, directory


def poll_inputs(sm, next_poll):
  # SubMaster's frequency config describes the caller's sampling rate; it does
  # not throttle poll(). Fast carState/control messages otherwise wake this loop
  # hundreds of times per second and fail the configured 20 Hz upper bound.
  time.sleep(max(0., next_poll-time.monotonic()))
  next_poll = time.monotonic()+.05
  sm.update(0)
  return next_poll


def make_submaster(messaging, required):
  # selfdrived publishes onroadEvents every second AND whenever events change.
  # Its burst rate is not a health signal; retain alive/valid checks instead.
  return messaging.SubMaster(required+['carrotYolo'], frequency=20, ignore_avg_freq=['onroadEvents'])


def main():
  import fcntl
  os.sched_setscheduler(0, os.SCHED_OTHER, os.sched_param(0))
  os.sched_setaffinity(0, {0, 1, 2, 3})
  from openpilot.cereal import messaging
  from openpilot.common.params import Params
  from openpilot.selfdrive.modeld.egpu_yolo import camera_time
  from openpilot.selfdrive.modeld.egpu_yolo_reuse import observation_state_permitted

  out = directory()
  lock = (out/'offroad.lock').open('a')
  # Manual benchmark sessions retain exclusive ownership. Do not unlink their
  # lease or replace their status while waiting for them to finish.
  while True:
    try:
      fcntl.flock(lock, fcntl.LOCK_EX | fcntl.LOCK_NB)
      break
    except BlockingIOError:
      time.sleep(1)
  lease_file = out/'reuse_session.json'
  required = ['carState', 'selfdriveState', 'carControl', 'deviceState', 'managerState', 'modelV2',
              'roadCameraState', 'wideRoadCameraState', 'driverCameraState', 'onroadEvents']
  sm = make_submaster(messaging, required)
  streams = ['modelV2', 'roadCameraState', 'wideRoadCameraState', 'driverCameraState']
  sockets = {name: messaging.sub_sock(name, conflate=False) for name in streams}
  params, policy = Params(), AutomaticRecovery()
  previous, history = {}, []
  owner = None
  owner_since = last_result = 0.
  last_overruns = 0
  next_save = next_lease = 0.
  next_poll = 0.
  was_enabled = False

  def stop(signum, frame):
    raise SystemExit

  signal.signal(signal.SIGTERM, stop)
  signal.signal(signal.SIGINT, stop)
  try:
    while configured():
      next_poll = poll_inputs(sm, next_poll)
      now = camera_time()
      current_owner = next((p.pid for p in sm['managerState'].processes if p.name == 'modeld' and p.running), None)
      if current_owner != owner:
        owner, owner_since = current_owner, now
        previous.clear()
        last_result, last_overruns = now, 0
      started = sm.seen['deviceState'] and sm.alive['deviceState'] and sm['deviceState'].started
      reason = ''
      health_failures = {s: [check for check in ('alive', 'valid', 'freq_ok') if not getattr(sm, check)[s]]
                         for s in required if not sm.all_checks([s])}
      if not sm.all_checks(required):
        reason = 'waiting for fresh vehicle/model/camera data: '+', '.join(health_failures)
      elif not observation_state_permitted(fresh=True, started=started, gear=str(sm['carState'].gearShifter), speed=sm['carState'].vEgo):
        reason = 'waiting for active ignition and valid vehicle state'
      elif not params.get_bool('UsbGpuActive') or params.get_bool('UsbGpuLoading'):
        reason = 'waiting for active eGPU'
      elif any(p.running and p.name in ('qcom_yolod', 'dmonitoringmodeld', 'dmonitoringd') for p in sm['managerState'].processes):
        reason = 'competing perception process active'
      elif {str(e.name) for e in sm['onroadEvents']}.intersection({'cameraMalfunction', 'cameraFrameRate', 'processNotRunning'}):
        reason = 'camera/process health guard'
      for service, sock in sockets.items():
        for _ in range(64):
          event = messaging.recv_one_or_none(sock)
          if event is None:
            break
          if event.logMonoTime/1e9 < owner_since:
            continue
          value = getattr(event, service)
          stamp = event.logMonoTime if service == 'modelV2' else value.timestampSof
          if not event.valid:
            reason = f'{service} invalid'
          if service in previous and not 0 < stamp-previous[service] <= 120_000_000:
            reason = f'{service} publication gap'
          previous[service] = stamp
          if service == 'modelV2' and (not math.isfinite(value.modelExecutionTime) or not 0 < value.modelExecutionTime < .06
                                       or not math.isfinite(value.frameDropPerc) or value.frameDropPerc > 1):
            reason = 'driving execution/drop guard'
        else:
          reason = 'supervisor input backlog'
      fatal = ''
      if sm.updated['carrotYolo'] and sm.logMonoTime['carrotYolo']/1e9 >= owner_since:
        yolo = sm['carrotYolo']
        if yolo.state == 'run':
          last_result = now
        if yolo.state == 'error':
          fatal = 'YOLO execution error; waiting for next normal model owner'
        if yolo.overruns > last_overruns:
          reason = 'YOLO completed inference exceeded its deadline'
          last_overruns = yolo.overruns
      if policy.enabled and now-max(last_result, policy.running_since) > 10:
        reason = 'no admitted YOLO result for 10 seconds'
      enabled = policy.update(now, owner=owner, started=started, healthy=not reason, reason=reason, fatal=fatal)
      if was_enabled and not enabled:
        yolo = sm['carrotYolo']
        history.append({'camera_time': now, 'reason': policy.reason, 'stable_seconds_required': policy.delay,
                        'yolo_state': yolo.state, 'runs': yolo.runs, 'overruns': yolo.overruns,
                        'required_ms': yolo.requiredTime*1000, 'available_ms': yolo.budgetTime*1000})
        del history[:-12]
      # Immediately revoke on faults; renew disabled leases too, so a prepared
      # owner can publish status throughout cooldown without executing GPU work.
      if not enabled and was_enabled:
        lease_file.unlink(missing_ok=True)
      if not owner or not started:
        lease_file.unlink(missing_ok=True)
      if owner and started and (now >= next_lease or enabled != was_enabled):
        lease = {'expires': now+1, 'prepared': True, 'enabled': enabled, 'mode': 'road_observation',
                 'automatic': True, 'boot_id': boot_id(), 'owner_pid': owner,
                 'generation': int(policy.running_since*1e9) if enabled else 0}
        temp = lease_file.with_suffix('.tmp')
        temp.write_text(json.dumps(lease))
        temp.replace(lease_file)
        next_lease = now+.25
      was_enabled = enabled
      if now >= next_save:
        report = {'stage': 'display' if enabled else 'offroad' if not started else 'stopped' if policy.fatal else 'cooldown',
                  'mode': 'road_observation', 'automatic': True, 'coordinator_pid': os.getpid(), 'model_pid': owner,
                  'updated_camera_time': now, 'reason': policy.reason, 'recovery_reason': policy.reason,
                  'retry_count': policy.failures, 'stable_seconds_required': policy.delay, 'recovery_history': history,
                  'health_failures': health_failures,
                  'latest': sm['carrotYolo'].to_dict() if sm.seen['carrotYolo'] else {}}
        temp = out/'live_reuse_status.tmp'
        temp.write_text(json.dumps(report))
        temp.replace(out/'live_reuse_status.json')
        next_save = now+1
  finally:
    lease_file.unlink(missing_ok=True)


if __name__ == '__main__':
  main()
