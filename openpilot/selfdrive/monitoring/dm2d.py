"""20 Hz DM dispatcher: stock camera criteria or automatic interaction fallback."""
import math
import os
import time

from openpilot.cereal import car
import openpilot.cereal.messaging as messaging
from openpilot.cereal.services import SERVICE_LIST
from openpilot.common.params import Params
from openpilot.common.realtime import DT_DMON, Ratekeeper, config_realtime_process
from openpilot.selfdrive.monitoring.config import experimental_mode
from openpilot.selfdrive.carrot.bluetooth.model import CommandReader
from openpilot.selfdrive.monitoring.dm2 import DriverMonitoring2
from openpilot.selfdrive.monitoring.dm2_cadence import DmRatekeeper
from openpilot.selfdrive.monitoring.dm2_context import (AutomaticCancelFilter, CameraAvailability, CancelPressSequence,
                                                       InteractionEdges, ObjectObservation, SteeringTouchEvidence, TrafficContext)


INPUT_FRESHNESS_SECONDS = 0.25
SESSION_DISABLED_PARAM = "DriverMonitoringSessionDisabled"


def traffic_observations(radar):
  leads = [radar.leadOne, radar.leadTwo, radar.leadLeft, radar.leadRight]
  for name in ("leadsCenter", "leadsLeft", "leadsRight", "leadsLeft2", "leadsRight2", "leadsCutIn"):
    leads.extend(getattr(radar, name))
  return [ObjectObservation(p.dRel, p.dPath, p.vLead, p.vRel) for p in leads if p.status]


def side_coverage(params, cp):
  # RadarInterface includes corner parser health in radarErrors on these platforms.
  from opendbc.car.hyundai.values import HyundaiExtFlags
  flags = int(HyundaiExtFlags.CORNER_RADAR_OBJECTS_235 | HyundaiExtFlags.CORNER_RADAR_OBJECTS_180 |
              HyundaiExtFlags.CORNER_RADAR_OBJECTS_430)
  return cp.brand == "hyundai" and params.get_int("EnableCornerRadar") > 0 and bool(int(cp.extFlags) & flags)


def replay_services_fresh(sm, services, now):
  return all(0 <= now - sm.logMonoTime.get(service, 0) / 1e9 < 10. / SERVICE_LIST[service].frequency
             for service in services)


def camera_sample_usable(sm, rhd, demo=False, replay_now=None):
  required_services = ['driverStateV2'] if demo else ['driverStateV2', 'liveCalibration', 'modelV2']
  if not sm.all_checks(required_services):
    return False
  if replay_now is not None and not replay_services_fresh(sm, required_services, replay_now):
    return False
  driver = sm['driverStateV2'].rightDriverData if rhd else sm['driverStateV2'].leftDriverData
  probabilities = (driver.faceProb, driver.leftEyeProb, driver.rightEyeProb, driver.leftBlinkProb, driver.rightBlinkProb,
                   driver.sleepProb, driver.phoneProb, driver.sunglassesProb)
  if not all(math.isfinite(v) and 0 <= v <= 1 for v in probabilities):
    return False
  vectors = (driver.faceOrientation, driver.facePosition, driver.faceOrientationStd, driver.facePositionStd)
  lengths = (2, 2, 2, 2)
  if not demo:
    vectors += (sm['liveCalibration'].rpyCalib, sm['modelV2'].meta.disengagePredictions.brakeDisengageProbs)
    lengths += (3, 1)
  return all(len(values) >= length and all(math.isfinite(v) for v in values)
             for values, length in zip(vectors, lengths, strict=True))


def parked_reset_eligible(sm, now, demo=False):
  if demo or not sm.all_checks(['carState', 'selfdriveState']):
    return False
  if not all(0 <= now - sm.logMonoTime.get(service, 0) / 1e9 < 0.25 for service in ('carState', 'selfdriveState')):
    return False
  cs, state = sm['carState'], sm['selfdriveState']
  # Raw speed must report zero. Allow only a tiny settling residue in the speed
  # filter, in addition to independent P/standstill/disengaged confirmations.
  return (cs.canValid and cs.gearShifter == car.CarState.GearShifter.park and cs.standstill and
          cs.vEgoRaw == 0 and math.isfinite(cs.vEgo) and abs(cs.vEgo) < 0.01 and
          not state.enabled and not state.active)


def new_driver_monitor(params, experimental):
  return DriverMonitoring2(rhd_saved=params.get_bool("IsRhdDetected"), always_on=params.get_bool("AlwaysOnDM"),
                           experimental=experimental)


def disabled_state_packet(rhd, experimental, monitor=None, valid=True, camera_unavailable=False):
  packet = monitor.get_state_packet(valid=valid) if monitor is not None else messaging.new_message('driverMonitoringState', valid=valid)
  state = packet.driverMonitoringState
  state.lockout = False
  state.alertCountLockoutPercent = 0
  state.alertTimeLockoutPercent = 0
  state.lockoutRecoveryPercent = 0
  state.alert3Count = 0
  state.noResponseCount = 0
  state.noResponseForceDecel = False
  state.alwaysOn = False
  state.alwaysOnLockout = False
  state.alertLevel = 'none'
  state.cameraUnavailable = camera_unavailable
  state.dm2Disabled = True
  state.dm2Experimental = experimental
  state.dm2StrictTimeRemaining = 0
  state.dm2WheelTimeoutFactor = 1
  state.dm2ForwardAttentionScore = 0
  state.dm2ForwardRecovery = False
  state.dm2InteractionCredit = 0
  state.dm2VisionTimeoutFactor = 1
  state.dm2InteractionGraceRemaining = 0
  if monitor is None:
    state.activePolicy = 'wheeltouch'
    state.isRHD = rhd
  state.visionPolicyState.awarenessPercent = 100
  state.visionPolicyState.awarenessStep = 0
  state.visionPolicyState.isDistracted = False
  state.visionPolicyState.distractedTypes.pose = False
  state.visionPolicyState.distractedTypes.eye = False
  state.visionPolicyState.distractedTypes.phone = False
  state.visionPolicyState.distractedTypes.sleep = False
  state.visionPolicyState.uncertainOffroadAlertPercent = 0
  state.wheeltouchPolicyState.awarenessPercent = 100
  state.wheeltouchPolicyState.awarenessStep = 0
  state.wheeltouchPolicyState.driverInteracting = False
  return packet


def dm_clock(sm, replay):
  # Replay preserves the route's monotonic timestamps but executes against the
  # host clock. Use the newest received route input so Driver View and model
  # gaps also keep all age, gesture and context calculations on one clock.
  if replay:
    route_time = max(sm.logMonoTime.get(service, 0)
                     for service in ('driverStateV2', 'modelV2', 'carState', 'selfdriveState')) / 1e9
    if math.isfinite(route_time) and route_time > 0:
      return route_time
  return time.monotonic()


def input_packet_fresh(now, event_time):
  return math.isfinite(event_time) and 0 <= now - event_time < INPUT_FRESHNESS_SECONDS


def cancel_echo_filter_required(cp):
  # Hyundai openpilot-long never sends cruise-button CANCEL from its long-control
  # branches. controlsd still keeps cruiseControl.cancel high while PCM cruise
  # reports enabled, so correlating that level would hide real wheel presses.
  return cp is None or cp.brand != "hyundai" or not cp.openpilotLongitudinalControl


def record_automatic_cancel_requests(sock, automatic_cancel, now, replay, filter_required):
  for packet in messaging.drain_sock(sock, wait_for_one=False):
    event_time = packet.logMonoTime / 1e9
    packet_now = now if replay else time.monotonic()
    # card actuates every alive carControl packet, including one whose validity
    # bit is false, so mirror that behavior for echo suppression.
    if filter_required and input_packet_fresh(packet_now, event_time):
      automatic_cancel.record(event_time, packet.carControl.cruiseControl.cancel)


def run_dm2(params, experimental, initial_car_params=None):
  # Read after process launch: process replay sets the environment before this
  # worker starts, while Python's module cache can predate that environment.
  replay = "REPLAY" in os.environ
  services = ['carState', 'selfdriveState', 'modelV2', 'radarState', 'liveCalibration', 'carParams', 'driverStateV2']
  # Preserve driver-camera cadence on device. Replay uses modelV2 because a
  # disabled route intentionally has no driverStateV2 messages.
  sm = messaging.SubMaster(services, poll='modelV2' if replay else 'driverStateV2')
  pm = messaging.PubMaster(['driverMonitoringState'])
  # carState is 100 Hz; conflating it to 20 Hz can lose a complete button press.
  input_sock = messaging.sub_sock('carState', conflate=False)
  control_sock = messaging.sub_sock('carControl', conflate=False)
  dm = new_driver_monitor(params, experimental)
  traffic, inputs = TrafficContext(), InteractionEdges()
  cancel_sequence = CancelPressSequence()
  automatic_cancel = AutomaticCancelFilter()
  filter_automatic_cancel = cancel_echo_filter_required(initial_car_params)
  bluetooth = CommandReader('attention')
  camera_health = CameraAvailability()
  touch_evidence = SteeringTouchEvidence()
  rk = Ratekeeper(int(1 / DT_DMON), print_delay_threshold=None) if replay else DmRatekeeper(DT_DMON)
  strict, clear = True, False
  covered = False
  allow_speed_buttons = False
  demo_mode = params.get_bool("IsDriverViewEnabled")
  rhd_saved = params.get_bool("IsRhdDetected")
  monitoring_enabled = params.get_bool("DriverMonitoringEnabled") and not params.get_bool(SESSION_DISABLED_PARAM)
  disable_write_pending = False
  last_fresh_car_state = -math.inf
  next_mode_check = time.monotonic() + 0.5
  while True:
    sm.update(int(DT_DMON * 1000))
    now = dm_clock(sm, replay)
    mode_now = time.monotonic()
    if mode_now >= next_mode_check:
      dm.set_experimental(experimental_mode(params))
      next_mode_check = mode_now + 0.5
    if disable_write_pending or rk.frame % 40 == 0:
      setting_enabled = params.get_bool("DriverMonitoringEnabled")
      session_disabled = params.get_bool(SESSION_DISABLED_PARAM)
      requested_enabled = setting_enabled and not session_disabled
      if disable_write_pending:
        # Keep the local gate closed until the asynchronous session marker is
        # observable, retrying at the ordinary two-second settings cadence.
        if session_disabled:
          disable_write_pending = False
        elif rk.frame % 40 == 0:
          params.put_bool_nonblocking(SESSION_DISABLED_PARAM, True)
        requested_enabled = False
      if requested_enabled and not monitoring_enabled:
        dm = new_driver_monitor(params, experimental_mode(params))
        traffic, inputs = TrafficContext(), InteractionEdges()
        cancel_sequence = CancelPressSequence()
        automatic_cancel = AutomaticCancelFilter()
        camera_health = CameraAvailability()
        touch_evidence = SteeringTouchEvidence()
        strict, clear = True, False
      monitoring_enabled = requested_enabled
      if monitoring_enabled:
        dm.always_on = params.get_bool("AlwaysOnDM")
      demo_mode = params.get_bool("IsDriverViewEnabled")
      rhd_saved = params.get_bool("IsRhdDetected")
    if sm.updated['carParams']:
      cp = sm['carParams']
      covered = side_coverage(params, cp)
      # Stock-ACC speed button injection can be indistinguishable from driver
      # input after a gateway echo. Do not grant credit on that configuration.
      allow_speed_buttons = cp.openpilotLongitudinalControl or params.get_int("SpeedFromPCM") == 0
      new_filter_automatic_cancel = cancel_echo_filter_required(cp)
      if new_filter_automatic_cancel != filter_automatic_cancel:
        automatic_cancel = AutomaticCancelFilter()
      filter_automatic_cancel = new_filter_automatic_cancel
    cs = sm['carState']
    state_services = ['carState', 'selfdriveState']
    valid = sm.all_checks(state_services) and cs.canValid
    if replay:
      valid = valid and replay_services_fresh(sm, state_services, now)
    response = False
    record_automatic_cancel_requests(control_sock, automatic_cancel, now, replay, filter_automatic_cancel)
    state_packets = messaging.drain_sock(input_sock, wait_for_one=False)
    # carControl causes the transmitted CANCEL. Drain once more after taking
    # the carState snapshot so a cross-socket delivery race cannot make the
    # resulting gateway echo look like a physical press.
    record_automatic_cancel_requests(control_sock, automatic_cancel, now, replay, filter_automatic_cancel)
    for packet in state_packets:
      sample = packet.carState
      event_time = packet.logMonoTime / 1e9
      packet_now = now if replay else time.monotonic()
      fresh = packet.valid and sample.canValid and input_packet_fresh(packet_now, event_time)
      if fresh:
        gap = event_time - last_fresh_car_state
        if math.isfinite(last_fresh_car_state) and (gap < 0 or gap >= INPUT_FRESHNESS_SECONDS):
          cancel_sequence.reset_input_stream()
          automatic_cancel.reset_input_stream()
        last_fresh_car_state = event_time
        raw_buttons = [(str(be.type), be.pressed) for be in sample.buttonEvents]
        physical_buttons = (automatic_cancel.filter_buttons(event_time, raw_buttons)
                            if filter_automatic_cancel else raw_buttons)
        buttons = [(kind, pressed) for kind, pressed in physical_buttons
                   if allow_speed_buttons or kind not in inputs.SPEED_BUTTONS]
        # The disable gesture is deliberately conservative: every received
        # non-CANCEL button event breaks consecutiveness, even where stock-ACC
        # speed-button echoes cannot be distinguished from physical input.
        if monitoring_enabled and cancel_sequence.update(event_time, physical_buttons):
          monitoring_enabled = False
          disable_write_pending = True
          params.put_bool_nonblocking(SESSION_DISABLED_PARAM, True)
        response |= inputs.update(event_time, sample.gasPressed, sample.brakePressed, sample.steeringPressed, buttons)
      else:
        cancel_sequence.reset_input_stream()
        automatic_cancel.reset_input_stream()
        last_fresh_car_state = -math.inf
    now = dm_clock(sm, replay)
    if now - last_fresh_car_state >= INPUT_FRESHNESS_SECONDS:
      cancel_sequence.reset_input_stream()
      automatic_cancel.reset_input_stream()
    bt_action = bluetooth.read(allowed=valid and monitoring_enabled, now=now)
    if bt_action is not None:
      response = True
      inputs.last_response = now
    if not monitoring_enabled:
      camera_ok = False
      if demo_mode:
        camera_ok = camera_health.update(now, camera_sample_usable(sm, dm.wheel_on_right, demo=True,
                                                                  replay_now=now if replay else None) and
                                         0 <= now - sm.logMonoTime['driverStateV2'] / 1e9 < 0.5)
        if camera_ok and sm.updated['driverStateV2']:
          dm.run_step(sm, demo=True)
      preview_monitor = dm if demo_mode and camera_ok else None
      pm.send('driverMonitoringState', disabled_state_packet(rhd_saved, dm.experimental, preview_monitor,
                                                             camera_unavailable=demo_mode and not camera_ok))
      rk.keep_time()
      continue
    road_services = ['modelV2', 'radarState']
    road_ok = sm.all_checks(road_services) and not any(sm['radarState'].radarErrors.to_dict().values())
    if replay:
      road_ok = road_ok and replay_services_fresh(sm, road_services, now)
    if not road_ok or sm.updated['radarState']:
      model = sm['modelV2']
      straight = (abs(cs.steeringAngleDeg) < 5 and abs(cs.aEgo) < 0.5 and
                  len(model.orientationRate.z) >= 10 and max(abs(model.orientationRate.z[i]) for i in range(10)) < 0.01 and
                  len(model.laneLineProbs) == 4 and min(model.laneLineProbs[1], model.laneLineProbs[2]) > 0.8 and
                  not cs.leftBlinker and not cs.rightBlinker)
      strict, clear = traffic.update(now, traffic_observations(sm['radarState']), road_ok, straight, covered)
    camera_ok = camera_health.update(now, camera_sample_usable(sm, dm.wheel_on_right, demo_mode,
                                                              replay_now=now if replay else None) and
                                     0 <= now - sm.logMonoTime['driverStateV2'] / 1e9 < 0.5)
    touch_held, touch_edge = touch_evidence.update(now, cs.steeringTouch, valid)
    if touch_edge:
      inputs.last_response = max(inputs.last_response, cs.steeringTouch.sampleMonoTime / 1e9)
      response = True
    dm.configure_context(now, camera_ok, strict, clear)
    if valid:
      dm.record_interaction(inputs.last_response)
    if camera_ok:
      if sm.updated['driverStateV2'] and (valid or demo_mode):
        dm.run_step(sm, demo=demo_mode)
    elif valid:
      dm.run_without_camera((response or touch_held) and valid, sm['selfdriveState'].enabled,
                            cs.vEgo < dm.settings._ALERT_MIN_SPEED,
                            cs.gearShifter not in (car.CarState.GearShifter.drive, car.CarState.GearShifter.low))
    dm.update_parked_reset(now, parked_reset_eligible(sm, now, demo_mode))
    packet = dm.get_state_packet(valid=valid or (demo_mode and camera_ok))
    packet.driverMonitoringState.cameraUnavailable = not camera_ok
    packet.driverMonitoringState.dm2Experimental = dm.experimental
    packet.driverMonitoringState.dm2Disabled = False
    packet.driverMonitoringState.dm2StrictTimeRemaining = max(0.0, traffic.strict_until - now)
    packet.driverMonitoringState.dm2WheelTimeoutFactor = dm.wheel_factor
    pm.send('driverMonitoringState', packet)
    if camera_ok and rk.frame % 6000 == 0 and not demo_mode and dm.wheelpos_offsetter.filtered_stat.n > dm.settings._WHEELPOS_FILTER_MIN_COUNT:
      params.put_bool("IsRhdDetected", dm.wheel_on_right)
    rk.keep_time()


def main():
  config_realtime_process([0, 1, 2, 3], 5)
  params = Params()
  # Onroad, wait for the current fingerprint so the first wheel press cannot
  # race the 0.02 Hz carParams publication. Driver View remains nonblocking.
  cp_bytes = params.get("CarParams", block=params.get_bool("IsOnroad"))
  initial_car_params = messaging.log_from_bytes(cp_bytes, car.CarParams) if cp_bytes is not None else None
  run_dm2(params, experimental_mode(params), initial_car_params)


if __name__ == '__main__':
  main()
