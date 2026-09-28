"""20 Hz DM dispatcher: stock camera criteria or automatic interaction fallback."""
import time
import math

from openpilot.cereal import car
import openpilot.cereal.messaging as messaging
from openpilot.common.params import Params
from openpilot.common.realtime import DT_DMON, Ratekeeper, config_realtime_process
from openpilot.selfdrive.monitoring.config import experimental_mode
from openpilot.selfdrive.carrot.bluetooth.model import CommandReader
from openpilot.selfdrive.monitoring.dm2 import DriverMonitoring2
from openpilot.selfdrive.monitoring.dm2_context import CameraAvailability, InteractionEdges, ObjectObservation, SteeringTouchEvidence, TrafficContext


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


def camera_sample_usable(sm, rhd, demo=False):
  if not sm.all_checks(['driverStateV2'] if demo else ['driverStateV2', 'liveCalibration', 'modelV2']):
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


def run_dm2(params, experimental):
  services = ['carState', 'selfdriveState', 'modelV2', 'radarState', 'liveCalibration', 'carParams', 'driverStateV2']
  # Like stock DM, use the polled service's frequency (20 Hz). SubMaster
  # forbids specifying both poll and frequency. The bounded update timeout
  # below still lets the interaction fallback run when the camera is absent.
  sm = messaging.SubMaster(services, poll='driverStateV2')
  pm = messaging.PubMaster(['driverMonitoringState'])
  # carState is 100 Hz; conflating it to 20 Hz can lose a complete button press.
  input_sock = messaging.sub_sock('carState', conflate=False)
  dm = DriverMonitoring2(rhd_saved=params.get_bool("IsRhdDetected"), always_on=params.get_bool("AlwaysOnDM"),
                         experimental=experimental)
  traffic, inputs = TrafficContext(), InteractionEdges()
  bluetooth = CommandReader('attention')
  camera_health = CameraAvailability()
  touch_evidence = SteeringTouchEvidence()
  rk = Ratekeeper(int(1 / DT_DMON), print_delay_threshold=None)
  strict, clear = True, False
  covered = False
  allow_speed_buttons = False
  demo_mode = params.get_bool("IsDriverViewEnabled")
  next_mode_check = time.monotonic() + 0.5
  while True:
    sm.update(int(DT_DMON * 1000))
    now = time.monotonic()
    if now >= next_mode_check:
      dm.set_experimental(experimental_mode(params))
      next_mode_check = now + 0.5
    if sm.updated['carParams']:
      covered = side_coverage(params, sm['carParams'])
      # Stock-ACC speed button injection can be indistinguishable from driver
      # input after a gateway echo. Do not grant credit on that configuration.
      allow_speed_buttons = sm['carParams'].openpilotLongitudinalControl or params.get_int("SpeedFromPCM") == 0
    cs = sm['carState']
    valid = sm.all_checks(['carState', 'selfdriveState']) and cs.canValid
    response = False
    for packet in messaging.drain_sock(input_sock, wait_for_one=False):
      sample = packet.carState
      if packet.valid and sample.canValid and 0 <= now - packet.logMonoTime / 1e9 < 0.25:
        buttons = [(str(be.type), be.pressed) for be in sample.buttonEvents
                   if allow_speed_buttons or str(be.type) not in inputs.SPEED_BUTTONS]
        response |= inputs.update(now, sample.gasPressed, sample.brakePressed, sample.steeringPressed, buttons)
    bt_action = bluetooth.read(allowed=valid, now=now)
    if bt_action is not None:
      response = True
      inputs.last_response = now
    road_ok = sm.all_checks(['modelV2', 'radarState']) and not any(sm['radarState'].radarErrors.to_dict().values())
    if not road_ok or sm.updated['radarState']:
      model = sm['modelV2']
      straight = (abs(cs.steeringAngleDeg) < 5 and abs(cs.aEgo) < 0.5 and
                  len(model.orientationRate.z) >= 10 and max(abs(model.orientationRate.z[i]) for i in range(10)) < 0.01 and
                  len(model.laneLineProbs) == 4 and min(model.laneLineProbs[1], model.laneLineProbs[2]) > 0.8 and
                  not cs.leftBlinker and not cs.rightBlinker)
      strict, clear = traffic.update(now, traffic_observations(sm['radarState']), road_ok, straight, covered)
    camera_ok = camera_health.update(now, camera_sample_usable(sm, dm.wheel_on_right, demo_mode) and
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
    packet.driverMonitoringState.dm2StrictTimeRemaining = max(0.0, traffic.strict_until - now)
    packet.driverMonitoringState.dm2WheelTimeoutFactor = dm.wheel_factor
    pm.send('driverMonitoringState', packet)
    if rk.frame % 40 == 0:
      dm.always_on = params.get_bool("AlwaysOnDM")
      demo_mode = params.get_bool("IsDriverViewEnabled")
    if camera_ok and rk.frame % 6000 == 0 and not demo_mode and dm.wheelpos_offsetter.filtered_stat.n > dm.settings._WHEELPOS_FILTER_MIN_COUNT:
      params.put_bool("IsRhdDetected", dm.wheel_on_right)
    rk.keep_time()


def main():
  config_realtime_process([0, 1, 2, 3], 5)
  params = Params()
  run_dm2(params, experimental_mode(params))


if __name__ == '__main__':
  main()
