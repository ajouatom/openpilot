#!/usr/bin/env python3
import os
import time
import threading

import openpilot.cereal.messaging as messaging

from openpilot.cereal import car, log

from openpilot.common.params import Params
from openpilot.common.realtime import config_realtime_process, Priority, Ratekeeper
from openpilot.common.swaglog import cloudlog, ForwardingHandler

from opendbc.car import DT_CTRL, structs
from opendbc.car.can_definitions import CanData, CanRecvCallable, CanSendCallable
from opendbc.car.carlog import carlog
from opendbc.car.fw_versions import ObdCallback
from opendbc.car.car_helpers import get_car, interfaces
from opendbc.car.interfaces import CarInterfaceBase, RadarInterfaceBase
from openpilot.selfdrive.pandad import can_capnp_to_list, can_list_to_can_capnp
from openpilot.selfdrive.car.alternative_experience import get_alternative_experience
from openpilot.selfdrive.car.card_diagnostics import should_log_card_diagnostics
from openpilot.selfdrive.car.cruise import VCruiseCarrot
from openpilot.selfdrive.car.car_specific import MockCarState
from openpilot.selfdrive.car.openpilot_toggle import CruiseMainOpenpilotToggle
from openpilot.selfdrive.carrot.xiaoge_lane import (
  XiaogeLaneResult,
  apply_xiaoge_lane_result,
  parse_xiaoge_lane_payload,
)

REPLAY = "REPLAY" in os.environ
XIAOGE_LANE_ERROR_LOG_INTERVAL_NS = 5_000_000_000

EventName = log.OnroadEvent.EventName
ButtonType = car.CarState.ButtonEvent.Type

# forward
carlog.addHandler(ForwardingHandler(cloudlog))


def obd_callback(params: Params) -> ObdCallback:
  def set_obd_multiplexing(obd_multiplexing: bool):
    if params.get_bool("ObdMultiplexingEnabled") != obd_multiplexing:
      cloudlog.warning(f"Setting OBD multiplexing to {obd_multiplexing}")
      params.remove("ObdMultiplexingChanged")
      params.put_bool("ObdMultiplexingEnabled", obd_multiplexing)
      params.get_bool("ObdMultiplexingChanged", block=True)
      cloudlog.warning("OBD multiplexing set successfully")
  return set_obd_multiplexing


def can_comm_callbacks(logcan: messaging.SubSocket, sendcan: messaging.PubSocket) -> tuple[CanRecvCallable, CanSendCallable]:
  def can_recv(wait_for_one: bool = False) -> list[list[CanData]]:
    """
    wait_for_one: wait the normal logcan socket timeout for a CAN packet, may return empty list if nothing comes

    Returns: CAN packets comprised of CanData objects for easy access
    """
    ret = []
    for can in messaging.drain_sock(logcan, wait_for_one=wait_for_one):
      ret.append([CanData(msg.address, msg.dat, msg.src) for msg in can.can])
    return ret

  def can_send(msgs: list[CanData]) -> None:
    sendcan.send(can_list_to_can_capnp(msgs, msgtype='sendcan'))

  return can_recv, can_send


class Car:
  CI: CarInterfaceBase
  RI: RadarInterfaceBase
  CP: car.CarParams

  def __init__(self, CI=None, RI=None) -> None:
    self.can_sock = messaging.sub_sock('can', timeout=20)
    self.sm = messaging.SubMaster(['pandaStates', 'carControl', 'onroadEvents', 'carrotMan', 'longitudinalPlan',
                                   'radarState', 'modelV2', 'drivingModelData', 'customReservedRawData0'])
    self.pm = messaging.PubMaster(['sendcan', 'carState', 'carParams', 'carOutput', 'liveTracks'])

    self.can_rcv_cum_timeout_counter = 0

    self.CC_prev = car.CarControl.new_message()
    self.CS_prev = car.CarState.new_message()
    self.initialized_prev = False
    self.cruise_main_toggle = CruiseMainOpenpilotToggle(ButtonType.mainCruise)

    self.last_actuators_output = structs.CarControl.Actuators()

    self.params = Params()

    self.can_callbacks = can_comm_callbacks(self.can_sock, self.pm.sock['sendcan'])

    is_release = True #self.params.get_bool("IsReleaseBranch")

    if CI is None:
      # wait for one pandaState and one CAN packet
      print("Waiting for CAN messages...")
      while True:
        can = messaging.recv_one_retry(self.can_sock)
        if len(can.can) > 0:
          break

      alpha_long_allowed = self.params.get_bool("AlphaLongitudinalEnabled")
      num_pandas = len(messaging.recv_one_retry(self.sm.sock['pandaStates']).pandaStates)

      cached_params = None
      cached_params_raw = self.params.get("CarParamsCache")
      if cached_params_raw is not None:
        with car.CarParams.from_bytes(cached_params_raw) as _cached_params:
          cached_params = _cached_params

      self.CI = get_car(*self.can_callbacks, obd_callback(self.params), alpha_long_allowed, is_release, num_pandas, cached_params)
      self.RI = interfaces[self.CI.CP.carFingerprint].RadarInterface(self.CI.CP)
      self.CP = self.CI.CP

      # continue onto next fingerprinting step in pandad
      self.params.put_bool("FirmwareQueryDone", True)
    else:
      self.CI, self.CP = CI, CI.CP
      self.RI = RI

    self.CP.alternativeExperience = get_alternative_experience(self.params.get_bool("DisengageOnAccelerator"))
    openpilot_enabled_toggle = self.params.get_bool("OpenpilotEnabledToggle")
    controller_available = self.CI.CC is not None and openpilot_enabled_toggle and not self.CP.dashcamOnly
    self.CP.passive = not controller_available or self.CP.dashcamOnly
    if self.CP.passive:
      safety_config = structs.CarParams.SafetyConfig()
      safety_config.safetyModel = structs.CarParams.SafetyModel.noOutput
      self.CP.safetyConfigs = [safety_config]

    if self.CP.secOcRequired:
      # Copy user key if available
      try:
        with open("/cache/params/SecOCKey") as f:
          user_key = f.readline().strip()
          if len(user_key) == 32:
            self.params.put("SecOCKey", user_key)
      except Exception:
        pass

      secoc_key = self.params.get("SecOCKey")
      if secoc_key is not None:
        saved_secoc_key = bytes.fromhex(secoc_key.strip())
        if len(saved_secoc_key) == 16:
          self.CP.secOcKeyAvailable = True
          self.CI.CS.secoc_key = saved_secoc_key
          if controller_available:
            self.CI.CC.secoc_key = saved_secoc_key
        else:
          cloudlog.warning("Saved SecOC key is invalid")

    # Write previous route's CarParams
    prev_cp = self.params.get("CarParamsPersistent")
    if prev_cp is not None:
      self.params.put("CarParamsPrevRoute", prev_cp)

    # Write CarParams for controls and radard
    cp_bytes = self.CP.to_bytes()
    self.params.put("CarParams", cp_bytes)
    self.params.put_nonblocking("CarParamsCache", cp_bytes)
    self.params.put_nonblocking("CarParamsPersistent", cp_bytes)

    self.mock_carstate = MockCarState()
    self.v_cruise_helper = VCruiseCarrot(self.CP) #VCruiseHelper(self.CP)

    self.is_metric = self.params.get_bool("IsMetric")
    self.experimental_mode = self.params.get_bool("ExperimentalMode")

    #self.t1 = time.monotonic()
    #self.t2 = self.t1
    #self.t3 = self.t2
    # card is driven by can recv, expected at 100Hz
    self.rk = Ratekeeper(100, print_delay_threshold=None)
    self.card_diag_recv_ns = 0
    self.card_diag_prev_recv_ns = 0
    self.card_diag_frames = 0
    self.card_diag_loop_max_us = 0
    self.card_diag_process_max_us = 0
    self.card_diag_slow_loop = 0
    self.card_diag_slow_process = 0
    self.card_diag_can_timeouts = 0
    self.xiaoge_lane_result: XiaogeLaneResult | None = None
    self.xiaoge_lane_error_log_at_ns = 0
    self.card_diag_stage_names = ('decode', 'ci_update', 'sm_update', 'radar', 'state_tail',
                                  'state_total', 'publish', 'apply', 'sendcan', 'total')
    self.card_diag_stage_current = dict.fromkeys(self.card_diag_stage_names, 0)
    self.card_diag_stage_sum_us = dict.fromkeys(self.card_diag_stage_names, 0)
    self.card_diag_stage_max_us = dict.fromkeys(self.card_diag_stage_names, 0)

  def state_update(self) -> tuple[car.CarState, structs.RadarDataT | None]:
    """carState update loop, driven by can"""

    can_strs = messaging.drain_sock_raw(self.can_sock, wait_for_one=True)
    recv_ns = time.monotonic_ns()
    if self.card_diag_prev_recv_ns != 0:
      loop_us = (recv_ns - self.card_diag_prev_recv_ns) // 1000
      self.card_diag_loop_max_us = max(self.card_diag_loop_max_us, loop_us)
      self.card_diag_slow_loop += loop_us > 12000
    self.card_diag_prev_recv_ns = recv_ns
    self.card_diag_recv_ns = recv_ns
    can_list = can_capnp_to_list(can_strs)
    decode_done_ns = time.monotonic_ns()

    rcv_time = time.time()

    # Update carState from CAN
    CS = self.CI.update(can_list)
    if self.CP.brand == 'mock':
      CS = self.mock_carstate.update(CS)
    ci_done_ns = time.monotonic_ns()

    # Update radar tracks from CAN
    #RD: structs.RadarDataT | None = self.RI.update_carrot(CS.vEgo, can_list)

    self.sm.update(0)
    sm_done_ns = time.monotonic_ns()
    if self.sm.updated['customReservedRawData0']:
      try:
        self.xiaoge_lane_result = parse_xiaoge_lane_payload(bytes(self.sm['customReservedRawData0']))
      except (UnicodeDecodeError, ValueError, TypeError) as error:
        self.xiaoge_lane_result = None
        if sm_done_ns - self.xiaoge_lane_error_log_at_ns >= XIAOGE_LANE_ERROR_LOG_INTERVAL_NS:
          cloudlog.warning(f"invalid Xiaoge lane payload: {error}")
          self.xiaoge_lane_error_log_at_ns = sm_done_ns
    apply_xiaoge_lane_result(CS, self.xiaoge_lane_result, sm_done_ns)
    #self.t1 = time.monotonic()

    can_rcv_valid = len(can_strs) > 0

    # Check for CAN timeout
    if not can_rcv_valid:
      self.can_rcv_cum_timeout_counter += 1
      self.card_diag_can_timeouts += 1

    if can_rcv_valid and REPLAY:
      self.can_log_mono_time = messaging.log_from_bytes(can_strs[0]).logMonoTime

    RD: structs.RadarDataT | None = self.RI.update_carrot(CS.vEgo, CS.aEgo, rcv_time, can_list)
    radar_done_ns = time.monotonic_ns()
    #self.t2 = time.monotonic()

    #self.v_cruise_helper.update_v_cruise(CS, self.sm['carControl'].enabled, self.is_metric)
    self.v_cruise_helper.update_v_cruise(CS, self.sm, self.is_metric)
    #self.t3 = time.monotonic()
    if self.sm['carControl'].enabled and not self.CC_prev.enabled:
      # Use CarState w/ buttons from the step selfdrived enables on
      self.v_cruise_helper.initialize_v_cruise(self.CS_prev, self.experimental_mode)

    # TODO: mirror the carState.cruiseState struct?
    #self.v_cruise_helper.update_v_cruise(CS, self.sm['carControl'].enabled, self.is_metric)
    if self.v_cruise_helper._paddle_decel_active:
      v_cruise_kph = v_cruise_cluster_kph = 0
    else:
      v_cruise_kph = self.v_cruise_helper.v_cruise_kph
      v_cruise_cluster_kph = self.v_cruise_helper.v_cruise_cluster_kph
    CS.logCarrot = self.v_cruise_helper.log
    CS.vCruise = float(v_cruise_kph)
    CS.vCruiseCluster = float(v_cruise_cluster_kph)
    CS.softHoldActive = self.v_cruise_helper._soft_hold_active
    CS.activateCruise = self.v_cruise_helper._activate_cruise
    CS.latEnabled = self.v_cruise_helper._lat_enabled
    CS.useLaneLineSpeed = self.v_cruise_helper.useLaneLineSpeedApply
    CS.carrotCruise = 1 if self.v_cruise_helper.carrot_cruise_active else 0

    self.CI.CS.softHoldActive = CS.softHoldActive
    state_done_ns = time.monotonic_ns()
    self.card_diag_stage_current = {
      'decode': (decode_done_ns - recv_ns) // 1000,
      'ci_update': (ci_done_ns - decode_done_ns) // 1000,
      'sm_update': (sm_done_ns - ci_done_ns) // 1000,
      'radar': (radar_done_ns - sm_done_ns) // 1000,
      'state_tail': (state_done_ns - radar_done_ns) // 1000,
      'state_total': (state_done_ns - recv_ns) // 1000,
      'publish': 0,
      'apply': 0,
      'sendcan': 0,
      'total': 0,
    }
    return CS, RD

  def state_publish(self, CS: car.CarState, RD: structs.RadarDataT | None):
    """carState and carParams publish loop"""

    # carParams - logged every 50 seconds (> 1 per segment)
    if self.sm.frame % int(50. / DT_CTRL) == 0:
      cp_send = messaging.new_message('carParams')
      cp_send.valid = True
      cp_send.carParams = self.CP
      self.pm.send('carParams', cp_send)

    # publish new carOutput
    co_send = messaging.new_message('carOutput')
    co_send.valid = self.sm.all_checks(['carControl'])
    co_send.carOutput.actuatorsOutput = self.last_actuators_output
    self.pm.send('carOutput', co_send)

    # kick off controlsd step while we actuate the latest carControl packet
    cs_send = messaging.new_message('carState')
    cs_send.valid = CS.canValid
    cs_send.carState = CS
    cs_send.carState.canErrorCounter = self.can_rcv_cum_timeout_counter
    cs_send.carState.cumLagMs = -self.rk.remaining * 1000.
    self.pm.send('carState', cs_send)

    if RD is not None:
      tracks_msg = messaging.new_message('liveTracks')
      tracks_msg.valid = not any(RD.errors.to_dict().values())
      tracks_msg.liveTracks = RD
      self.pm.send('liveTracks', tracks_msg)

  def controls_update(self, CS: car.CarState, CC: car.CarControl):
    """control update loop, driven by carControl"""

    if not self.initialized_prev:
      # Initialize CarInterface, once controls are ready
      # TODO: this can make us miss at least a few cycles when doing an ECU knockout
      self.CI.init(self.CP, *self.can_callbacks)
      # signal pandad to switch to car safety mode
      self.params.put_bool_nonblocking("ControlsReady", True)

    if self.sm.all_alive(['carControl']):
      # send car controls over can
      apply_start_ns = time.monotonic_ns()
      now_nanos = self.can_log_mono_time if REPLAY else int(time.monotonic() * 1e9)
      model_v2 = self.sm['modelV2'] if self.sm.valid['modelV2'] and self.sm.alive['modelV2'] else None
      self.last_actuators_output, can_sends = self.CI.apply(CC, now_nanos, model_v2)
      apply_done_ns = time.monotonic_ns()
      self.pm.send('sendcan', can_list_to_can_capnp(can_sends, msgtype='sendcan', valid=CS.canValid))
      sendcan_done_ns = time.monotonic_ns()

      process_us = (sendcan_done_ns - self.card_diag_recv_ns) // 1000
      self.card_diag_stage_current['apply'] = (apply_done_ns - apply_start_ns) // 1000
      self.card_diag_stage_current['sendcan'] = (sendcan_done_ns - apply_done_ns) // 1000
      self.card_diag_stage_current['total'] = process_us
      for name in self.card_diag_stage_names:
        stage_us = self.card_diag_stage_current[name]
        self.card_diag_stage_sum_us[name] += stage_us
        self.card_diag_stage_max_us[name] = max(self.card_diag_stage_max_us[name], stage_us)
      self.card_diag_process_max_us = max(self.card_diag_process_max_us, process_us)
      self.card_diag_slow_process += process_us > 5000
      self.card_diag_frames += 1
      if self.card_diag_frames >= 100:
        if should_log_card_diagnostics(self.card_diag_loop_max_us, self.card_diag_process_max_us, self.card_diag_can_timeouts):
          print(f"card_sendcan_diag: can_timeouts={self.card_diag_can_timeouts}, "
                f"loop_max_us={self.card_diag_loop_max_us}, process_max_us={self.card_diag_process_max_us}, "
                f"loop_over_12ms={self.card_diag_slow_loop}, process_over_5ms={self.card_diag_slow_process}")
          print(f"card_stage_state_diag: decode_avg_us={self.card_diag_stage_sum_us['decode'] // self.card_diag_frames}, "
                f"decode_max_us={self.card_diag_stage_max_us['decode']}, "
                f"ci_avg_us={self.card_diag_stage_sum_us['ci_update'] // self.card_diag_frames}, "
                f"ci_max_us={self.card_diag_stage_max_us['ci_update']}, "
                f"sm_avg_us={self.card_diag_stage_sum_us['sm_update'] // self.card_diag_frames}, "
                f"sm_max_us={self.card_diag_stage_max_us['sm_update']}, "
                f"radar_avg_us={self.card_diag_stage_sum_us['radar'] // self.card_diag_frames}, "
                f"radar_max_us={self.card_diag_stage_max_us['radar']}, "
                f"tail_avg_us={self.card_diag_stage_sum_us['state_tail'] // self.card_diag_frames}, "
                f"tail_max_us={self.card_diag_stage_max_us['state_tail']}, "
                f"state_avg_us={self.card_diag_stage_sum_us['state_total'] // self.card_diag_frames}, "
                f"state_max_us={self.card_diag_stage_max_us['state_total']}")
          print(f"card_stage_send_diag: publish_avg_us={self.card_diag_stage_sum_us['publish'] // self.card_diag_frames}, "
                f"publish_max_us={self.card_diag_stage_max_us['publish']}, "
                f"apply_avg_us={self.card_diag_stage_sum_us['apply'] // self.card_diag_frames}, "
                f"apply_max_us={self.card_diag_stage_max_us['apply']}, "
                f"sendcan_avg_us={self.card_diag_stage_sum_us['sendcan'] // self.card_diag_frames}, "
                f"sendcan_max_us={self.card_diag_stage_max_us['sendcan']}, "
                f"total_avg_us={self.card_diag_stage_sum_us['total'] // self.card_diag_frames}, "
                f"total_max_us={self.card_diag_stage_max_us['total']}")
        self.card_diag_frames = 0
        self.card_diag_loop_max_us = 0
        self.card_diag_process_max_us = 0
        self.card_diag_slow_loop = 0
        self.card_diag_slow_process = 0
        self.card_diag_can_timeouts = 0
        self.card_diag_stage_sum_us = dict.fromkeys(self.card_diag_stage_names, 0)
        self.card_diag_stage_max_us = dict.fromkeys(self.card_diag_stage_names, 0)

      self.CC_prev = CC

  def step(self):
    CS, RD = self.state_update()

    if self.cruise_main_toggle.update(CS.buttonEvents, self.sm['carControl'].enabled):
      if self.CI.CC is not None and not self.CP.dashcamOnly:
        openpilot_enabled = not self.params.get_bool("OpenpilotEnabledToggle")
        cloudlog.warning(f"Cruise MAIN long press: setting OpenpilotEnabledToggle to {openpilot_enabled}")
        self.params.put_bool("OpenpilotEnabledToggle", openpilot_enabled)
        self.params.put_bool("OnroadCycleRequested", True)
      else:
        cloudlog.warning("Cruise MAIN long press ignored: vehicle has no openpilot controller")

    publish_start_ns = time.monotonic_ns()
    self.state_publish(CS, RD)
    self.card_diag_stage_current['publish'] = (time.monotonic_ns() - publish_start_ns) // 1000

    initialized = (not any(e.name == EventName.selfdriveInitializing for e in self.sm['onroadEvents']) and
                   self.sm.seen['onroadEvents'])
    if not self.CP.passive and initialized:
      self.controls_update(CS, self.sm['carControl'])

    self.initialized_prev = initialized
    self.CS_prev = CS

  def params_thread(self, evt):
    while not evt.is_set():
      self.is_metric = self.params.get_bool("IsMetric")
      self.experimental_mode = self.params.get_bool("ExperimentalMode") and self.CP.openpilotLongitudinalControl
      time.sleep(0.1)

  def card_thread(self):
    e = threading.Event()
    t = threading.Thread(target=self.params_thread, args=(e, ))
    try:
      t.start()
      while True:
        #start = time.monotonic()
        self.step()
        #if self.sm.frame % 100 == 0:
        #  print(f"elapsed time = {(self.t1 - start)*1000.:.2f}, {(self.t2 - self.t1)*1000.:.2f}, {(self.t3 - self.t1)*1000.:.2f}, {(time.monotonic() - self.t1)*1000.:.2f}")
        self.rk.monitor_time()
    finally:
      e.set()
      t.join()
    
def main():
  #config_realtime_process(4, Priority.CTRL_HIGH)
  config_realtime_process(6, Priority.CTRL_HIGH)
  car = Car()
  car.card_thread()


if __name__ == "__main__":
  main()
