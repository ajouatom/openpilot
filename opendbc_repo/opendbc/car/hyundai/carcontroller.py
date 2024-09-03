from opendbc.can.packer import CANPacker
from opendbc.car import DT_CTRL, apply_driver_steer_torque_limits, common_fault_avoidance, make_tester_present_msg, structs
from opendbc.car.common.conversions import Conversions as CV
from opendbc.car.common.numpy_fast import clip
from opendbc.car.hyundai import hyundaicanfd, hyundaican
from opendbc.car.hyundai.carstate import CarState
from opendbc.car.hyundai.hyundaicanfd import CanBus
from opendbc.car.hyundai.values import HyundaiFlags, Buttons, CarControllerParams, CAR, CAN_GEARS, HyundaiExtFlags
from opendbc.car.interfaces import CarControllerBase

VisualAlert = structs.CarControl.HUDControl.VisualAlert
LongCtrlState = structs.CarControl.Actuators.LongControlState


from openpilot.common.params import Params

# EPS faults if you apply torque while the steering angle is above 90 degrees for more than 1 second
# All slightly below EPS thresholds to avoid fault
MAX_ANGLE = 85
MAX_ANGLE_FRAMES = 89
MAX_ANGLE_CONSECUTIVE_FRAMES = 2


def process_hud_alert(enabled, fingerprint, hud_control):
  sys_warning = (hud_control.visualAlert in (VisualAlert.steerRequired, VisualAlert.ldw))

  # initialize to no line visible
  # TODO: this is not accurate for all cars
  sys_state = 1
  if hud_control.leftLaneVisible and hud_control.rightLaneVisible or sys_warning:  # HUD alert only display when LKAS status is active
    sys_state = 3 if enabled or sys_warning else 4
  elif hud_control.leftLaneVisible:
    sys_state = 5
  elif hud_control.rightLaneVisible:
    sys_state = 6

  # initialize to no warnings
  left_lane_warning = 0
  right_lane_warning = 0
  if hud_control.leftLaneDepart:
    left_lane_warning = 1 if fingerprint in (CAR.GENESIS_G90, CAR.GENESIS_G80) else 2
  if hud_control.rightLaneDepart:
    right_lane_warning = 1 if fingerprint in (CAR.GENESIS_G90, CAR.GENESIS_G80) else 2

  return sys_warning, sys_state, left_lane_warning, right_lane_warning


class CarController(CarControllerBase):
  def __init__(self, dbc_name, CP):
    super().__init__(dbc_name, CP)
    self.CAN = CanBus(CP)
    self.params = CarControllerParams(CP)
    self.packer = CANPacker(dbc_name)
    self.angle_limit_counter = 0

    self.accel_last = 0
    self.apply_steer_last = 0
    self.car_fingerprint = CP.carFingerprint
    self.last_button_frame = 0

    self.hyundai_jerk = HyundaiJerk()
    self.speedCameraHapticEndFrame = 0
    self.hapticFeedbackWhenSpeedCamera = 0
    self.max_angle_frames = MAX_ANGLE_FRAMES
    self.blinking_signal = False # 1Hz
    self.blinking_frame = int(1.0 / DT_CTRL)
    self.soft_hold_mode = 2

    self.activateCruise = 0
    self.button_wait = 12
    self.cruise_buttons_msg_values = None
    self.cruise_buttons_msg_cnt = 0
    self.button_spamming_count = 0
    self.prev_clu_speed = 0
    self.button_spam1 = 8
    self.button_spam2 = 30
    self.button_spam3 = 1

    self.suppress_lfa_counter = 0

  def update(self, CC, CS, now_nanos):

    if self.frame % 50 == 0:
      params = Params()
      self.max_angle_frames = params.get_int("MaxAngleFrames")
      steerMax = params.get_int("CustomSteerMax")
      steerDeltaUp = params.get_int("CustomSteerDeltaUp")
      steerDeltaDown = params.get_int("CustomSteerDeltaDown")
      if steerMax > 0:
        self.params.STEER_MAX = steerMax
      if steerDeltaUp > 0:
        self.params.STEER_DELTA_UP = steerDeltaUp
      if steerDeltaDown > 0:
        self.params.STEER_DELTA_DOWN = steerDeltaDown
      self.soft_hold_mode = 1 if params.get_int("AutoCruiseControl") > 1 else 2
      self.hapticFeedbackWhenSpeedCamera = int(params.get_int("HapticFeedbackWhenSpeedCamera"))
      
      self.button_spam1 = params.get_int("CruiseButtonTest1")
      self.button_spam2 = params.get_int("CruiseButtonTest2")
      self.button_spam3 = params.get_int("CruiseButtonTest3")
      self.speed_from_pcm = params.get_int("SpeedFromPCM")
      

    actuators = CC.actuators
    hud_control = CC.hudControl

    # steering torque
    new_steer = int(round(actuators.steer * self.params.STEER_MAX))
    apply_steer = apply_driver_steer_torque_limits(new_steer, self.apply_steer_last, CS.out.steeringTorque, self.params)

    # >90 degree steering fault prevention
    self.angle_limit_counter, apply_steer_req = common_fault_avoidance(abs(CS.out.steeringAngleDeg) >= MAX_ANGLE, CC.latActive,
                                                                       self.angle_limit_counter, self.max_angle_frames,
                                                                       MAX_ANGLE_CONSECUTIVE_FRAMES)

    if not CC.latActive:
      apply_steer = 0

    # Hold torque with induced temporary fault when cutting the actuation bit
    torque_fault = CC.latActive and not apply_steer_req

    self.apply_steer_last = apply_steer

    # accel + longitudinal
    accel = clip(actuators.accel, CarControllerParams.ACCEL_MIN, CarControllerParams.ACCEL_MAX)
    stopping = actuators.longControlState == LongCtrlState.stopping
    set_speed_in_units = hud_control.setSpeed * (CV.MS_TO_KPH if CS.is_metric else CV.MS_TO_MPH)

    # HUD messages
    sys_warning, sys_state, left_lane_warning, right_lane_warning = process_hud_alert(CC.enabled, self.car_fingerprint,
                                                                                      hud_control)

    active_speed_decel = hud_control.activeCarrot == 3 # 3: Speed Decel
    if active_speed_decel and self.speedCameraHapticEndFrame < 0: # 과속카메라 감속시작
      self.speedCameraHapticEndFrame = self.frame + (8.0 / DT_CTRL)  #8초간 켜줌.
    elif not active_speed_decel:
      self.speedCameraHapticEndFrame = -1

    if self.frame < self.speedCameraHapticEndFrame and self.hapticFeedbackWhenSpeedCamera>0:
      haptic_stop = (self.speedCameraHapticEndFrame - (5.0/DT_CTRL)) < self.frame < (self.speedCameraHapticEndFrame - (3.0/DT_CTRL))
      if not haptic_stop:
         left_lane_warning = right_lane_warning = self.hapticFeedbackWhenSpeedCamera
      if self.speedCameraHapticEndFrame < self.frame:
        self.speedCameraHapticEndFrame = -1
     
    if self.frame % self.blinking_frame == 0:
      self.blinking_signal = True
    elif self.frame % self.blinking_frame == self.blinking_frame / 2:
      self.blinking_signal = False



    can_sends = []

    # *** common hyundai stuff ***

    # tester present - w/ no response (keeps relevant ECU disabled)
    if self.frame % 100 == 0 and not (self.CP.flags & HyundaiFlags.CANFD_CAMERA_SCC.value) and self.CP.openpilotLongitudinalControl:
      # for longitudinal control, either radar or ADAS driving ECU
      addr, bus = 0x7d0, self.CAN.ECAN if self.CP.flags & HyundaiFlags.CANFD else 0
      if self.CP.flags & HyundaiFlags.CANFD_HDA2.value:
        addr, bus = 0x730, self.CAN.ECAN
      can_sends.append(make_tester_present_msg(addr, bus, suppress_response=True))

      # for blinkers
      if self.CP.flags & HyundaiFlags.ENABLE_BLINKERS:
        can_sends.append(make_tester_present_msg(0x7b1, self.CAN.ECAN, suppress_response=True))

    camera_scc = self.CP.flags & HyundaiFlags.CAMERA_SCC
    # CAN-FD platforms
    if self.CP.flags & HyundaiFlags.CANFD:
      hda2 = self.CP.flags & HyundaiFlags.CANFD_HDA2
      hda2_long = hda2 and self.CP.openpilotLongitudinalControl

      # steering control
      can_sends.extend(hyundaicanfd.create_steering_messages(self.packer, self.CP, self.CAN, CC.enabled, apply_steer_req, apply_steer))

      # prevent LFA from activating on HDA2 by sending "no lane lines detected" to ADAS ECU
      if self.frame % 5 == 0 and hda2:
        if self.CP.extFlags & HyundaiExtFlags.ACAN_PANDA.value:
          self.suppress_lfa_counter += 1
          can_sends.append(hyundaicanfd.create_suppress_lfa_scc2(self.packer, self.CAN, self.CP.flags & HyundaiFlags.CANFD_HDA2_ALT_STEERING, self.suppress_lfa_counter))
        elif not camera_scc:
          can_sends.append(hyundaicanfd.create_suppress_lfa(self.packer, self.CAN, CS.hda2_lfa_block_msg,
                                                            self.CP.flags & HyundaiFlags.CANFD_HDA2_ALT_STEERING))

      # LFA and HDA icons
      if self.frame % 5 == 0 and (not hda2 or hda2_long):
        can_sends.append(hyundaicanfd.create_lfahda_cluster(self.packer, self.CAN, CC.enabled))

      # blinkers
      if hda2 and self.CP.flags & HyundaiFlags.ENABLE_BLINKERS:
        can_sends.extend(hyundaicanfd.create_spas_messages(self.packer, self.CAN, self.frame, CC.leftBlinker, CC.rightBlinker))

      if self.CP.openpilotLongitudinalControl:
        self.hyundai_jerk.make_jerk(self.CP, CS, accel, actuators, hud_control)

        if not camera_scc:
          if hda2:
            can_sends.extend(hyundaicanfd.create_adrv_messages(self.CP, self.packer, self.CAN, self.frame))
          else:
            can_sends.extend(hyundaicanfd.create_fca_warning_light(self.packer, self.CAN, self.frame))
        if self.frame % 2 == 0:
          if False: #self.CP.flags & HyundaiFlags.CAMERA_SCC.value:
            can_sends.append(hyundaicanfd.create_acc_control_scc2(self.packer, self.CAN, CC.enabled, self.accel_last, accel, stopping, CC.cruiseControl.override,
                                                             set_speed_in_units, hud_control, self.hyundai_jerk.jerk_u, self.hyundai_jerk.jerk_l, CS.cruise_info))
          else:
            can_sends.append(hyundaicanfd.create_acc_control(self.packer, self.CAN, CC.enabled, self.accel_last, accel, stopping, CC.cruiseControl.override,
                                                             set_speed_in_units, hud_control, self.hyundai_jerk.jerk_u, self.hyundai_jerk.jerk_l, CS))
          self.accel_last = accel
      else:
        # button presses
        can_sends.extend(self.create_button_messages(CC, CS, use_clu11=False))
    else:
      can_sends.append(hyundaican.create_lkas11(self.packer, self.frame, self.CP, apply_steer, apply_steer_req,
                                                torque_fault, CS.lkas11, sys_warning, sys_state, CC.enabled,
                                                hud_control.leftLaneVisible, hud_control.rightLaneVisible,
                                                left_lane_warning, right_lane_warning))

      if not self.CP.openpilotLongitudinalControl:
        can_sends.extend(self.create_button_messages(CC, CS, use_clu11=True))
      if self.CP.carFingerprint in CAN_GEARS["send_mdps12"]:  # send mdps12 to LKAS to prevent LKAS error
        can_sends.append(hyundaican.create_mdps12(self.packer, self.frame, CS.mdps12))

      if self.frame % 2 == 0 and self.CP.openpilotLongitudinalControl and camera_scc:
        self.hyundai_jerk.make_jerk(self.CP, CS, accel, actuators, hud_control)
        jerk = 3.0 if actuators.longControlState == LongCtrlState.pid else 1.0
        use_fca = self.CP.flags & HyundaiFlags.USE_FCA.value
        can_sends.extend(hyundaican.create_acc_commands_scc(self.packer, CC.enabled, accel, self.hyundai_jerk, int(self.frame / 2),
                                                        hud_control, set_speed_in_units, stopping,
                                                        CC.cruiseControl.override, use_fca, CS, self.soft_hold_mode))
      elif self.frame % 2 == 0 and self.CP.openpilotLongitudinalControl:
        self.hyundai_jerk.make_jerk(self.CP, CS, accel, actuators, hud_control)
        # TODO: unclear if this is needed
        jerk = 3.0 if actuators.longControlState == LongCtrlState.pid else 1.0
        use_fca = self.CP.flags & HyundaiFlags.USE_FCA.value
        can_sends.extend(hyundaican.create_acc_commands(self.packer, CC.enabled, accel, self.hyundai_jerk, int(self.frame / 2),
                                                        hud_control, set_speed_in_units, stopping,
                                                        CC.cruiseControl.override, use_fca, CS, self.soft_hold_mode))

      # 20 Hz LFA MFA message
      if self.frame % 5 == 0 and self.CP.flags & HyundaiFlags.SEND_LFA.value:
        can_sends.append(hyundaican.create_lfahda_mfc(self.packer, CC, self.blinking_signal))

      # 5 Hz ACC options
      if self.frame % 20 == 0 and self.CP.openpilotLongitudinalControl:
        if camera_scc:
          if CS.scc13 is not None:
            can_sends.append(hyundaican.create_acc_opt_copy(CS, self.packer))
        else:
          can_sends.extend(hyundaican.create_acc_opt(self.packer))

      # 2 Hz front radar options
      if self.frame % 50 == 0 and self.CP.openpilotLongitudinalControl and not camera_scc:
        can_sends.append(hyundaican.create_frt_radar_opt(self.packer))

    new_actuators = actuators.as_builder()
    new_actuators.steer = apply_steer / self.params.STEER_MAX
    new_actuators.steerOutputCan = apply_steer
    new_actuators.accel = accel

    self.frame += 1
    return new_actuators, can_sends

  def create_button_messages(self, CC: structs.CarControl, CS: CarState, use_clu11: bool):
    can_sends = []
    if CS.out.brakePressed or CS.out.brakeHoldActive:
      return can_sends
    if use_clu11:
      if CC.cruiseControl.cancel:
        can_sends.append(hyundaican.create_clu11(self.packer, self.frame, CS.clu11, Buttons.CANCEL, self.CP))
      elif False: #CC.cruiseControl.resume:
        # send resume at a max freq of 10Hz
        if (self.frame - self.last_button_frame) * DT_CTRL > 0.1:
          # send 25 messages at a time to increases the likelihood of resume being accepted
          can_sends.extend([hyundaican.create_clu11(self.packer, self.frame, CS.clu11, Buttons.RES_ACCEL, self.CP)] * 25)
          if (self.frame - self.last_button_frame) * DT_CTRL >= 0.15:
            self.last_button_frame = self.frame

      if self.last_button_frame != self.frame:
        send_button = self.make_spam_button(CC, CS)
        if send_button > 0:
          can_sends.append(hyundaican.create_clu11_button(self.packer, self.frame, CS.clu11, send_button, self.CP))
      
    else:

      # carrot.. 왜 alt_cruise_button는 값이 리스트일까?, 그리고 왜? 빈데이터가 들어오는것일까?
      if CS.cruise_buttons_msg is not None and self.CP.flags & HyundaiFlags.CANFD_ALT_BUTTONS:
        try:
          cruise_buttons_msg_values = {key: value[0] for key, value in CS.cruise_buttons_msg.items()}
        except IndexError:
          #print("IndexError....")
          cruise_buttons_msg_values = None
          self.cruise_buttons_msg_cnt += 1
        if cruise_buttons_msg_values is not None:
          self.cruise_buttons_msg_values = cruise_buttons_msg_values
          self.cruise_buttons_msg_cnt = 0

      if (self.frame - self.last_button_frame) * DT_CTRL > 0.25:
        # cruise cancel
        if CC.cruiseControl.cancel:
          if (self.frame - self.last_button_frame) * DT_CTRL > 0.1:
            print("cruiseControl.cancel222222")
            if self.CP.flags & HyundaiFlags.CANFD_ALT_BUTTONS:
              #can_sends.append(hyundaicanfd.create_acc_cancel(self.packer, self.CP, self.CAN, CS.cruise_info))
              if self.cruise_buttons_msg_values is not None:
                can_sends.append(hyundaicanfd.alt_cruise_buttons(self.packer, self.CP, self.CAN, Buttons.CANCEL, self.cruise_buttons_msg_values, self.cruise_buttons_msg_cnt))
            
            else:
              for _ in range(20):
                can_sends.append(hyundaicanfd.create_buttons(self.packer, self.CP, self.CAN, CS.buttons_counter+1, Buttons.CANCEL))
            self.last_button_frame = self.frame

        # cruise standstill resume
        elif False: #CC.cruiseControl.resume:
          if self.CP.flags & HyundaiFlags.CANFD_ALT_BUTTONS:
            # TODO: resume for alt button cars
            pass
          else:
            for _ in range(20):
              can_sends.append(hyundaicanfd.create_buttons(self.packer, self.CP, self.CAN, CS.buttons_counter+1, Buttons.RES_ACCEL))
            self.last_button_frame = self.frame

      ## button 스패밍을 안했을때...
      if self.last_button_frame != self.frame:
        dat = self.canfd_speed_control_pcm(CC, CS, self.cruise_buttons_msg_values)
        if dat is not None:
          for _ in range(self.button_spam3):
            can_sends.append(dat)
          self.cruise_buttons_msg_cnt += 1

    return can_sends

  def canfd_speed_control_pcm(self, CC, CS, cruise_buttons_msg_values):

    alt_buttons = True if self.CP.flags & HyundaiFlags.CANFD_ALT_BUTTONS else False

    if alt_buttons and cruise_buttons_msg_values is None:
      return None

    send_button = self.make_spam_button(CC, CS)
    if send_button > 0:
      if alt_buttons:
        return hyundaicanfd.alt_cruise_buttons(self.packer, self.CP, self.CAN, send_button, cruise_buttons_msg_values, self.cruise_buttons_msg_cnt)
      else:
        return hyundaicanfd.create_buttons(self.packer, self.CP, self.CAN, CS.buttons_counter+1, send_button)

    return None


  def make_spam_button(self, CC, CS):
    hud_control = CC.hudControl
    set_speed_in_units = hud_control.setSpeed * (CV.MS_TO_KPH if CS.is_metric else CV.MS_TO_MPH)
    target = int(set_speed_in_units+0.5)
    current = int(CS.out.cruiseState.speed*CV.MS_TO_KPH + 0.5)
    v_ego_kph = CS.out.vEgo * CV.MS_TO_KPH

    send_button = 0
    activate_cruise = False

    if CC.enabled:
      if not CS.out.cruiseState.enabled:
        if (hud_control.leadVisible or v_ego_kph > 10.0) and self.activateCruise == 0:
          send_button = Buttons.RES_ACCEL
          self.activateCruise = 1
          activate_cruise = True
      elif CC.cruiseControl.resume:
        send_button = Buttons.RES_ACCEL
      elif target < current and current>= 31 and self.speed_from_pcm != 1:
        send_button = Buttons.SET_DECEL
      elif target > current and current < 160 and self.speed_from_pcm != 1:
        send_button = Buttons.RES_ACCEL
    elif CS.out.activateCruise: #CC.cruiseControl.activate:
      if (hud_control.leadVisible or v_ego_kph > 10.0) and self.activateCruise == 0:
        self.activateCruise = 1
        send_button = Buttons.RES_ACCEL
        activate_cruise = True

    if CS.out.brakePressed or CS.out.gasPressed:
      self.activateCruise = 0

    if send_button == 0:
      self.button_spamming_count = 0
      self.prev_clu_speed = current      
      return 0

    speed_diff = self.prev_clu_speed - current
    spamming_max = self.button_spam1
    if CS.cruise_buttons[-1] != Buttons.NONE:
      self.last_button_frame = self.frame
      self.button_wait = self.button_spam2
      self.button_spamming_count = 0
    elif abs(self.button_spamming_count) >= spamming_max or abs(speed_diff) > 0:
      self.last_button_frame = self.frame
      self.button_wait = self.button_spam2 if abs(self.button_spamming_count) >= spamming_max else 7
      self.button_spamming_count = 0

    self.prev_clu_speed = current
    send_button_allowed = (self.frame - self.last_button_frame) > self.button_wait
    #CC.debugTextCC = "{} speed_diff={:.1f},{:.0f}/{:.0f}, button={}, button_wait={}, count={}".format(
    #  send_button_allowed, speed_diff, target, current, send_button, self.button_wait, self.button_spamming_count)

    if send_button_allowed or activate_cruise or (CC.cruiseControl.resume and self.frame % 2 == 0):
      self.button_spamming_count = self.button_spamming_count + 1 if send_button == Buttons.RES_ACCEL else self.button_spamming_count - 1
      return send_button
    else:
      self.button_spamming_count = 0
    return 0

from openpilot.common.filter_simple import StreamingMovingAverage
class HyundaiJerk:
  def __init__(self):
    self.jerk = 0.0
    self.jerk_u = self.jerk_l = 0.0
    self.cb_upper = self.cb_lower = 0.0
    self.jerk_u_min = 0.5

  def make_jerk(self, CP, CS, accel, actuators, hud_control):
    jerk = actuators.jerk if actuators.longControlState == LongCtrlState.pid else 0.0
    a_error = actuators.aTargetNow - CS.out.aEgo
    self.jerk = jerk + a_error

    jerk_max_l = 5.0
    jerk_max_u = jerk_max_l
    if actuators.longControlState == LongCtrlState.off:
      self.jerk_u = jerk_max_u
      self.jerk_l = jerk_max_l
      self.cb_upper = self.cb_lower = 0.0
    else:
      if CP.flags & HyundaiFlags.CANFD:
        self.jerk_u = min(max(self.jerk_u_min, self.jerk * 2.0), jerk_max_u)
        self.jerk_l = min(max(1.0, -self.jerk * 3.0), jerk_max_l)
        self.cb_upper = self.cb_lower = 0.0
      else:
        self.jerk_u = min(max(self.jerk_u_min, self.jerk * 2.0), jerk_max_u)
        self.jerk_l = min(max(0.5, -self.jerk * 2.0), jerk_max_l)
        self.cb_upper = clip(0.9 + accel * 0.2, 0, 1.2)
        self.cb_lower = clip(0.8 + accel * 0.2, 0, 1.2)

