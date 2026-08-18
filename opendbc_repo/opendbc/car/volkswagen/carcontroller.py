import numpy as np
from opendbc.can import CANPacker
from opendbc.car import Bus, DT_CTRL, apply_driver_steer_torque_limits, structs
from opendbc.car.common.conversions import Conversions as CV
from opendbc.car.interfaces import CarControllerBase
from opendbc.car.lateral import apply_std_curvature_limits
from opendbc.car.volkswagen import mqbcan, pqcan, mebcan
from opendbc.car.volkswagen.values import CANBUS, CarControllerParams, VolkswagenFlags

VisualAlert = structs.CarControl.HUDControl.VisualAlert
LongCtrlState = structs.CarControl.Actuators.LongControlState


class CarController(CarControllerBase):
  def __init__(self, dbc_names, CP):
    super().__init__(dbc_names, CP)
    self.CCP = CarControllerParams(CP)
    if CP.flags & VolkswagenFlags.PQ:
      self.CCS = pqcan
    elif CP.flags & VolkswagenFlags.MEB:
      self.CCS = mebcan
    else:
      self.CCS = mqbcan
    self.packer_pt = CANPacker(dbc_names[Bus.pt])
    self.ext_bus = CANBUS.pt if CP.networkLocation == structs.CarParams.NetworkLocation.fwdCamera else CANBUS.cam
    self.aeb_available = not CP.flags & VolkswagenFlags.PQ

    self.apply_torque_last = 0
    self.apply_curvature_last = 0.
    self.steering_power_last = 0
    self.gra_acc_counter_last = None
    self.eps_timer_soft_disable_alert = False
    self.hca_frame_timer_running = 0
    self.hca_frame_same_torque = 0
    # MEB longitudinal (ported from infiniteCable2): EPB-error mitigation ramp counters
    self.long_override_counter = 0
    self.hold_release_frames = 0
    self.long_disabled_counter = 0
    self.klr_counter_last = None  # capacitive wheel touch (EA hands-on pacification)

    # 계기판 내비 이벤트 배너->미니 아이콘 전환 (배너 ~1.2초 후 Lead_Brightness 아이콘으로 유지)
    self.navi_event_last = 0
    self.navi_banner_frames = 0
    # 도로제한속도: 제한값이 바뀔 때만 "인식됨" 배너 후 km/h 아이콘 유지
    self.road_limit_last = 0
    self.road_banner_frames = 0
    # 앞차 지배 표시 디바운스 (정체 깜빡임 방지)
    self.lead_limit_disp = False
    self.lead_limit_cnt = 0

    self.acc_hold_type_last = mebcan.ACC_HMS_NO_REQUEST  # last ACC_Anforderung_HMS sent (hold-release ramp)

  def update(self, CC, CS, now_nanos):
    actuators = CC.actuators
    hud_control = CC.hudControl
    can_sends = []


    # **** Steering Controls ************************************************ #

    if self.frame % self.CCP.STEER_STEP == 0:
      if self.CP.flags & VolkswagenFlags.MEB:
        # MEB (ID.4 등) curvature 기반 조향 제어
        # HCA_03 메시지로 곡률(curvature) 및 파워(power %) 전송
        if CC.latActive:
          hca_enabled = True
          # infiniteCable2 방식: openpilot core 곡률 순수 passthrough + rate limit
          apply_curvature = apply_std_curvature_limits(actuators.curvature, self.apply_curvature_last,
                                                       CS.out.vEgoRaw, CS.curvature,
                                                       self.CCP.STEER_STEP, CC.latActive,
                                                       self.CCP.CURVATURE_LIMITS)
          # 조향 파워 계산 (운전자 개입 감지 시 감소).
          # infiniteCable2는 '부호 있는' steeringTorque를 그대로 np.interp에 넣어 음(-) 방향 개입에서는
          # 항상 POWER_MAX로 클램프됐다 (한쪽 방향만 파워가 빠지는 좌우 비대칭).
          # commaai/opendbc는 abs()를 쓴다 -> 양쪽 개입에 동일하게 반응하도록 수정.
          min_power = max(self.steering_power_last - self.CCP.STEERING_POWER_STEP, self.CCP.STEERING_POWER_MIN)
          max_power = min(self.steering_power_last + self.CCP.STEERING_POWER_STEP, self.CCP.STEERING_POWER_MAX)
          target_power_driver = int(np.interp(abs(CS.out.steeringTorque),
                                              [self.CCP.STEER_DRIVER_ALLOWANCE, self.CCP.STEER_DRIVER_MAX],
                                              [self.CCP.STEERING_POWER_MAX, self.CCP.STEERING_POWER_MIN]))
          target_power = int(np.interp(CS.out.vEgo, [0., 0.5], [self.CCP.STEERING_POWER_MIN, target_power_driver]))
          steering_power = min(max(target_power, min_power), max_power)
        else:
          # 비활성화 시 파워를 천천히 0으로 줄여 HCA 상태 유지
          if self.steering_power_last > 0:
            hca_enabled = True
            apply_curvature = float(np.clip(CS.curvature,
                                            -self.CCP.CURVATURE_LIMITS.CURVATURE_MAX,
                                            self.CCP.CURVATURE_LIMITS.CURVATURE_MAX))
            steering_power = max(self.steering_power_last - self.CCP.STEERING_POWER_STEP, 0)
          else:
            hca_enabled = False
            apply_curvature = 0.
            steering_power = 0

        can_sends.append(self.CCS.create_steering_control(self.packer_pt, CANBUS.pt, apply_curvature, hca_enabled, steering_power))
        self.apply_curvature_last = apply_curvature
        self.steering_power_last = steering_power

      else:
        # MQB/PQ 토크 기반 조향 제어
        # Logic to avoid HCA state 4 "refused":
        #   * Don't steer unless HCA is in state 3 "ready" or 5 "active"
        #   * Don't steer at standstill
        #   * Don't send > 3.00 Newton-meters torque
        #   * Don't send the same torque for > 6 seconds
        #   * Don't send uninterrupted steering for > 360 seconds
        # MQB racks reset the uninterrupted steering timer after a single frame
        # of HCA disabled; this is done whenever output happens to be zero.

        if CC.latActive:
          new_torque = int(round(actuators.torque * self.CCP.STEER_MAX))
          apply_torque = apply_driver_steer_torque_limits(new_torque, self.apply_torque_last, CS.out.steeringTorque, self.CCP)
          self.hca_frame_timer_running += self.CCP.STEER_STEP
          if self.apply_torque_last == apply_torque:
            self.hca_frame_same_torque += self.CCP.STEER_STEP
            if self.hca_frame_same_torque > self.CCP.STEER_TIME_STUCK_TORQUE / DT_CTRL:
              apply_torque -= (1, -1)[apply_torque < 0]
              self.hca_frame_same_torque = 0
          else:
            self.hca_frame_same_torque = 0
          hca_enabled = abs(apply_torque) > 0
        else:
          hca_enabled = False
          apply_torque = 0

        if not hca_enabled:
          self.hca_frame_timer_running = 0

        self.eps_timer_soft_disable_alert = self.hca_frame_timer_running > self.CCP.STEER_TIME_ALERT / DT_CTRL
        self.apply_torque_last = apply_torque
        can_sends.append(self.CCS.create_steering_control(self.packer_pt, CANBUS.pt, apply_torque, hca_enabled))

        if self.CP.flags & VolkswagenFlags.STOCK_HCA_PRESENT:
          # Pacify VW Emergency Assist driver inactivity detection by changing its view of driver steering input torque
          # to the greatest of actual driver input or 2x openpilot's output (1x openpilot output is not enough to
          # consistently reset inactivity detection on straight level roads). See commaai/openpilot#23274 for background.
          ea_simulated_torque = float(np.clip(apply_torque * 2, -self.CCP.STEER_MAX, self.CCP.STEER_MAX))
          if abs(CS.out.steeringTorque) > abs(ea_simulated_torque):
            ea_simulated_torque = CS.out.steeringTorque
          can_sends.append(self.CCS.create_eps_update(self.packer_pt, CANBUS.cam, CS.eps_stock_values, ea_simulated_torque))

    # **** Capacitive steering wheel touch (Emergency Assist hands-on pacification) ******* #

    if self.CP.flags & VolkswagenFlags.STOCK_KLR_PRESENT and CS.klr_stock_values:
      # Report "hands on" (when lat active) so the car doesn't escalate EA (brake jolt / stop).
      if CS.klr_stock_values["COUNTER"] != self.klr_counter_last:
        can_sends.append(self.CCS.create_capacitive_wheel_touch(self.packer_pt, CANBUS.cam, CC.latActive, CS.klr_stock_values))
        can_sends.append(self.CCS.create_capacitive_wheel_touch(self.packer_pt, CANBUS.pt, CC.latActive, CS.klr_stock_values))
      self.klr_counter_last = CS.klr_stock_values["COUNTER"]

    # **** Emergency Assist HUD relay (steering-wheel / hands-off icon) ****** #

    if self.CP.flags & VolkswagenFlags.STOCK_EA_PRESENT and CS.ea_hud_stock_values and self.frame % 2 == 0:
      # 순정 깜빡이가 이미 점등 중이면 OP 깜빡이 중계를 끔 (중복 방지, infiniteCable2 방식)
      blinker_active = CS.left_blinker_active or CS.right_blinker_active
      left_blinker = CC.leftBlinker if not blinker_active else False
      right_blinker = CC.rightBlinker if not blinker_active else False
      # NOTE(divergence): infiniteCable2는 hide_error=self.hide_ea_error(레이더 비활성 시에만 True)이라
      # 게이트웨이+레이더 정상 구성에선 EA 오류를 숨기지 않음. 우리는 주행 중 LKAS 오류 증상 때문에
      # CC.latActive로 숨김 -> KLR 핸즈온이 EA 오류를 막아주는지 실차 확인 후 원본대로 되돌릴지 결정.
      hide_error = CC.latActive
      can_sends.append(self.CCS.create_blinker_control(self.packer_pt, CANBUS.pt, CS.ea_hud_stock_values,
                                                       CS.ea_control_stock_values, left_blinker, right_blinker, hide_error))

    # **** Acceleration Controls ******************************************** #

    if self.CP.openpilotLongitudinalControl:
      if self.frame % self.CCP.ACC_CONTROL_STEP == 0:
        if self.CP.flags & VolkswagenFlags.MEB:
          stopping = actuators.longControlState == LongCtrlState.stopping
          accel = float(np.clip(actuators.accel, self.CCP.ACCEL_MIN, self.CCP.ACCEL_MAX) if CC.enabled else 0)
          # Creeping to a stop behind a lead keeps the planner in "pid" instead of "stopping".
          # If the car then latches its own ESP hold, ACC_Anfahren is never sent with the
          # (lcs == starting) condition alone and the car stays stuck in hold even after the
          # lead pulls away. Request a start whenever openpilot wants to accelerate out of hold.
          starting_planner = actuators.longControlState == LongCtrlState.starting and CS.out.vEgo <= self.CP.vEgoStarting
          # Only begin the release above 0.2 m/s^2 so tiny accel values can't unlatch the hold
          # and let the car roll back on a hill (normal launch uses startAccel 0.8, no delay).
          begin_release = (actuators.longControlState == LongCtrlState.pid
                           and CS.esp_hold_confirmation and accel >= 0.2)
          # The car refuses longitudinal control right after the driver brakes hard at low speed
          # (VMM_02.Long_Control_Inhibit); requesting a drive-off there faults TSK (commaai/opendbc).
          if CS.long_control_inhibit:
            begin_release = False

          # Hold the request until the car is actually moving. Do NOT keep esp_hold_confirmation
          # in the sustain condition: it goes false the moment the car acknowledges the release,
          # which cancels our own request mid-handshake. The car then falls back to the EPB and
          # escalates to P (measured: parkingBrake 0.06s after hold released, gear=park 0.8s later).
          if begin_release:
            self.hold_release_frames = self.CCP.HOLD_RELEASE_MAX_STEPS
          elif self.hold_release_frames > 0:
            self.hold_release_frames -= 1
          if not CC.enabled or stopping or accel <= 0.0 or CS.out.vEgo > self.CCP.HOLD_RELEASE_DONE_SPEED:
            self.hold_release_frames = 0
          starting = (starting_planner or self.hold_release_frames > 0) and not CS.long_control_inhibit

          # override / disable ramp handling to avoid EPB error at low speed (infiniteCable2)
          long_override = CC.cruiseControl.override or CS.out.gasPressed
          self.long_override_counter = min(self.long_override_counter + 1, 5) if long_override else 0
          long_override_begin = long_override and self.long_override_counter < 5
          self.long_disabled_counter = min(self.long_disabled_counter + 1, 5) if not CC.enabled else 0
          long_disabling = not CC.enabled and self.long_disabled_counter < 5

          acc_control = self.CCS.acc_control_value(CS.out.cruiseState.available, CS.out.accFaulted, CC.enabled, long_override)
          # HALTEN(1)/ANFAHREN(4) -> KEINE_ANFORDERUNG(0) 직행은 차가 P 로 폴트난다 (commaai/opendbc).
          # 직전에 홀드 계열을 보냈고 아직 HOLD_RELEASE_SPEED 미만이면 LOESEN_UEBER_RAMPE(5)를 거쳐 나간다.
          releasing = (self.acc_hold_type_last in (self.CCS.ACC_HMS_HOLD, self.CCS.ACC_HMS_RELEASE,
                                                   self.CCS.ACC_HMS_RAMP_RELEASE)
                       and CS.out.vEgo < self.CCP.HOLD_RELEASE_SPEED)
          acc_hold = self.CCS.acc_hold_type(CS.out.cruiseState.available, CS.out.accFaulted, CC.enabled, starting, stopping,
                                            CS.esp_hold_confirmation, long_override, long_override_begin, long_disabling,
                                            releasing)
          self.acc_hold_type_last = acc_hold
          # jerk/제어한계는 infiniteCable2 기본값(comfort OFF: jerk 4.0, 한계 0) 고정 - 실차 검증값.
          # (if2의 동적 comfort 모드는 출발 가속이 느려져 미채택; 필요 시 if2 mebutils에서 재이식)
          can_sends.extend(self.CCS.create_acc_accel_control(self.packer_pt, CANBUS.pt, self.CP, CS.acc_type, CC.enabled,
                                                             4.0, 4.0, 0., 0.,
                                                             accel, acc_control, acc_hold, stopping, starting, CS.esp_hold_confirmation,
                                                             CS.out.vEgoRaw * CV.MS_TO_KPH, long_override, CS.travel_assist_available))
        else:
          # MQB longitudinal (original carrot path, mqbcan signatures)
          acc_control = self.CCS.acc_control_value(CS.out.cruiseState.available, CS.out.accFaulted, CC.longActive)
          accel = float(np.clip(actuators.accel, self.CCP.ACCEL_MIN, self.CCP.ACCEL_MAX) if CC.longActive else 0)
          stopping = actuators.longControlState == LongCtrlState.stopping
          starting = actuators.longControlState == LongCtrlState.pid and (CS.esp_hold_confirmation or CS.out.vEgo < self.CP.vEgoStopping)
          can_sends.extend(self.CCS.create_acc_accel_control(self.packer_pt, CANBUS.pt, CS.acc_type, CC.longActive, accel,
                                                             acc_control, stopping, starting, CS.esp_hold_confirmation))

      #if self.aeb_available:
      #  if self.frame % self.CCP.AEB_CONTROL_STEP == 0:
      #    can_sends.append(self.CCS.create_aeb_control(self.packer_pt, False, False, 0.0))
      #  if self.frame % self.CCP.AEB_HUD_STEP == 0:
      #    can_sends.append(self.CCS.create_aeb_hud(self.packer_pt, False, False))

    # **** HUD Controls ***************************************************** #

    if self.frame % self.CCP.LDW_STEP == 0:
      hud_alert = 0
      if hud_control.visualAlert in (VisualAlert.steerRequired, VisualAlert.ldw):
        hud_alert = self.CCP.LDW_MESSAGES["laneAssistTakeOver"]
      if self.CP.flags & VolkswagenFlags.MEB:
        # MEB: create_lka_hud_control은 sound_alert 인자 추가 필요
        can_sends.append(self.CCS.create_lka_hud_control(self.packer_pt, CANBUS.pt, CS.ldw_stock_values, CC.latActive,
                                                         CS.out.steeringPressed, hud_alert, hud_control, 0))
      else:
        can_sends.append(self.CCS.create_lka_hud_control(self.packer_pt, CANBUS.pt, CS.ldw_stock_values, CC.latActive,
                                                         CS.out.steeringPressed, hud_alert, hud_control))

    if self.frame % self.CCP.ACC_HUD_STEP == 0 and self.CP.openpilotLongitudinalControl:
      if self.CP.flags & VolkswagenFlags.MEB:
        long_override = CC.cruiseControl.override or CS.out.gasPressed
        # MEB 계기판 충돌경고(빨간 앞차 심볼 + 비프 + "Break!") 상시 숨김 - ID.4 선택 시 자동 적용.
        # 출발/따라잡기 오탐 삐 소리 문제로 계기판 경고는 끄고, openpilot 화면 자체 FCW 경고는 유지.
        # 순정 레이더 AEB의 실제 개입/경고 체계는 별개 경로라 영향 없음. (tjddyd0130/opendbc a6869ce DisableClusterFcw 상시화)
        fcw_alert = False
        acc_hud_status = self.CCS.acc_hud_status_value(CS.out.cruiseState.available, CS.out.accFaulted, CC.enabled, long_override)
        set_speed = hud_control.setSpeed * CV.MS_TO_KPH
        distance = max(8, hud_control.leadDistance) if hud_control.leadDistance != 0 else 0
        desired_gap = max(8, CS.out.vEgo * 1.45)  # carrot HUDControl lacks leadFollowTime -> default time gap

        # 계기판 내비(TMAP) 표시 - ID.4 선택 시 자동 적용 (carrot SDI/ATC 데이터 -> hudControl 경유).
        # ACC_Events 실차 스캔 확정 지도(ID.4 MK1): 3 정차대기 / 4 표지판"전방" / 5 표지판"인식됨"
        # 6 커브(S자) / 7~8 커브(방향) / 9 교차로 / 10 로터리 / 11 출구 / 12 정체 / 13 병목 / 14 내리막 / 15 오르막
        acc_event = self.CCS.acc_hud_event(acc_hud_status, CS.esp_hold_confirmation, False, 0, 0)  # 3(정차대기) 또는 0
        speed_limit = 0.
        event_speed_kph = 0
        if acc_hud_status in (3, 4):  # ACTIVE / OVERRIDE 에서만 계기판 이벤트 표시
          if hud_control.naviSpeedLimit > 0:  # 단속카메라/구간단속 제한속도
            speed_limit = hud_control.naviSpeedLimit * CV.KPH_TO_MS
            acc_event = 4  # 표지판 + "전방" (앞에 있는 카메라에 대한 예고 - 아이콘 없이 유지)
          elif hud_control.naviEventType == 1 and hud_control.naviEventSpeed != 0:  # 커브 (부호=방향)
            # 실차 확인: 7=우커브, 8=좌커브. vTurnSpeed 양수=우, 음수=좌 (실주행 검증으로 확정).
            acc_event = 7 if hud_control.naviEventSpeed > 0 else 8
            event_speed_kph = abs(hud_control.naviEventSpeed)
          elif hud_control.naviEventType == 2 and hud_control.naviEventSpeed > 0:  # 교차로 좌/우회전
            acc_event = 9
            event_speed_kph = hud_control.naviEventSpeed
          elif hud_control.naviEventType == 3 and hud_control.naviEventSpeed > 0:  # 분기/고속도로 출구
            acc_event = 11
            event_speed_kph = hud_control.naviEventSpeed
          elif hud_control.naviEventType == 4:  # 로터리
            acc_event = 10
            event_speed_kph = hud_control.naviEventSpeed
          elif hud_control.naviEventType == 5:  # 병목 구간 (티맵 SDI)
            acc_event = 13

        # 배너 -> 미니 아이콘 전환: 내비 이벤트(6~15)는 새로 뜰 때만 큰 배너 ~1.2초, 이후 미니 아이콘으로 유지.
        # Lead_Brightness N = 이벤트 N+1과 같은 그림 (실차 스캔). 카메라(4/5)/정차(3)는 기존처럼 배너 유지.
        status_icon = 0
        if acc_event >= 6:
          if acc_event != self.navi_event_last:
            self.navi_banner_frames = 20  # ACC_HUD_STEP 6 (약 16Hz) x 20 = 약 1.2초
          self.navi_event_last = acc_event
          if self.navi_banner_frames > 0:
            self.navi_banner_frames -= 1
          else:
            # 미니 아이콘 = 이벤트-1 (실차 스캔). 단 커브 방향 그림은 배너와 좌우가 거울상:
            # 배너 7=우/8=좌 <-> 아이콘 7=우/6=좌 (실주행 검증) -> 커브만 특례 매핑
            if acc_event == 7:      # 우커브
              status_icon = 7
            elif acc_event == 8:    # 좌커브
              status_icon = 6
            else:
              status_icon = acc_event - 1
            acc_event = 0
            event_speed_kph = 0
        else:
          self.navi_event_last = 0
          self.navi_banner_frames = 0
          # 앞차 지배 상태 디바운스: 정체에서 lead<->도로제한 지배가 수초마다 왕복하며
          # 하이라이트/아이콘이 깜빡이는 것 방지. 약 2초(32프레임@16Hz) 유지돼야 전환.
          if hud_control.leadLimiting != self.lead_limit_disp:
            self.lead_limit_cnt += 1
            if self.lead_limit_cnt >= 32:
              self.lead_limit_disp = hud_control.leadLimiting
              self.lead_limit_cnt = 0
          else:
            self.lead_limit_cnt = 0
          # 도로제한속도 제어 중: 제한값이 "실제로 바뀔 때만" 배너 ~1.2초, 이후 km/h 아이콘.
          # road_limit_last는 지배자 변동/정차 등으로 잠깐 끊겨도 유지 -> 같은 제한값 배너 재생 방지.
          if acc_event == 0 and acc_hud_status in (3, 4) and hud_control.naviEventType == 8:
            road_limit_kph = abs(hud_control.naviEventSpeed)
            if road_limit_kph != self.road_limit_last:
              self.road_banner_frames = 20
            self.road_limit_last = road_limit_kph
            if self.road_banner_frames > 0 and road_limit_kph > 0:
              self.road_banner_frames -= 1
              acc_event = 5  # 표지판 + "인식됨"
              speed_limit = road_limit_kph * CV.KPH_TO_MS
            elif not self.lead_limit_disp:
              # Lead_Brightness는 단일 필드라 아이콘과 앞차 하이라이트를 동시에 못 씀.
              # 색 = 현재 속도의 지배자: 앞차가 속도를 제한 중이면 앞차 흰색 우선(디바운스 적용),
              # 그 외(앞차가 없거나 멀어서 도로제한이 지배)에는 km/h 뱃지 점등.
              status_icon = 4
          else:
            self.road_banner_frames = 0

        can_sends.append(self.CCS.create_acc_hud_control(self.packer_pt, CANBUS.pt, acc_hud_status, set_speed,
                                                         hud_control.leadVisible, hud_control.leadDistanceBars, True,
                                                         CS.esp_hold_confirmation, distance, desired_gap, fcw_alert, acc_event,
                                                         speed_limit, event_speed_kph, status_icon=status_icon))
      else:
        # MQB ACC HUD (original carrot path, mqbcan signatures)
        lead_distance = 0
        if hud_control.leadVisible and self.frame * DT_CTRL > 1.0:
          lead_distance = 512 if CS.upscale_lead_car_signal else 8
        acc_hud_status = self.CCS.acc_hud_status_value(CS.out.cruiseState.available, CS.out.accFaulted, CC.longActive)
        set_speed = hud_control.setSpeed * CV.MS_TO_KPH
        can_sends.append(self.CCS.create_acc_hud_control(self.packer_pt, CANBUS.pt, acc_hud_status, set_speed,
                                                         lead_distance, hud_control.leadDistanceBars))

    # **** Stock ACC Button Controls **************************************** #

    gra_send_ready = self.CP.pcmCruise and CS.gra_stock_values["COUNTER"] != self.gra_acc_counter_last
    if gra_send_ready and (CC.cruiseControl.cancel or CC.cruiseControl.resume):
      can_sends.append(self.CCS.create_acc_buttons_control(self.packer_pt, self.ext_bus, CS.gra_stock_values,
                                                           cancel=CC.cruiseControl.cancel, resume=CC.cruiseControl.resume))

    new_actuators = actuators.as_builder()
    if self.CP.flags & VolkswagenFlags.MEB:
      new_actuators.curvature = self.apply_curvature_last
    else:
      new_actuators.torque = self.apply_torque_last / self.CCP.STEER_MAX
      new_actuators.torqueOutputCan = self.apply_torque_last

    self.gra_acc_counter_last = CS.gra_stock_values["COUNTER"]
    self.frame += 1
    return new_actuators, can_sends
