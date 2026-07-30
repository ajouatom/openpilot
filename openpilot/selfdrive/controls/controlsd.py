#!/usr/bin/env python3
import math
import time
from numbers import Number

from openpilot.cereal import car, log
import openpilot.cereal.messaging as messaging
from openpilot.common.constants import CV
from openpilot.common.params import Params
from openpilot.common.pid import MultiplicativeUnwindPID
from openpilot.common.realtime import config_realtime_process, Priority, Ratekeeper
from openpilot.common.swaglog import cloudlog
import numpy as np
from collections import deque

from opendbc.car.car_helpers import interfaces
from opendbc.car.vehicle_model import VehicleModel
from opendbc.car.volkswagen.values import MEB_CURVATURE_PID_KP, MEB_CURVATURE_PID_KI, MEB_CURVATURE_PID_KF, MEB_CURVATURE_MAX

from openpilot.selfdrive.controls.lib.drive_helpers import clip_curvature, get_lag_adjusted_curvature, is_volkswagen_meb
from openpilot.selfdrive.controls.lib.latcontrol import LatControl, MIN_LATERAL_CONTROL_SPEED
from openpilot.selfdrive.controls.lib.latcontrol_pid import LatControlPID
from openpilot.selfdrive.controls.lib.latcontrol_angle import LatControlAngle, STEER_ANGLE_SATURATION_THRESHOLD
from openpilot.selfdrive.controls.lib.latcontrol_torque import LatControlTorque
from openpilot.selfdrive.controls.lib.longcontrol import LongControl
from openpilot.selfdrive.controls.lib.steer_ratio import resolve_vehicle_model_steer_ratio


from openpilot.common.realtime import DT_CTRL, DT_MDL
from openpilot.selfdrive.modeld.constants import ModelConstants
from openpilot.selfdrive.controls.lib.drive_helpers import CONTROL_N
from openpilot.selfdrive.modeld.modeld import LAT_SMOOTH_SECONDS
from openpilot.selfdrive.locationd.helpers import PoseCalibrator, Pose

from openpilot.selfdrive.carrot.carrot_controls import CarrotControls

State = log.SelfdriveState.OpenpilotState
LaneChangeState = log.LaneChangeState
LaneChangeDirection = log.LaneChangeDirection
LAT_CURVATURE_SATURATION_ACCEL = 0.1  # infiniteCable2 LatControlCurvature: 곡률 기반 steer_limited 임계 (m/s^2 환산)

ACTUATOR_FIELDS = tuple(car.CarControl.Actuators.schema.fields.keys())


class Controls:
  def __init__(self) -> None:
    self.params = Params()
    cloudlog.info("controlsd is waiting for CarParams")
    self.CP = messaging.log_from_bytes(self.params.get("CarParams", block=True), car.CarParams)
    cloudlog.info("controlsd got CarParams")

    self.CI = interfaces[self.CP.carFingerprint](self.CP)

    self.disable_dm = False

    self.sm = messaging.SubMaster(['liveDelay', 'liveParameters', 'liveTorqueParameters', 'modelV2', 'selfdriveState',
                                   'liveCalibration', 'livePose', 'longitudinalPlan', 'carState', 'carOutput',
                                   'carrotMan', 'lateralPlan', 'radarState',
                                   'driverMonitoringState', 'onroadEvents', 'driverAssistance'], poll='selfdriveState')
    self.pm = messaging.PubMaster(['carControl', 'controlsState'])

    self.steer_limited_by_safety = False
    self.curvature = 0.0
    self.desired_curvature = 0.0

    # VW MEB(ID.4/ID.5)에서만 사용. infiniteCable2 LatControlCurvature 정확 복제:
    # EnableCurvatureController=1(기본 ON) 상태의 곡률 폐루프 PID + useCarSteerCurvature 보정
    # (id4-meb 브랜치 실차 검증판). 게인은 opendbc values.py의 MEB_CURVATURE_PID_*가 단일 소스.
    self.is_vw_meb = is_volkswagen_meb(self.CP)
    self.meb_curvature_pid = (MultiplicativeUnwindPID(MEB_CURVATURE_PID_KP, MEB_CURVATURE_PID_KI,
                                                      k_f=MEB_CURVATURE_PID_KF,
                                                      pos_limit=MEB_CURVATURE_MAX, neg_limit=-MEB_CURVATURE_MAX)
                              if self.is_vw_meb else None)
    self.atc_turn_speed = self.params.get_int("AutoTurnControlSpeedTurn")

    self.pose_calibrator = PoseCalibrator()
    self.calibrated_pose: Pose | None = None
    
    self.side_state = {
        "left":  {"main": {"dRel": None, "lat": None}, "sub": {"dRel": None, "lat": None}},
        "right": {"main": {"dRel": None, "lat": None}, "sub": {"dRel": None, "lat": None}},
    }

    self.LoC = LongControl(self.CP)
    self.VM = VehicleModel(self.CP)
    self.LaC: LatControl
    if self.CP.steerControlType == car.CarParams.SteerControlType.angle:
      self.LaC = LatControlAngle(self.CP, self.CI)
    elif self.CP.lateralTuning.which() == 'pid':
      self.LaC = LatControlPID(self.CP, self.CI)
    elif self.CP.lateralTuning.which() == 'torque':
      self.LaC = LatControlTorque(self.CP, self.CI)
    self.carrot_controls = CarrotControls(self.CP)

  def update(self):
    self.sm.update(15)
    if self.sm.updated["liveCalibration"]:
      self.pose_calibrator.feed_live_calib(self.sm['liveCalibration'])
    if self.sm.updated["livePose"]:
      device_pose = Pose.from_live_pose(self.sm['livePose'])
      self.calibrated_pose = self.pose_calibrator.build_calibrated_pose(device_pose)

  def state_control(self):
    CS = self.sm['carState']

    # Update VehicleModel
    lp = self.sm['liveParameters']
    x = max(lp.stiffnessFactor, 0.1)
    # VW MEB uses the learned ratio directly. Other platforms may scale it or
    # override it, but legacy/out-of-range persisted rates must never collapse
    # the vehicle-model ratio and destabilize lateral feedback.
    sr = resolve_vehicle_model_steer_ratio(lp.steerRatio,
                                           self.params.get_float("SteerRatioRate"),
                                           self.params.get_float("CustomSR"),
                                           self.is_vw_meb)
    self.VM.update_params(x, sr)

    steer_angle_without_offset = math.radians(CS.steeringAngleDeg - lp.angleOffsetDeg)
    self.curvature = -self.VM.calc_curvature(steer_angle_without_offset, CS.vEgo, lp.roll)

    # Update Torque Params
    if self.CP.lateralTuning.which() == 'torque':
      torque_params = self.sm['liveTorqueParameters']
      if self.sm.all_checks(['liveTorqueParameters']) and torque_params.useParams:
        self.LaC.update_live_torque_params(torque_params.latAccelFactorFiltered, torque_params.latAccelOffsetFiltered,
                                           torque_params.frictionCoefficientFiltered)

    long_plan = self.sm['longitudinalPlan']
    model_v2 = self.sm['modelV2']

    CC = car.CarControl.new_message()
    CC.enabled = self.sm['selfdriveState'].enabled

    # carrot
    gear = car.CarState.GearShifter
    driving_gear = CS.gearShifter not in (gear.neutral, gear.park, gear.reverse, gear.unknown)
    lateral_enabled = driving_gear and self.params.get_bool("AlwaysLateral")
    #self.soft_hold_active = CS.softHoldActive #car.OnroadEvent.EventName.softHold in [e.name for e in self.sm['onroadEvents']]

    # Check which actuators can be enabled
    standstill = abs(CS.vEgo) <= max(self.CP.minSteerSpeed, MIN_LATERAL_CONTROL_SPEED) or CS.standstill
    CC.latActive = ((self.sm['selfdriveState'].active or lateral_enabled) and CS.latEnabled and
                    not CS.steerFaultTemporary and not CS.steerFaultPermanent and not standstill)
    CC.latActive = self.carrot_controls.lat_suspend_control(CS, CC.latActive)
    CC.longActive = CC.enabled and not any(e.overrideLongitudinal for e in self.sm['onroadEvents']) and self.CP.openpilotLongitudinalControl

    actuators = CC.actuators
    actuators.longControlState = self.LoC.long_control_state

    # Enable blinkers while lane changing
    if model_v2.meta.laneChangeState != LaneChangeState.off:
      CC.leftBlinker = model_v2.meta.laneChangeDirection == LaneChangeDirection.left
      CC.rightBlinker = model_v2.meta.laneChangeDirection == LaneChangeDirection.right

    if not CC.latActive:
      self.LaC.reset()
    if not CC.longActive:
      self.LoC.reset()

    # accel PID loop
    pid_accel_limits = self.CI.get_pid_accel_limits(self.CP, CS.vEgo, CS.vCruise * CV.KPH_TO_MS)
    t_since_plan = (self.sm.frame - self.sm.recv_frame['longitudinalPlan']) * DT_CTRL
    accel, aTarget, jerk = self.LoC.update(CC.longActive, CS, long_plan, pid_accel_limits, t_since_plan, self.sm['radarState'])
    actuators.accel = float(accel)
    actuators.aTarget = float(aTarget)
    actuators.jerk = float(jerk)

    # Steering PID loop and lateral MPC
    lat_plan = self.sm['lateralPlan']
    curve_speed_abs = abs(self.sm['carrotMan'].vTurnSpeed)
    self.lanefull_mode_enabled = (lat_plan.useLaneLines and curve_speed_abs > self.params.get_int("UseLaneLineCurveSpeed"))
    lat_smooth_seconds = self.params.get_float("LatSmoothSec") * 0.01
    steer_actuator_delay = self.params.get_float("SteerActuatorDelay") * 0.01
    if steer_actuator_delay == 0.0:
      steer_actuator_delay = self.sm['liveDelay'].lateralDelay 
    
    def smooth_value(val, prev_val, tau):
      alpha = 1 - np.exp(-DT_CTRL / tau) if tau > 0 else 1
      return alpha * val + (1 - alpha) * prev_val

    if not CC.latActive:
      new_desired_curvature = self.curvature
    elif self.is_vw_meb:
      # VW MEB(ID.4/ID.5): 기본은 레인리스(raw 모델곡률 = infiniteCable2 동일).
      # carrot 횡플래너가 레인모드 활성(lat_plan.useLaneLines, UseLaneLineSpeed>0 & 차선감지)일 때만
      # 차선기반 lateralPlan 경로를 쓴다(opt-in). 모델 경로 출렁임(차선넘나듦)을 차선 지오메트리로 보완.
      if getattr(lat_plan, 'useLaneLines', False) and len(lat_plan.curvatures) > 0:
        curvature = get_lag_adjusted_curvature(self.CP, CS.vEgo, lat_plan.psis, lat_plan.curvatures,
                                               steer_actuator_delay + lat_smooth_seconds, lat_plan.distances)
        new_desired_curvature = smooth_value(curvature, self.desired_curvature, lat_smooth_seconds)
      else:
        new_desired_curvature = float(model_v2.action.desiredCurvature)  # raw 모델곡률 (if2 기본과 동일)
    elif self.lanefull_mode_enabled:
      if len(lat_plan.curvatures) == 0:
        new_desired_curvature = self.curvature
      else:
        curvature = get_lag_adjusted_curvature(self.CP, CS.vEgo, lat_plan.psis, lat_plan.curvatures, steer_actuator_delay + lat_smooth_seconds, lat_plan.distances)
        new_desired_curvature = smooth_value(curvature, self.desired_curvature, lat_smooth_seconds)
    else:      
      new_desired_curvature = smooth_value(model_v2.action.desiredCurvature, self.desired_curvature, 0.1)

    self.desired_curvature, curvature_limited = clip_curvature(CS.vEgo, self.desired_curvature, new_desired_curvature, lp.roll)

    actuators.curvature = float(self.desired_curvature)

    # VW MEB 폐루프 곡률 보정 = infiniteCable2 LatControlCurvature.update() 정확 복제.
    # carrot엔 곡률 전용 횡제어기가 없어 모델 목표곡률을 open-loop로 보내면 EPS가 명령만큼 안 꺾여
    # 차선 쏠림 발생. infiniteCable2 기본설정(EnableCurvatureController=1 -> PID ON,
    # EnableCurvatureD=0 -> curvatured/liveCurvatureParameters 미사용, useCarSteerCurvature=True)을 그대로:
    #   output = pid(error, feedforward) + (차량실측곡률 - VM모델곡률_롤제외)
    #   feedforward = 목표곡률 - 롤보정,  error = 목표곡률 - 실측곡률(VM+pose 블렌딩)
    # PID는 MultiplicativeUnwindPID(게인 infiniteCable2 동일). steeringPressed 시 적분 unwind.
    # (brand==volkswagen & steerControlType==angle == MEB 고유 조건). 최종 rate/크기 제한은 carcontroller.
    if self.is_vw_meb:
      if not CC.latActive:
        self.meb_curvature_pid.reset()
      else:
        roll_compensation = -self.VM.roll_compensation(lp.roll, CS.vEgo)
        actual_curvature_vm_no_roll = -self.VM.calc_curvature(steer_angle_without_offset, CS.vEgo, 0.)
        actual_curvature_vm = actual_curvature_vm_no_roll - roll_compensation
        actual_curvature = actual_curvature_vm
        if self.calibrated_pose is not None and CS.vEgo > 5.0:
          actual_curvature_pose = self.calibrated_pose.angular_velocity.yaw / max(CS.vEgo, 0.1)
          actual_curvature = float(np.interp(CS.vEgo, [2.0, 5.0], [actual_curvature_vm, actual_curvature_pose]))
        feedforward = self.desired_curvature - roll_compensation
        error = self.desired_curvature - actual_curvature
        freeze_integrator = self.steer_limited_by_safety or CS.vEgo < 5 or CS.steeringPressed
        pid_curvature = self.meb_curvature_pid.update(error, speed=CS.vEgo, feedforward=feedforward,
                                                      freeze_integrator=freeze_integrator, override=CS.steeringPressed)
        output_curvature = pid_curvature + (CS.steeringCurvature - actual_curvature_vm_no_roll)  # useCarSteerCurvature
        # infiniteCable2와 동일: 여기선 clip 안 함. 실제 한계는 carcontroller apply_std_curvature_limits
        # (rate+평균+0.195) 와 panda(0.195)가 bound. 곡률은 물리적으로 bounded(~±0.2)라 NaN검사만으로 충분.
        actuators.curvature = float(output_curvature)

    # 주의: MEB는 위 곡률 PID가 실제 조향을 만들고, 아래 LaC(LatControlAngle)와 lac_log는
    # 파이프라인 형식 유지를 위한 더미임 (로그의 angleState는 실제 제어 상태가 아님).
    steer, steeringAngleDeg, lac_log = self.LaC.update(CC.latActive, CS, self.VM, lp,
                                                       self.steer_limited_by_safety, self.desired_curvature,
                                                       CC, curvature_limited,
                                                       model_data=self.sm['modelV2'])
    actuators.torque = float(steer)
    actuators.steeringAngleDeg = float(steeringAngleDeg)
    # Ensure no NaNs/Infs
    for p in ACTUATOR_FIELDS:
      attr = getattr(actuators, p)
      if not isinstance(attr, Number):
        continue

      if not math.isfinite(attr):
        cloudlog.error(f"actuators.{p} not finite {actuators.to_dict()}")
        setattr(actuators, p, 0.0)

    return CC, lac_log

  def publish(self, CC, lac_log):
    CS = self.sm['carState']

    # Orientation and angle rates can be useful for carcontroller
    # Only calibrated (car) frame is relevant for the carcontroller
    CC.currentCurvature = self.curvature
    if self.calibrated_pose is not None:
      CC.orientationNED = self.calibrated_pose.orientation.xyz.tolist()
      CC.angularVelocity = self.calibrated_pose.angular_velocity.xyz.tolist()

    #acceleration_value = list(self.sm['liveLocationKalman'].accelerationCalibrated.value)
    #if len(acceleration_value) > 2:
    #  if abs(acceleration_value[0]) > 16.0:
    #    print("Collision detected. disable openpilot, restart")
    #    self.params.put_bool("OpenpilotEnabledToggle", False)
    #    self.params.put_int("SoftRestartTriggered", 1)

    CC.cruiseControl.override = CC.enabled and not CC.longActive and self.CP.openpilotLongitudinalControl
    CC.cruiseControl.cancel = CS.cruiseState.enabled and (not CC.enabled or not self.CP.pcmCruise)

    desired_kph = min(CS.vCruiseCluster, self.sm['carrotMan'].desiredSpeed)
    setSpeed = float(desired_kph * CV.KPH_TO_MS)
    speeds = self.sm['longitudinalPlan'].speeds
    if len(speeds):
      CC.cruiseControl.resume = CC.enabled and CS.cruiseState.standstill and speeds[-1] > 0.1
      vCluRatio = CS.vCluRatio if CS.vCluRatio > 0.5 else 1.0
      setSpeed = speeds[-1] / vCluRatio

    hudControl = CC.hudControl

    hudControl.activeCarrot = self.sm['carrotMan'].activeCarrot
    hudControl.atcDistance = self.sm['carrotMan'].xDistToTurn

    # VW MEB(ID.4/ID.5) 계기판 내비 표시용 - ID.4 선택 시 자동 (토글 없음).
    # carrot_serv의 SDI/ATC/커브 데이터를 hudControl로 전달하고, VW carcontroller가
    # MEB_ACC_01(ACC_19)의 ACC_Tempolimit/ACC_Events/ACC_Event_Wunschgeschw로 변환한다.
    # (tjddyd0130/opendbc meb-cluster-tmap 검증 표시 체계: 카메라 > 커브 > 교차로)
    if self.is_vw_meb:
      carrot_man = self.sm['carrotMan']
      # 단속카메라/구간단속 (방지턱 22는 계기판 미표시 - 제한속도 표지로 오인됨)
      navi_speed_limit = 0
      if carrot_man.xSpdType >= 0 and carrot_man.xSpdType != 22 and carrot_man.xSpdLimit > 0:
        navi_speed_limit = int(carrot_man.xSpdLimit)
      hudControl.naviSpeedLimit = navi_speed_limit
      # 커브 / 교차로 / 분기·출구 / 로터리 / 병목 / 정체 (실차 스캔으로 확정된 이벤트 지도 기반)
      navi_event_type = 0
      navi_event_speed = 0
      v_ego_kph = CS.vEgo * CV.MS_TO_KPH
      vturn_kph = abs(carrot_man.vTurnSpeed)
      atc_type = carrot_man.atcType
      if self.sm.frame % 100 == 0:  # 1Hz로만 파라미터 IO (100Hz 루프 보호)
        self.atc_turn_speed = self.params.get_int("AutoTurnControlSpeedTurn")
      if 0 < vturn_kph < 120 and vturn_kph < v_ego_kph - 3:  # 커브 감속이 실제로 작동 중일 때만
        navi_event_type = 1
        navi_event_speed = int(carrot_man.vTurnSpeed)  # 부호 유지 (양수=우커브, 음수=좌커브 - 실주행 검증)
      elif atc_type in ("turn left", "turn right", "atc left", "atc right"):  # 교차로 좌/우회전 (prepare 제외)
        navi_event_type = 2
        navi_event_speed = int(self.atc_turn_speed)
      elif atc_type in ("fork left", "fork right") and carrot_man.nRoadLimitSpeed > 0:  # 분기/고속도로 출구
        navi_event_type = 3
        navi_event_speed = int(carrot_man.nRoadLimitSpeed)
      elif carrot_man.xTurnInfo == 5 and 0 < carrot_man.xDistToTurn < 500:  # 로터리 (TBT 131~142, 500m 이내)
        navi_event_type = 4
        navi_event_speed = int(self.atc_turn_speed)
      elif carrot_man.szSdiDescr in ("병목지점", "Bottleneck point", "瓶颈路段"):  # 티맵 SDI 50 (경보 범위 내 수신)
        navi_event_type = 5
      elif carrot_man.desiredSource == "road" and 0 < carrot_man.desiredSpeed < 200:
        navi_event_type = 8  # 도로제한속도 자동속도 제어 중 (제한값 변경 시 "인식됨" 배너 -> km/h 아이콘)
        navi_event_speed = int(carrot_man.nRoadLimitSpeed)
      # 앞차가 실제로 속도를 제한 중(MPC lead 모드)이면 계기판 앞차 하이라이트 우선 (km/h 아이콘 양보)
      hudControl.leadLimiting = bool(hudControl.leadVisible and self.sm['longitudinalPlan'].xState == 0)  # XState.lead
      hudControl.naviEventType = navi_event_type
      hudControl.naviEventSpeed = navi_event_speed

    lp = self.sm['longitudinalPlan']
    if self.CP.pcmCruise:
      speed_from_pcm = self.params.get_int("SpeedFromPCM")
      if speed_from_pcm == 1: #toyota
        hudControl.setSpeed = float(CS.vCruiseCluster * CV.KPH_TO_MS)
      elif speed_from_pcm == 2:
        hudControl.setSpeed = float(max(30/3.6, desired_kph * CV.KPH_TO_MS))
      elif speed_from_pcm == 3: # honda
        hudControl.setSpeed = setSpeed if lp.xState == 3 else float(desired_kph * CV.KPH_TO_MS)
      else:
        hudControl.setSpeed = float(max(30/3.6, setSpeed))
    else:
      hudControl.setSpeed = setSpeed if lp.xState == 3 else float(desired_kph * CV.KPH_TO_MS)
    hudControl.speedVisible = CC.enabled
    hudControl.lanesVisible = CC.enabled
    hudControl.leadVisible = self.sm['longitudinalPlan'].hasLead
    hudControl.leadDistanceBars = self.sm['selfdriveState'].personality.raw + 1
    hudControl.visualAlert = self.sm['selfdriveState'].alertHudVisual

    radarState = self.sm['radarState']
    leadOne = radarState.leadOne
    hudControl.leadDistance = leadOne.dRel if leadOne.status else 0
    hudControl.leadRelSpeed = leadOne.vRel if leadOne.status else 0
    hudControl.leadRadar = 1 if leadOne.radar else 0
    hudControl.leadDPath = leadOne.dPath

    meta = self.sm['modelV2'].meta
    if False: # command
      desire_map = {
        log.Desire.turnLeft: 1,
        log.Desire.turnRight: 2,
        log.Desire.laneChangeLeft: 3,
        log.Desire.laneChangeRight: 4,
      }
      hudControl.modelDesire = desire_map.get(meta.desire, 0)
    else: # model.
      hud_desire = 0
      if len(meta.desireState) > 4:
        if meta.desireState[1] > 0.1:
          hud_desire = 1   # turnLeft
        elif meta.desireState[2] > 0.1:
          hud_desire = 2   # turnRight
        elif meta.desireState[3] > 0.1:
          hud_desire = 3   # laneChangeLeft
        elif meta.desireState[4] > 0.1:
          hud_desire = 4   # laneChangeRight
      hudControl.modelDesire = hud_desire

    hudControl.rightLaneVisible = True
    hudControl.leftLaneVisible = True
    if self.sm.valid['driverAssistance']:
      hudControl.leftLaneDepart = self.sm['driverAssistance'].leftLaneDeparture
      hudControl.rightLaneDepart = self.sm['driverAssistance'].rightLaneDeparture

    if self.sm['selfdriveState'].active:
      CO = self.sm['carOutput']
      if self.is_vw_meb:
        # VW MEB: 곡률로 액추에이션하므로 곡률 기반으로 steer_limited 계산 (infiniteCable2 curvatureDEPRECATED 분기와 동일).
        # carrot 기본 angle 분기는 steeringAngleDeg(미사용 출력) 기준이라 우리 곡률 제한을 반영 못 함.
        self.steer_limited_by_safety = abs(CC.actuators.curvature - CO.actuatorsOutput.curvature) * CS.vEgo ** 2 > \
                                              LAT_CURVATURE_SATURATION_ACCEL
      elif self.CP.steerControlType == car.CarParams.SteerControlType.angle:
        self.steer_limited_by_safety = abs(CC.actuators.steeringAngleDeg - CO.actuatorsOutput.steeringAngleDeg) > \
                                              STEER_ANGLE_SATURATION_THRESHOLD
      else:
        self.steer_limited_by_safety = abs(CC.actuators.torque - CO.actuatorsOutput.torque) > 1e-2

    # TODO: both controlsState and carControl valids should be set by
    #       sm.all_checks(), but this creates a circular dependency

    # controlsState
    dat = messaging.new_message('controlsState')
    dat.valid = CS.canValid
    cs = dat.controlsState

    cs.curvature = self.curvature
    cs.longitudinalPlanMonoTime = self.sm.logMonoTime['longitudinalPlan']
    cs.lateralPlanMonoTime = self.sm.logMonoTime['modelV2']
    cs.desiredCurvature = self.desired_curvature
    cs.longControlState = self.LoC.long_control_state
    cs.upAccelCmd = float(self.LoC.pid.p)
    cs.uiAccelCmd = float(self.LoC.pid.i)
    cs.ufAccelCmd = float(self.LoC.pid.f)
    cs.forceDecel = False
    if self.params.get_int("DisableDM") == 0:
      cs.forceDecel = bool((self.sm['driverMonitoringState'].alertLevel == log.DriverMonitoringState.AlertLevel.three) or
                           (self.sm['selfdriveState'].state == State.softDisabling))


    lat_tuning = self.CP.lateralTuning.which()
    if self.CP.steerControlType == car.CarParams.SteerControlType.angle:
      cs.lateralControlState.angleState = lac_log
    elif lat_tuning == 'pid':
      cs.lateralControlState.pidState = lac_log
    elif lat_tuning == 'torque':
      cs.lateralControlState.torqueState = lac_log

    cs.activeLaneLine = self.lanefull_mode_enabled
    self.pm.send('controlsState', dat)

    # carControl
    cc_send = messaging.new_message('carControl')
    cc_send.valid = CS.canValid
    cc_send.carControl = CC
    self.pm.send('carControl', cc_send)

  def run(self):
    rk = Ratekeeper(100, print_delay_threshold=None)
    while True:
      self.update()
      CC, lac_log = self.state_control()
      self.publish(CC, lac_log)
      rk.monitor_time()


def main():
  config_realtime_process(4, Priority.CTRL_HIGH)
  controls = Controls()
  controls.run()


if __name__ == "__main__":
  main()
