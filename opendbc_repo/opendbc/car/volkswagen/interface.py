from opendbc.car import get_safety_config, structs
from opendbc.car.interfaces import CarInterfaceBase
from opendbc.car.volkswagen.carcontroller import CarController
from opendbc.car.volkswagen.carstate import CarState
from opendbc.car.volkswagen.radar_interface import RadarInterface
from opendbc.car.volkswagen.values import CAR, NetworkLocation, TransmissionType, VolkswagenFlags, VolkswagenSafetyFlags


class CarInterface(CarInterfaceBase):
  CarState = CarState
  CarController = CarController
  RadarInterface = RadarInterface

  @staticmethod
  def _get_params(ret: structs.CarParams, candidate: CAR, fingerprint, car_fw, alpha_long, is_release, docs) -> structs.CarParams:
    ret.brand = "volkswagen"
    ret.radarUnavailable = True

    if ret.flags & VolkswagenFlags.PQ:
      # Set global PQ35/PQ46/NMS parameters
      ret.safetyConfigs = [get_safety_config(structs.CarParams.SafetyModel.volkswagenPq)]
      ret.enableBsm = 0x3BA in fingerprint[0]  # SWA_1

      if 0x440 in fingerprint[0] or docs:  # Getriebe_1
        ret.transmissionType = TransmissionType.automatic
      else:
        ret.transmissionType = TransmissionType.manual

      if any(msg in fingerprint[1] for msg in (0x1A0, 0xC2)):  # Bremse_1, Lenkwinkel_1
        ret.networkLocation = NetworkLocation.gateway
      else:
        ret.networkLocation = NetworkLocation.fwdCamera

      # The PQ port is in dashcam-only mode due to a fixed six-minute maximum timer on HCA steering. An unsupported
      # EPS flash update to work around this timer, and enable steering down to zero, is available from:
      #   https://github.com/pd0wm/pq-flasher
      # It is documented in a four-part blog series:
      #   https://blog.willemmelching.nl/carhacking/2022/01/02/vw-part1/
      # Panda ALLOW_DEBUG firmware required.
      ret.dashcamOnly = True

    elif ret.flags & VolkswagenFlags.MEB:
      # Set global MEB parameters (ID.4, ID.5, etc.)
      ret.safetyConfigs = [get_safety_config(structs.CarParams.SafetyModel.volkswagenMeb)]
      if ret.flags & VolkswagenFlags.MEB_GEN2:
        # 2024+ (ID.4 MK2 등): 신형 CRC 변형 - panda safety에 플래그 전달
        ret.safetyConfigs[0].safetyParam |= VolkswagenSafetyFlags.ALT_CRC_VARIANT_1.value
      ret.enableBsm = 0x24C in fingerprint[0]  # MEB_Side_Assist_01
      ret.transmissionType = TransmissionType.direct
      # MEB is curvature-controlled (HCA_03). openpilot routes curvature cars through
      # LatControlAngle (steerControlType == angle); the carcontroller consumes
      # actuators.curvature directly. Using curvatureDEPRECATED instead falls through
      # to LatControlPID, which crashes on the empty MEB pid tune.
      ret.steerControlType = structs.CarParams.SteerControlType.angle
      ret.steerAtStandstill = True

      if 0x25D in fingerprint[0]:  # KLR_01 - capacitive steering wheel module (Emergency Assist hands-on)
        ret.flags |= VolkswagenFlags.STOCK_KLR_PRESENT.value

      if all(msg in fingerprint[2] for msg in (0x1A4, 0x1F0)):  # EA_01, EA_02 - Emergency Assist HUD
        ret.flags |= VolkswagenFlags.STOCK_EA_PRESENT.value

      if any(msg in fingerprint[1] for msg in (0x520, 0x86, 0xFD, 0x13D)):  # MEB gateway messages
        ret.networkLocation = NetworkLocation.gateway
      else:
        ret.networkLocation = NetworkLocation.fwdCamera

      # On the gateway harness the stock radar stays active; use it (radar+vision fusion) when present.
      # Strukturen_01 (0x24F) is the MEB radar object message. Camera harness keeps radar unavailable.
      # radarDelay(0.8) 보정과 함께 사용 (infiniteCable2와 동일). 보정 없으면 레이더 리드 타이밍이
      # 어긋나 조기제동/재출발 막힘이 났었음.
      if ret.networkLocation == NetworkLocation.gateway:
        ret.radarUnavailable = 0x24F not in fingerprint[0]

    else:
      # Set global MQB parameters
      ret.safetyConfigs = [get_safety_config(structs.CarParams.SafetyModel.volkswagen)]
      ret.enableBsm = 0x30F in fingerprint[0]  # SWA_01

      if 0xAD in fingerprint[0] or docs:  # Getriebe_11
        ret.transmissionType = TransmissionType.automatic
      elif 0x187 in fingerprint[0]:  # Motor_EV_01
        ret.transmissionType = TransmissionType.direct
      else:
        ret.transmissionType = TransmissionType.manual

      if any(msg in fingerprint[1] for msg in (0x40, 0x86, 0xB2, 0xFD)):  # Airbag_01, LWI_01, ESP_19, ESP_21
        ret.networkLocation = NetworkLocation.gateway
      else:
        ret.networkLocation = NetworkLocation.fwdCamera

      if 0x126 in fingerprint[2]:  # HCA_01
        ret.flags |= VolkswagenFlags.STOCK_HCA_PRESENT.value

    # Global lateral tuning defaults, can be overridden per-vehicle

    ret.steerLimitTimer = 0.4
    if ret.flags & VolkswagenFlags.PQ:
      ret.steerActuatorDelay = 0.2
      CarInterfaceBase.configure_torque_tune(candidate, ret.lateralTuning)
    elif ret.flags & VolkswagenFlags.MEB:
      ret.steerActuatorDelay = 0.3
    else:
      ret.steerActuatorDelay = 0.1
      ret.lateralTuning.pid.kpBP = [0.]
      ret.lateralTuning.pid.kiBP = [0.]
      ret.lateralTuning.pid.kf = 0.00006
      ret.lateralTuning.pid.kpV = [0.6]
      ret.lateralTuning.pid.kiV = [0.2]

    # Global longitudinal tuning defaults, can be overridden per-vehicle
    # MEB longitudinal is supported on the gateway harness (stock radar/AEB stay active, no radar-disable
    # needed). Camera-harness MEB long would require DISABLE_RADAR support (not ported), so keep it gateway-only.
    ret.alphaLongitudinalAvailable = ret.networkLocation == NetworkLocation.gateway or docs

    # MQB는 carrot 원본과 동일하게 alpha_long만으로 활성(카메라 하네스 MQB 롱컨 유지).
    # MEB만 게이트웨이 하네스 필요(카메라 하네스 MEB 롱컨은 레이더 무력화 미지원).
    if alpha_long and (ret.alphaLongitudinalAvailable or not (ret.flags & VolkswagenFlags.MEB)):
      ret.openpilotLongitudinalControl = True
      ret.safetyConfigs[0].safetyParam |= VolkswagenSafetyFlags.LONG_CONTROL.value
      if ret.transmissionType == TransmissionType.manual:
        ret.minEnableSpeed = 4.5

    ret.pcmCruise = not ret.openpilotLongitudinalControl
    ret.stopAccel = -0.55
    ret.vEgoStarting = 0.1
    ret.vEgoStopping = 0.5
    ret.autoResumeSng = ret.minEnableSpeed == -1

    if ret.flags & VolkswagenFlags.MEB:
      # MEB longitudinal: openpilot's "starting" long-control state is required so the carcontroller
      # sends ACC_Anfahren (launch) on re-launch; without it the planner skips to pid, ACC_Anfahren
      # never fires, and the car won't release the EPB hold (re-launch fails). Also a very slow launch
      # can fault the car (EPB shutdown), so keep vEgoStarting ~0.5 m/s.
      ret.startingState = True
      ret.startAccel = 0.8
      ret.vEgoStarting = 0.5
      ret.vEgoStopping = 0.1
      # infiniteCable2와 동일: 종방향 액추에이터 지연 + 레이더 측정 지연 보정.
      # radarDelay 누락 시 레이더 리드의 위치/속도가 어긋나 조기제동·리드 불안정 유발.
      ret.longitudinalActuatorDelay = 0.5
      ret.radarDelay = 0.8
      ret.longitudinalTuning.kiBP = [0., 30.]
      ret.longitudinalTuning.kiV = [0.4, 0.]

    return ret
