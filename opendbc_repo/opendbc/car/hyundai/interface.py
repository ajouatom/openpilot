from panda import Panda
from opendbc.car import get_safety_config, structs
from opendbc.car.hyundai.hyundaicanfd import CanBus
from opendbc.car.hyundai.values import HyundaiFlags, CAR, DBC, CAMERA_SCC_CAR, CANFD_RADAR_SCC_CAR, \
                                                   CANFD_UNSUPPORTED_LONGITUDINAL_CAR, \
                                                   UNSUPPORTED_LONGITUDINAL_CAR, HyundaiExtFlags
from opendbc.car.hyundai.radar_interface import RADAR_START_ADDR
from opendbc.car.interfaces import CarInterfaceBase
from opendbc.car.disable_ecu import disable_ecu

from openpilot.common.params import Params

ButtonType = structs.CarState.ButtonEvent.Type
Ecu = structs.CarParams.Ecu

# Cancel button can sometimes be ACC pause/resume button, main button can also enable on some cars
ENABLE_BUTTONS = (ButtonType.accelCruise, ButtonType.decelCruise, ButtonType.cancel, ButtonType.mainCruise)


class CarInterface(CarInterfaceBase):
  @staticmethod
  def _get_params(ret: structs.CarParams, candidate, fingerprint, car_fw, experimental_long, docs) -> structs.CarParams:

    params = Params()
    camera_scc = params.get_int("HyundaiCameraSCC")
    if camera_scc > 0:
      ret.flags |= HyundaiFlags.CAMERA_SCC.value
      print("$$$CAMERA_SCC toggled...")
    if camera_scc > 1:
      ret.extFlags |= HyundaiExtFlags.ACAN_PANDA.value
      print("$$$sub PANDA enabled...")
      


    ret.carName = "hyundai"

    cam_can = CanBus(None, fingerprint).CAM
    hda2 = 0x50 in fingerprint[cam_can] or 0x110 in fingerprint[cam_can]
    hda2 = hda2 or params.get_bool("CanfdHDA2")
    CAN = CanBus(None, fingerprint, hda2)

    if ret.flags & HyundaiFlags.CANFD:
      # Shared configuration for CAN-FD cars
      ret.experimentalLongitudinalAvailable = candidate not in (CANFD_UNSUPPORTED_LONGITUDINAL_CAR | CANFD_RADAR_SCC_CAR)
      ret.enableBsm = 0x1e5 in fingerprint[CAN.ECAN]

      if 0x105 in fingerprint[CAN.ECAN]:
        ret.flags |= HyundaiFlags.HYBRID.value

      # detect HDA2 with ADAS Driving ECU
      if hda2:
        ret.flags |= HyundaiFlags.CANFD_HDA2.value
        if camera_scc > 0:
          if 0x110 in fingerprint[CAN.ACAN]:
            ret.flags |= HyundaiFlags.CANFD_HDA2_ALT_STEERING.value
            print("$$$CANFD ALT_STEERING1")
        else:
          if 0x110 in fingerprint[CAN.CAM]: # 0x110(272): LKAS_ALT
            ret.flags |= HyundaiFlags.CANFD_HDA2_ALT_STEERING.value
            print("$$$CANFD ALT_STEERING1")
          ## carrot_todo: sorento: 
          if 0x2a4 not in fingerprint[CAN.CAM]: # 0x2a4(676): CAM_0x2a4
            ret.flags |= HyundaiFlags.CANFD_HDA2_ALT_STEERING.value
            print("$$$CANFD ALT_STEERING2")

        ## carrot: canival 4th, no 0x1cf
        if 0x1cf not in fingerprint[CAN.ECAN]: # 0x1cf(463): CRUISE_BUTTONS
          ret.flags |= HyundaiFlags.CANFD_ALT_BUTTONS.value
          print("$$$CANFD ALT_BUTTONS")
      else:
        # non-HDA2
        print("$$$CANFD non HDA2")
        if 0x1cf not in fingerprint[CAN.ECAN]:
          ret.flags |= HyundaiFlags.CANFD_ALT_BUTTONS.value
          print("$$$CANFD ALT_BUTTONS")
        if not ret.flags & HyundaiFlags.RADAR_SCC:
          ret.flags |= HyundaiFlags.CANFD_CAMERA_SCC.value
          print("$$$CANFD CAMERA_SCC")
      # Some HDA2 cars have alternative messages for gear checks
      # ICE cars do not have 0x130; GEARS message on 0x40 or 0x70 instead
      if 0x130 not in fingerprint[CAN.ECAN]: # 0x130(304): GEAR_SHIFTER
        if 0x40 not in fingerprint[CAN.ECAN]: # 0x40(64): GEAR_ALT
          if 112 not in fingerprint[CAN.ECAN]:  # carrot: eGV70
            if 69 in fingerprint[CAN.ECAN]:
              ret.extFlags |= HyundaiExtFlags.CANFD_GEARS_69.value
              print("$$$CANFD GEARS_69")
            else:
              ret.extFlags |= HyundaiExtFlags.CANFD_GEARS_NONE.value
              print("$$$CANFD GEARS_NONE")
          else:
            ret.flags |= HyundaiFlags.CANFD_ALT_GEARS_2.value
            print("$$$CANFD ALT_GEARS_2")
        else:
          ret.flags |= HyundaiFlags.CANFD_ALT_GEARS.value
          print("$$$CANFD ALT_GEARS")
      cfgs = [get_safety_config(structs.CarParams.SafetyModel.hyundaiCanfd), ]
      if CAN.ECAN >= 4:
        cfgs.insert(0, get_safety_config(structs.CarParams.SafetyModel.noOutput))
      ret.safetyConfigs = cfgs

      if ret.flags & HyundaiFlags.CANFD_HDA2:
        ret.safetyConfigs[-1].safetyParam |= Panda.FLAG_HYUNDAI_CANFD_HDA2
        if ret.flags & HyundaiFlags.CANFD_HDA2_ALT_STEERING:
          ret.safetyConfigs[-1].safetyParam |= Panda.FLAG_HYUNDAI_CANFD_HDA2_ALT_STEERING
      if ret.flags & HyundaiFlags.CANFD_ALT_BUTTONS:
        ret.safetyConfigs[-1].safetyParam |= Panda.FLAG_HYUNDAI_CANFD_ALT_BUTTONS
      if ret.flags & HyundaiFlags.CANFD_CAMERA_SCC:
        ret.safetyConfigs[-1].safetyParam |= Panda.FLAG_HYUNDAI_CAMERA_SCC

    else:
      # Shared configuration for non CAN-FD cars
      ret.experimentalLongitudinalAvailable = candidate not in (UNSUPPORTED_LONGITUDINAL_CAR | CAMERA_SCC_CAR)
      ret.enableBsm = 0x58b in fingerprint[0]

      # Send LFA message on cars with HDA
      if 0x485 in fingerprint[2]:
        ret.flags |= HyundaiFlags.SEND_LFA.value

      # These cars use the FCA11 message for the AEB and FCW signals, all others use SCC12
      if 0x38d in fingerprint[0] or 0x38d in fingerprint[2]:
        ret.flags |= HyundaiFlags.USE_FCA.value

      if ret.flags & HyundaiFlags.LEGACY:
        # these cars require a special panda safety mode due to missing counters and checksums in the messages
        ret.safetyConfigs = [get_safety_config(structs.CarParams.SafetyModel.hyundaiLegacy)]
      else:
        ret.safetyConfigs = [get_safety_config(structs.CarParams.SafetyModel.hyundai, 0)]

      if ret.flags & HyundaiFlags.CAMERA_SCC:
        ret.safetyConfigs[0].safetyParam |= Panda.FLAG_HYUNDAI_CAMERA_SCC
        print("$$$CAMERA_SCC")

      if 1290 in fingerprint[2]:
        ret.extFlags |= HyundaiExtFlags.HAS_SCC13.value
      if 905 in fingerprint[2]:
        ret.extFlags |= HyundaiExtFlags.HAS_SCC14.value

    # Common lateral control setup

    ret.centerToFront = ret.wheelbase * 0.4
    ret.steerActuatorDelay = 0.1
    ret.steerLimitTimer = 0.4
    CarInterfaceBase.configure_torque_tune(candidate, ret.lateralTuning)

    if ret.flags & HyundaiFlags.ALT_LIMITS:
      ret.safetyConfigs[-1].safetyParam |= Panda.FLAG_HYUNDAI_ALT_LIMITS

    # Common longitudinal control setup

    ret.radarUnavailable = RADAR_START_ADDR not in fingerprint[1] or DBC[ret.carFingerprint]["radar"] is None
    ret.openpilotLongitudinalControl = experimental_long and ret.experimentalLongitudinalAvailable

    # carrot, if camera_scc enabled, enable openpilotLongitudinalControl
    if ret.flags & HyundaiFlags.CAMERA_SCC.value:
      ret.radarUnavailable = False
      ret.openpilotLongitudinalControl = True

    ret.pcmCruise = not ret.openpilotLongitudinalControl
    ret.startingState = False # True  # carrot
    ret.vEgoStarting = 0.1
    ret.startAccel = 1.0
    ret.longitudinalActuatorDelay = 0.5

    ret.longitudinalTuning.kpBP = [0.]
    ret.longitudinalTuning.kpV = [1.]
    ret.longitudinalTuning.kf = 1.0

    # *** feature detection ***
    if ret.flags & HyundaiFlags.CANFD:
      if candidate in (CAR.KIA_CARNIVAL_4TH_GEN) and hda2: ##카니발4th & hda2 인경우에만 BSM이 ADAS에서 나옴.
        ret.extFlags |= HyundaiExtFlags.BSM_IN_ADAS.value
      print(f"$$$$$ CanFD ECAN = {CAN.ECAN}")
      if 0x1fa in fingerprint[CAN.ECAN]:
        ret.extFlags |= HyundaiExtFlags.NAVI_CLUSTER.value
        print("$$$$ NaviCluster = True")
      else:
        print("$$$$ NaviCluster = False")
      if 0x3a0 in fingerprint[CAN.ECAN]: # 0x3a0(928): TPMS
        ret.extFlags |= HyundaiExtFlags.CANFD_TPMS.value
        print("$$$CANFD TPMS")

    else:
      if 1348 in fingerprint[0]:
        ret.extFlags |= HyundaiExtFlags.NAVI_CLUSTER.value
        print("$$$$ NaviCluster = True")
      if 1157 in fingerprint[0] or 1157 in fingerprint[2]:
        ret.extFlags |= HyundaiExtFlags.HAS_LFAHDA.value
      if 913 in fingerprint[0]:
        ret.extFlags |= HyundaiExtFlags.HAS_LFA_BUTTON.value

    print(f"$$$$ enableBsm = {ret.enableBsm}")

    if ret.openpilotLongitudinalControl:
      ret.safetyConfigs[-1].safetyParam |= Panda.FLAG_HYUNDAI_LONG
    if ret.flags & HyundaiFlags.HYBRID:
      ret.safetyConfigs[-1].safetyParam |= Panda.FLAG_HYUNDAI_HYBRID_GAS
    elif ret.flags & HyundaiFlags.EV:
      ret.safetyConfigs[-1].safetyParam |= Panda.FLAG_HYUNDAI_EV_GAS

   
    
    ret.radarTimeStep = 0.05 if params.get_int("EnableRadarTracks") > 0 else 0.02 # SCC(50Hz), radar tracks(20Hz)

    # Car specific configuration overrides

    if candidate == CAR.KIA_OPTIMA_G4_FL:
      ret.steerActuatorDelay = 0.2

    # Dashcam cars are missing a test route, or otherwise need validation
    # TODO: Optima Hybrid 2017 uses a different SCC12 checksum
    ret.dashcamOnly = candidate in {CAR.KIA_OPTIMA_H, }

    return ret

  @staticmethod
  def init(CP, can_recv, can_send):

    Params().put('LongitudinalPersonalityMax', "4")

    if CP.openpilotLongitudinalControl and not (CP.flags & HyundaiFlags.CANFD_CAMERA_SCC.value):
      addr, bus = 0x7d0, 0
      if CP.flags & HyundaiFlags.CANFD_HDA2.value:
        addr, bus = 0x730, CanBus(CP).ECAN
      disable_ecu(can_recv, can_send, bus=bus, addr=addr, com_cont_req=b'\x28\x83\x01')

    params = Params()
    if params.get_int("EnableRadarTracks") > 0 and not CP.flags & HyundaiFlags.CANFD:
      result = enable_radar_tracks(CP, can_recv, can_send)
      params.put_bool("EnableRadarTracksResult", result)

    # for blinkers
    if CP.flags & HyundaiFlags.ENABLE_BLINKERS:
      disable_ecu(can_recv, can_send, bus=CanBus(CP).ECAN, addr=0x7B1, com_cont_req=b'\x28\x83\x01')

def enable_radar_tracks(CP, logcan, sendcan):
  from opendbc.car.isotp_parallel_query import IsoTpParallelQuery
  print("################ Try To Enable Radar Tracks ####################")

  ret = False
  sccBus = 2 if CP.flags & HyundaiFlags.CAMERA_SCC.value else 0
  rdr_fw = None
  rdr_fw_address = 0x7d0 #
  try:
    try:
      query = IsoTpParallelQuery(sendcan, logcan, sccBus, [rdr_fw_address], [b'\x10\x07'], [b'\x50\x07'], debug=True)
      for addr, dat in query.get_data(0.1).items(): # pylint: disable=unused-variable
        print("ecu write data by id ...")
        new_config = b"\x00\x00\x00\x01\x00\x01"
        #new_config = b"\x00\x00\x00\x00\x00\x01"
        dataId = b'\x01\x42'
        WRITE_DAT_REQUEST = b'\x2e'
        WRITE_DAT_RESPONSE = b'\x68'
        query = IsoTpParallelQuery(sendcan, logcan, sccBus, [rdr_fw_address], [WRITE_DAT_REQUEST+dataId+new_config], [WRITE_DAT_RESPONSE], debug=True)
        result = query.get_data(0)
        print("result=", result)
        ret = True
        break
    except Exception as e:
      print(f"Failed : {e}") 
  except Exception as e:
    print("##############  Failed to enable tracks" + str(e))
  print("################ END Try to enable radar tracks")
  return ret
